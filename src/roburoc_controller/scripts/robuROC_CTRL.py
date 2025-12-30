#!/usr/bin/env python3
from sys import byteorder

from utils.CTW import CTW
from utils.COBID import COBID
from roburoc_canopen_interfaces.msg import CANWrite, CANSubscription
from roburoc_canopen_interfaces.srv import CANRead, CANConnection, CANPeriodicTask, CANSubscribe
import rclpy
import rclpy.subscription, logging, time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from time import sleep

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    handlers=[logging.FileHandler("CAN_Logs"),
                              logging.StreamHandler()])

class RobuROC_CTRL(Node):
    """
    RobuROC controller node, abstracts the control of the RobuROC 4 platform futher, by utilising the services and other
    interfaces from the roburoc_canopen node and any navigation node that it might be attached to.
    """
    # Initialize fields to contain periodic status updates
    _CURRENT_PERIODIC = [None, None, None, None]
    _VELOCITY_PERIODIC = [None, None, None, None]
    _STATUS_PERIODIC = [None, None, None, None]
    _TEMPORARY_PERIODIC = [None, None, None, None]

    # Setup fields for status and debugging
    _CONNECTED = False
    _PERIODIC = False
    _HEARTBEAT = False

    # Public variables
    MAX_SPEED = 2  # Meters per second
    WHEEL_RADIUS = 0.28  # Meters

    # Constants
    _SCALE_CURRENT = pow(2, 13) / 40.0  # to Amps
    _SCALE_SETCURRENT = pow(2, 15) / 40.0  # to Amps
    _SCALE_VELOCITY = ((pow(2, 17) / (2 * 20000) * pow(2, 19)) / 1000) * 32  # To RPM
    _SCALE_RPM_TO_MPS = ((2.0 * 3.14) / 60.0) * WHEEL_RADIUS

    _DEADMAN_SWITCH = False

    def __init__(self):
        super().__init__('ROC_CTRL')
        self.logger = self.get_logger()

        # Setup canopen help modules
        self._CTW = CTW()
        self._COBID = COBID()

        # Set up publisher
        self.WritePub = self.create_publisher(CANWrite, '/roburoc/CANWrite', 10)
        self.SpeedPub = self.create_publisher(CANSubscribe, '/roburoc/Velocity', 10)

        # Set up service clients
        self.ConnectionSrvCli = self.create_client(CANConnection, '/roburoc/CANConnection')
        self.ConnectionSrvCli.wait_for_service()
        self.PeriodicSrvCli = self.create_client(CANPeriodicTask, '/roburoc/CANPeriodic')
        self.PeriodicSrvCli.wait_for_service()
        self.SubscribeSrvCli = self.create_client(CANSubscribe, '/roburoc/CANSubscribe')
        self.SubscribeSrvCli.wait_for_service()
        self.ReadSrvCli = self.create_client(CANRead, '/roburoc/CANRead')
        self.ReadSrvCli.wait_for_service()

        # Set up subscriptions
        self.SubscriptionSub = self.create_subscription(CANSubscription, '/roburoc/CANSubscription', self.Subscription_CB, 10)
        self.VelSubJoy = self.create_subscription(Joy, '/joy', self.setSpeed, 10)
        self.VelSubCmd = self.create_subscription(Twist, '/roburoc/cmd_vel', self.setSpeed, 10)

        # Set up interface variables
        self._CAN_NODES = []

    def setup(self):
        """
        Setup for the RobuROC-4 (pre-configured) CANBus
        :return: None
        """
        # If the system is already initialized, stop it and reset everything
        request = CANConnection.Request()
        request.command = "Connect"
        while not self._CONNECTED:
            response = self.ConnectionSrvCli.call_async(request)
            rclpy.spin_until_future_complete(self, response)
            self._CONNECTED = response.result().success
            self._CAN_NODES.extend(response.result().node_list)
            self.logger.info(f"Connected to nodes: {self._CAN_NODES}")
        if self._CONNECTED == True:
            self.NMT_Write([self._CTW.RESET])
            self.NMT_Write([self._CTW.ENABLE_OP])
            sleep(2)

            # self.NMT_Write([self._CTW.ENABLE_OP])
            self.NMT_Write([self._CTW.ENABLE])
            for node in self._CAN_NODES:
                self.SDO_Write(node-1, [0x2062, 0x04], [0xFF, 0xFF, 0xFF, 0x0F])
                self.SDO_Write(node-1, [0x6040, 0x00], [0x0F, 0x00, 0x00, 0x00])

            if not self._PERIODIC:
                self.InitPeriodic()
            if not self._HEARTBEAT:
                self.InitHeartbeat()

        else:
            Node.get_logger(self).error(f"Unable to connect to CANBUS")
    def InitHeartbeat(self, heartbeat_time_ms: float = 0.2):
        """
        Initialize heartbeat messages where the host (Node 0) is the producer
        and the nodes are consumers.

        :param heartbeat_time_ms: Time interval in seconds for sending heartbeat messages.
        """
        success = self.AddPeriodicTask(0x700+self._COBID.HOST, [0], heartbeat_time_ms)
        if success:
            self._HEARTBEAT = True
        else:
            self._HEARTBEAT = False
    def InitPeriodic(self):
        """
        Initialize periodic read TPDO messages, such that it returns with periodic updates to the state of the drives
        and motors.
        :return: None
        """
        success_vel = []
        success_cur = []
        success_heart = []

        for node in self._CAN_NODES:
            success_vel.append(self.Subscribe(self._COBID.ACT_VELOCITY.ALL[node-1]))
            success_cur.append(self.Subscribe(self._COBID.ACT_CURRENT.ALL[node-1]))
            success_heart.append(self.Subscribe(self._COBID.HEARTBEAT.ALL[node-1]))

        self.logger.debug(f"Successfully subcribed to Velocity for nodes [1, 2, 3, 4]:{success_vel}, Current: {success_cur}, Heartbeat: {success_heart}")
        if all(success_vel) and all(success_cur) and all(success_heart):
            self._PERIODIC = True
        else:
            self._PERIODIC = False

    def setSpeed(self, message):
        """
        Set the speed of the individual wheels based on the type of message received.

        - If Joy message (Joystick message) and has Non-zero data:
            - Utilize gamepad_control method
        - If Twist message:
            - Utilize navigation_control method
        - If Joy message and only has zero-data or is neither type of message:
            - Stop all motion

        :param message: Input message, expected to by of type Joy (joystick) or Twist (Navigation).
        :return: None
        """
        try:
            if type(message) == type(Joy()) and any(message.buttons):
                self.gamepad_control(message)
            elif type(message) == type(Twist()):
                self.navigation_control(message)
            elif type(message) == type(Joy()) and not any(message.buttons):
                time.sleep(0.01) # Ensures that the CANBus is not overloaded
                self.SDO_Write(0, [0x60FF, 0x00], [0x00])
                self.SDO_Write(1, [0x60FF, 0x00], [0x00])
                self.SDO_Write(2, [0x60FF, 0x00], [0x00])
                self.SDO_Write(3, [0x60FF, 0x00], [0x00])
            elif type(message) != type(Joy()) and type(message) != type(Twist()):
                time.sleep(0.01)  # Ensures that the CANBus is not overloaded
                self.SDO_Write(0, [0x60FF, 0x00], [0x00])
                self.SDO_Write(1, [0x60FF, 0x00], [0x00])
                self.SDO_Write(2, [0x60FF, 0x00], [0x00])
                self.SDO_Write(3, [0x60FF, 0x00], [0x00])
        except Exception as e:
            self.logger.error(f"Unknown message format; error {e}")

    def gamepad_control(self, message):
        """
        Gamepad control method, utilizes the DUALSHOCK controller for operating the platform.
        The implementation is aimed at security, thereby meant that all functionality that provides movement requires
        the use of a dead-man switch. The following are the available commands:

        - '▲' + Left Stick: Set wheel speeds based on linear and angular axes. ('▲' is the dead-man switch)
        - `■`: Brakes the vehicle.
        - `⬤`: Recovers from an error state or after re-enabling the security measures
        :param message:
        :return: None
        """

        if message.buttons[2] == 1: # '▲'
            left, right = None, None
            left = round(message.axes[1] - message.axes[0]/ 4, 4) * 1.0
            right = -round(message.axes[1] + message.axes[0]/ 4, 4) * 1.0
            vel_MPS = int(left * (self._SCALE_VELOCITY / self._SCALE_RPM_TO_MPS))
            vel2_MPS = int(right * (self._SCALE_VELOCITY / self._SCALE_RPM_TO_MPS))
            vel_MPS = list(bytearray(vel_MPS.to_bytes(4, byteorder='little', signed=True)))
            vel2_MPS = list(bytearray(vel2_MPS.to_bytes(4, byteorder='little', signed=True)))
            self.SDO_Write(0, [0x60FF, 0x00], vel_MPS)
            self.SDO_Write(1, [0x60FF, 0x00], vel2_MPS)
            self.SDO_Write(2, [0x60FF, 0x00], vel2_MPS)
            self.SDO_Write(3, [0x60FF, 0x00], vel_MPS)
        elif message.buttons[1] == 1: # `■`
            self.SDO_Write(0, [0x60FF, 0x00], [0x00])
            self.SDO_Write(1, [0x60FF, 0x00], [0x00])
            self.SDO_Write(2, [0x60FF, 0x00], [0x00])
            self.SDO_Write(3, [0x60FF, 0x00], [0x00])
            self.brake()
        elif message.buttons[3] == 1: # `⬤`
            self.recover()

    def navigation_control(self, message):
        """
        Navigation control method, utilising the internal linear and angular velocity commands embedded in most
        algorithms and packages in ROS2.

        TODO: Test (Not done)
        :param message:
        :return:
        """
        if message.angular.z < 0.4:
            left = round(message.linear.x + message.angular.z / 4, 4) * 2
            right = round(message.linear.x - message.angular.z / 4, 4) * 2
        else:
            turn_magnitude = round(message.angular.z / 2, 4)
            left = turn_magnitude
            right = -turn_magnitude
        vel_MPS = int(left * (self._SCALE_VELOCITY / self._SCALE_RPM_TO_MPS))
        vel2_MPS = int(-right * (self._SCALE_VELOCITY / self._SCALE_RPM_TO_MPS))
        vel_MPS = list(bytearray(vel_MPS.to_bytes(4, byteorder='little', signed=True)))
        vel2_MPS = list(bytearray(vel2_MPS.to_bytes(4, byteorder='little', signed=True)))
        self.SDO_Write(0, [0x60FF, 0x00], vel_MPS)
        self.SDO_Write(1, [0x60FF, 0x00], vel2_MPS)
        self.SDO_Write(2, [0x60FF, 0x00], vel2_MPS)
        self.SDO_Write(3, [0x60FF, 0x00], vel_MPS)


    def brake(self):
        """
        Brakes the RobuROC4 by setting velocities to zero and transitioning nodes to Quickstop state.
        :return: None
        """

        self.SDO_Write(0, [0x6040, 0x00], [0x02, 0x00, 0x00, 0x00])
        self.SDO_Write(1, [0x6040, 0x00], [0x02, 0x00, 0x00, 0x00])
        self.SDO_Write(2, [0x6040, 0x00], [0x02, 0x00, 0x00, 0x00])
        self.SDO_Write(3, [0x6040, 0x00], [0x02, 0x00, 0x00, 0x00])
        for node in self._CAN_NODES:
            self.SDO_Write(node - 1, [0x2062, 0x04], [0xFF, 0xFF, 0xFF, 0x0F])
            self.SDO_Write(node-1, [0x6040, 0x00], [0x0F, 0x00, 0x00, 0x00])

    def recover(self):
        """
        Recovers the RobuROC4 from a software-related issue by resetting and enabling CAN nodes.
        This method does not reset the emergency stop module.
        :return: None
        """
        self.NMT_Write([self._CTW.RESET])
        self.NMT_Write([self._CTW.ENABLE_OP])
        sleep(0.5)
        self.NMT_Write([self._CTW.ENABLE])
        for node in self._CAN_NODES:
            self.SDO_Write(node-1, [0x6040, 0x00], [0x0F, 0x00, 0x00, 0x00])

    def AddPeriodicTask(self, COBID: int, data: list, period: float):
        """
        Adds a periodic CAN message task to be executed at a specified interval.
        :param COBID: The COB-ID of the CAN message.
        :param data: The data payload of the CAN message.
        :param period: Interval in seconds for the task.
        """
        request = CANPeriodicTask.Request()
        request.command = "Add"
        request.cobid = COBID # 0x700 + COBID.HOST
        request.data = data
        request.period = period
        response = self.PeriodicSrvCli.call_async(request)
        rclpy.spin_until_future_complete(self, response)
        return response.result().success
    def RemovePeriodicTask(self, COBID:int):
        """
        Removes a periodic CAN message task.COBID:
        :param COBID: The COB-ID of the CAN message task to be removed.:
        :return: True if successful, False otherwise.
        """
        request = CANPeriodicTask.Request()
        request.command = "Remove"
        request.cobid = COBID  # 0x700 + COBID.HOST
        response = self.PeriodicSrvCli.call_async(request)
        rclpy.spin_until_future_complete(self, response)
        return response.result().success
    def Subscribe(self, COBID:int):
        """
        Subscribe "override" method, to subscribe to specific COBIDs in the CANBUS node
        :param COBID: COBID of the desired object
        :return: None
        """
        Subscribe_req = CANSubscribe.Request()
        Subscribe_req.command = "Add"
        Subscribe_req.cobid = COBID
        response = self.SubscribeSrvCli.call_async(Subscribe_req)
        rclpy.spin_until_future_complete(self, response)
        return response.result().success
    def Unsubscribe(self, COBID:int):
        """
        Subscribes to a specific CANBUS COB-ID.


        :param COBID: The COB-ID of the CAN message to subscribe to.
        :return: True if successful, False otherwise.
        """
        Subscribe_req = CANSubscribe.Request()
        Subscribe_req.command = "Remove"
        Subscribe_req.cobid = COBID
        response = self.SubscribeSrvCli.call_async(Subscribe_req)
        rclpy.spin_until_future_complete(self, response)
        return response.result().success

    def NMT_Write(self, data: list):
        """
        Sends a Network Management (NMT) command to the CANBUS node.

        :param data: NMT command data.
        :return: None
        """

        NMT_msg = CANWrite()
        NMT_msg.command = "NMT"
        NMT_msg.data = data
        self.WritePub.publish(NMT_msg)
    def SDO_Write(self, node_id: int, indices: list, data: list):
        """
        Sends a Service Data Object (SDO) write command to the CANBUS node.

        :param node_id: Node ID of the target device.
        :param indices: List of indices/sub-indices for the SDO operation.
        :param data: Data payload for the SDO write.
        :return: None
        """
        SDO_msg = CANWrite()
        SDO_msg.command = "SDO"
        SDO_msg.node_id = node_id
        SDO_msg.indices = indices
        SDO_msg.data = data
        self.WritePub.publish(SDO_msg)
    def PDO_Write(self, COBID: int, node_id: int, data: list):
        """
        Sends a Process Data Object (PDO) write command to the CANBUS node.

        :param COBID: COB-ID of the PDO message.
        :param node_id: Node ID of the target device.
        :param data: Data payload for the PDO write.
        :return: None
        TODO:FIX
        """

        PDO_msg = CANWrite()
        PDO_msg.command = "PDO"
        PDO_msg.cobid = COBID
        PDO_msg.node_id = node_id
        PDO_msg.data = data
        self.WritePub.publish(PDO_msg)
    def SDORead(self, node_id: int, indices: list):
        """
        Sends a Service Data Object (SDO) read command to the CANBUS node and returns the response.

        :param node_id: Node ID of the target device.
        :param indices: List of indices/sub-indices for the SDO read operation.
        :return: The result of the SDO read operation.
        """
        SDO_Read = CANRead.Request()
        SDO_Read.command = "SDO"
        SDO_Read.node_id = node_id
        SDO_Read.indices = indices
        response = self.SubscribeSrvCli.call_async(SDO_Read)
        rclpy.spin_until_future_complete(self, response)
        return response.result()

    def Subscription_CB(self, message):
        """
        Generic callback for storing data recieved from the RobuROC 4 drivers.
        This includes:
        - Velocity readings
        - Current readings
        - Heartbeat (Status) readings
        These are stored for future use in the instantiated object.

        :param message:
        :return: None
        """
        if message.cobid in self._COBID.ACT_CURRENT.ALL:
            current = int.from_bytes(message.data, 'little', signed=True)
            current_amps = current * (pow(2, 13) / 40.0) # to amps
            self._CURRENT_PERIODIC[message.node_id-1] = current_amps
        else:
            self.logger.log(logging.ERROR, f"COBID {message.cobid} not recognised")

        if message.cobid in self._COBID.ACT_VELOCITY.ALL:
            velocity = int.from_bytes(message.data, 'little', signed=True)
            velocity_mps = velocity / ((((pow(2, 17) / (2 * 20000) * pow(2, 19)) / 1000) * 32) * (((2.0 * 3.14) / 60.0) * 0.28)) # To MPS
            self._VELOCITY_PERIODIC[message.node_id-1] = velocity_mps
            self.SpeedPub.publish(message)
        else:
            self.logger.log(logging.ERROR, f"COBID {message.cobid} not recognized")

        if message.cobid in self._COBID.HEARTBEAT.ALL:
            status = int.from_bytes(message.data, 'little', signed=True)
            if status == 0x85 or status == 0x05:
                self._STATUS_PERIODIC[message.node_id-1] = 'OPERATIONAL'
            elif status == 0x84 or status == 0x04:
                self._STATUS_PERIODIC[message.node_id-1] = 'STOPPED'
            elif status == 0xFF or status == 0x7F:
                self._STATUS_PERIODIC[message.node_id-1] = 'PRE-OPERATIONAL'
            else:
                self._STATUS_PERIODIC[message.node_id-1] = 'UNKNOWN'
def main():
    rclpy.init()
    ctrl = RobuROC_CTRL()
    ctrl.setup()

    try:
        rclpy.spin(ctrl)
    except Exception as e:
        print(e)
    finally:
        ctrl.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()