#!/usr/bin/env python3
"""
Created for Lunar surface preparation project
9. Semester, Robotics, 2024 Fall Semester
Group 962

Copyright 2024 Thomas Schou Sørensen, Julian Witold Wagner and César Zacharie G Hostens

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""
import canopen, os, logging
from canopen import SdoCommunicationError

import rclpy
from rclpy.node import Node
from time import sleep
from roburoc_canopen_interfaces.msg import CANWrite, CANSubscription
from roburoc_canopen_interfaces.srv import CANRead, CANConnection, CANPeriodicTask, CANSubscribe

# Set logging level and output files
logging.basicConfig(level=logging.ERROR)

class RobuROC_Canopen(Node):
    """
    The RobuROC_Canopen class is constructed to communicate with the DPCANTR-040B080 drives installed in the
    RobuROC4. The methods implemented here are defined such that the low level CANBus communication is simplified to the
    extent possible. This particular class is adapted to function within the ROS2 framework and is based upon services
    and topics to provide the requested data. This is due to the integration in ROS2 a standalone node, with customized
    messages and services to reduce redundancy.
    """

    _DRIVE_CONFIG = os.path.join(os.path.dirname(__file__), "AMC_Digiflex_1.0.14.eds")
    CAN_NETWORK = canopen.Network()
    CAN_NODES = []

    def __init__(self):
        """
        """
        super().__init__("ROC_CAN")
        self.declare_parameter('bustype', 'pcan')
        self.declare_parameter('channel', 'PCAN_USBBUS1')
        self.declare_parameter('bitrate', 1000000)

        # Init logging
        self.logger = self.get_logger()

        # Initialize a network object
        self.CAN_NODES = []
        self._CAN_SUBSCRIPTION = []
        self._CAN_PERIODICTASK = {}
        self._PERIODIC_ID = 0
        self._CONNECTED = False

        # Initialize subscriptions
        self.WriteSub = self.create_subscription(CANWrite, '/roburoc/CANWrite', self.Write_CB, 10)

        # Initialize publishers
        self.SubscriptionPub = self.create_publisher(CANSubscription, '/roburoc/CANSubscription', 10)

        # Initialize services
        self.ConnectionSrv = self.create_service(CANConnection, '/roburoc/CANConnection', self.Connection_CB)
        self.PeriodicSrv = self.create_service(CANPeriodicTask, '/roburoc/CANPeriodic', self.PeriodicTask_CB)
        self.SubscribeSrv = self.create_service(CANSubscribe, '/roburoc/CANSubscribe', self.Subscribe_CB)
        self.ReadSrv = self.create_service(CANRead, '/roburoc/CANRead', self.Read_CB)


    def __del__(self):
        self.Disconnect()

        self.destroy_service(self.ConnectionSrv)
        self.destroy_publisher(self.SubscriptionPub)
        self.destroy_service(self.PeriodicSrv)
        self.destroy_service(self.SubscribeSrv)
        self.destroy_service(self.ReadSrv)

        self.destroy_node()

    def Connection_CB(self, request, response):
        """
        Callback function for connection signals, depending on the signal received (Connect or Disconnect), the node
        will call the corresponding method. Each method is called up to 10 times, and after that will return unsuccessful
        :request.std_msgs/String command: "Connect" or "Disconnect", otherwise will return error
        :return: request.success [bool] Whether the connection was successful or not
        """

        if request.command.lower() == "connect":
            bustype = self.get_parameter('bustype').get_parameter_value().string_value
            channel = self.get_parameter('channel').get_parameter_value().string_value
            bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
            for _ in range(10):
                if self.Connect(bustype, channel, bitrate):
                    response.success = True
                    response.node_list = []
                    for node in self.CAN_NODES:
                        response.node_list.append(node.id)
                    return response
            response.success = False
        elif request.command.lower() == "disconnect":
            if self.Disconnect():
                response.success = True
            else:
                response.success = False
    def Connect(self, bustype:str = 'pcan', channel:str = 'PCAN_USBBUS1', bitrate:int = 1000000):
        """
        Connection method for attempting to connect to the CANBUS network specified by bustype and channel at the specified
        bitrate.
        :param bustype: type of CANBus
        :param channel: Connection channel (or object), defined bu the interface to the CANBus
        :param bitrate: The bitrate for the connection
        :return: Bool: true or false for connection
        """

        if not self._CONNECTED:
            try:
                self.CAN_NETWORK.connect(bustype=bustype, channel=channel, bitrate=bitrate)
                self._CONNECTED = True
                if len (self.CAN_NODES) == 0:
                    try:
                        # Need at least four nodes to ensure all drives are connected
                        while len(self.CAN_NETWORK.scanner.nodes) < 4:
                            self.CAN_NETWORK.scanner.search()
                            sleep(1) # Provide the scanner time to find nodes
                        node_list = []
                        for node in self.CAN_NETWORK.scanner.nodes:
                            node = self.CAN_NETWORK.add_node(node, self._DRIVE_CONFIG)
                            self.CAN_NODES.append(node)
                            node_list.append(node.id)
                        self.logger.info(f"Connected to CANBus with nodes: {node_list}")
                    except Exception as e:
                        self.logger.error(f"Unable to initialize nodes, Error: {e}")
                        self._CONNECTED = False
            except Exception as error:
                self.logger.error(f"Unable to connect to CAN Bus, Error: {error}")
                self._CONNECTED = False
            finally:

                return self._CONNECTED
    def Disconnect(self):
        """
        Disconnection method for disconnection the CANBUS network and stopping/ending all
        related subscriptions and periodic tasks
        :return: Bool: True or False for successful disconnection
        """
        if self._CONNECTED:
            try:
                self.CAN_NETWORK.disconnect()
                # It is here assumed that the HeartBeat and Periodic status checks are enabled
                if len(self._CAN_PERIODICTASK) != 0:
                    for task in self._CAN_PERIODICTASK.copy():
                        self._CAN_PERIODICTASK[task].stop()
                        del self._CAN_PERIODICTASK[task]
                if len(self._CAN_SUBSCRIPTION) != 0:
                    for ids in self._CAN_SUBSCRIPTION:
                            self.Unsubscribe(ids)
            except Exception as error:
                self.logger.error(f"Unable to disconnect from CAN Bus, Error: {error}")
            finally:
                self.CAN_NODES.clear()
                self.CAN_NETWORK.scanner.nodes.clear()
                return not self._CONNECTED
        else:
            return True
    def PeriodicTask_CB(self, request, response):
        """
        Callback method for periodic tasks. Adds or removes a periodic task based on the specified command.

        :param request.command: "Add" to add a periodic task or "Remove" to remove one.
        :param request.cobid: (Optional) COBID of PDO to send data to or identify the task to remove.
        :param request.data: (Optional) Data to send to the PDO or identify the task to remove.
        :param request.period: (Optional) Period between transmissions for "Add" command.
        :param response.success: Boolean indicating the success of the command.
        :return: response
        """

        try:
            command = request.command #Has to exist
            cobid = getattr(request, "cobid", None)
            data = getattr(request, "data", None)
            period = getattr(request, "period", None)

            if request.command.lower() == "add":
                if cobid and data and period:
                    kwargs = {k: getattr(request, k) for k in ['cobid', 'data', 'period'] if hasattr(request, k)}
                    response.key = self._PERIODIC_ID
                    self._PERIODIC_ID += 1
                    response.success = self.PeriodicTask(**kwargs)
                    return response
                else:
                    raise ValueError("Missing parameters for Subscription")
            elif request.command.lower() == "remove":
                        response.success = self.RemovePeriodicTask(cobid)
            else:
                raise ValueError(f"Unknown command '{command}' specified.")
        except AttributeError as e:
            self.logger.error(f"Missing attribute in request, error {e}")
            response.success = False
        except ValueError as e:
            self.logger.error(f"Missing parameter(s) for request, error {e}")
            response.success = False
        except Exception as e:
            self.logger.error(f"Unknown error, {e}")
            response.success = False
        finally:
            return response
    def PeriodicTask(self, cobid: int, data: list, period: float = 0.2):
        """
        Adds a CANOpen periodically sent message, e.g. for heartbeat. This task is stored should it later be
        required to be stopped.
        :param cobid: COBID of the intended target
        :param data: Data to be sent periodically
        :param period: The period between messages in seconds
        :return: Bool: True or False for successful periodic task addition
        """
        data = list(data)
        sent_data = None
        if len(data) <= 8:
            # Zero-pad data if necessary
            data += [0] * (8 - len(data))
            data.reverse()
            sent_data = bytearray(data)
        try:
            task = self.CAN_NETWORK.send_periodic(cobid, sent_data, period)
            self._CAN_PERIODICTASK.setdefault(self._PERIODIC_ID, task)
            self._PERIODIC_ID += 1
            return True
        except Exception as error:
            self.logger.error(f"Unable to send periodic task, Error: {error}")
            return False
    def RemovePeriodicTask(self, key: int):
        """
        Removes an existing Periodic task from the _CAN_PERIODICTASK list and stops it. Returns True if the tasks exists
        and is removed. Returns False if task either doesn't exist or is not removed.
        :param key: key for the task in stored dictionary
        :return: Bool success
        """
        try:
            self._CAN_PERIODICTASK[key].stop()
            del self._CAN_PERIODICTASK[key]
            return True
        except Exception as error:
            self.logger.error(f"Unable to remove Periodic task, Error: {error}")
            return False
    def Subscribe_CB(self, request, response):
        """
        Subscribe callback method for subscribing to specific COBIDs. The resulting subscription will provide
        data when the internal values change.
        :param request.command: either "Subscribe" or "Unsubscribe" depending on desired outcome
        :param request.cobid: COBID to subscribe to (Statuswords)
        :param response.success: Bool success of command
        :return: response
        """
        if request.command.lower() == "add":
            response.success = self.Subscribe(request.cobid, self.Generic_callback)
            return response
        elif request.command.lower() == "remove":
            response.success = self.Unsubscribe(request.cobid)
            return response
        else:
            self.logger.error(f"Unknown command {request.command} specified")
            response.success = False
            return response
    def Subscribe(self, COBID: int, callback: callable):
        """
        Adds a CANOpen subscription to a specified COBID, the return value is handled through the specified callback
        :param COBID: COBID of the intended target
        :param callback: Callback function, should contain: (COBID, data, percentage), where percentage corresponds to the amount of the poackage received
        :return: None
        """
        try:
            self.CAN_NETWORK.subscribe(COBID, callback)
            self._CAN_SUBSCRIPTION.append(COBID)
            return True
        except Exception as error:
            self.logger.error(f"Unable to subscribe to {COBID}, Error: {error}")
            return False
    def Unsubscribe(self, COBID: int, callback: callable = None):
        """
        Removes a CANOpen subscription to a specific COBID
        :param COBID: COBID of the intended target
        :param callback: Callback function or None
        :return:
        """
        try:
            if callback == None:
                self.CAN_NETWORK.unsubscribe(COBID)
                self._CAN_SUBSCRIPTION.remove(COBID)
                self.logger.debug(f"Added subscription to {COBID}")
                return True
            else:
                self.CAN_NETWORK.unsubscribe(COBID, callback)
                self._CAN_SUBSCRIPTION.remove(COBID)
                self.logger.debug(f"Added subscription to {COBID} with callback {callback.__name__}")
                return True
        except Exception as e:
            self.logger.error(f"Error unsubscribing from {COBID}, error: {e}")
            return False
    def Generic_callback(self, COBID:int, data:bytearray, rec_percentage:float):
        """
        Generic callback function for returning data from subscriptions
        :param COBID: The COBID of the subscription
        :param data: The data returned from the subscription
        :param rec_percentage: The percentage of the data returned from the subscription
        :return: None
        """
        Subscription_message = CANSubscription()
        Subscription_message.cobid = COBID
        Subscription_message.node_id = [node for node in self.CAN_NODES if COBID >> 7 == node.id][0]
        Subscription_message.data = list(data)
        self.SubscriptionPub.publish(Subscription_message)
    def Write_CB(self, message: CANWrite):
        """
        Write callback method for writing with different methods, depending on the desired target.
        The message structure is assumed to be a list of most significant bit first.
        :param message.command: The desired write command "SDO", "PDO" or "NMT" for Service data object, Process data
        object or network management respectively
        :param message.cobid: If command "PDO" the COBID to write to
        :param message.node_id: If command "SDO" or "NMT" the nodeid to write to
        :param message.indices: If command "SDO" the indices of the object in the object dictionary.
        :param message.data: data specific to the individual commands: "SDO" 4-bit (len=4) list of data, "PDO" 8-bit
        (len=8) list of data, "NMT" 1-bit (len=1) list of data.
        :return: None
        """
        match message.command.lower():
            case "sdo":
                success = self.SDOwrite(message.node_id, message.indices, message.data)
            case "pdo":
                success = self.PDOwrite(message.cobid, message.data)
            case "nmt":
                success = self.NMTwrite(message.data)
            case _:
                self.logger.error(f"Unknown command: {message.command}")
                success = False
        return success # TODO: Test
    def SDOwrite(self, node_id: int, indices: list, data: list):
        """
        Write data to a specified SDO for the given node.
        :param node_id: The ID of the node to write to.
        :param indices: A list of the index and subindex of the SDO as in [index bit 0, index bit 1, subindex bit].
        :param data: The data to be sent (as a list of hexadecimals or ints).
        :return: Bool: True if the write operation was successful, False otherwise.
        """
        data = list(data)
        success = None
        try:
            self.CAN_NODES[node_id].sdo.download(indices[0], indices[1], bytearray(data))
            success = True
        except Exception as e:
            self.logger.error(f"Error writing to SDO {indices[0]}:{indices[1]} in node {node_id}, error: {e}")
            success = False
        finally:
            return success
    def PDOwrite(self, COBID: int, data: list):
        """
        Write data to a mapped RPDO object (Controlword)
        :param COBID: The mapped ID of the RPDO
        :param data: The data to be written (list of hexadecimals or ints).
        :return: Bool: True if successful, False if an error occurred.

        TODO: Adjust to work with nodes
        TODO: If fix is found please adapt it
        NOTE: Does not work
        """
        data = list(data)
        if len(data) <= 8:
            data += [0] * (8 - len(data))
            written_data = bytearray(data)
            try:
                self.CAN_NETWORK.send_message(COBID, written_data)
                return True
            except canopen.SdoCommunicationError as e:
                self.logger.error(f"PDO Write Error: {e}")
                return False
        else:
            self.logger.error(f"Message length over 8 bits is not supported")
            return False
    def NMTwrite(self, ctrlWord: list):
        ctrlWord = list(ctrlWord)
        try:
            self.CAN_NETWORK.nmt.send_command(ctrlWord[0])
            return True
        except Exception as e:
            self.logger.error(f"NMT Write Error: {e}")
            return False
    def Read_CB(self, request, response):
        """
        Read callback method for reading from a SDO (or PDO (NOT SUPPORTED) object in the object library
        :param request.command: either "SDO" or "PDO" (PDO not supported)
        :param request.cobid: If "PDO" COBID of the statusword to read from
        :param request.node_id: If "SDO" id of the node to read from
        :param request.indices: If "SDO" indices of the object in object dictionary
        :param response.indices: Indices read from
        :param response.data: Data returned from the object
        :param response.success: Bool success of read
        :return: response
        """
        if request.command.lower() == "sdo":
            data = self.SDOread(request.node_id, request.indices)
            if data is not None:
                response.success = True
                response.data = data
                response.indices = request.indices
                return response
            else:
                response.success = False
                return response
        if request.command.pdo == "pdo":
            self.logger.error(f"PDO read is not supported")
            response.success = False
            return response
    def SDOread(self, node_id: int, indices: list):
        """
        Retrieve the current data from the specified SDO for the given node.
        :param node_id: The ID of the node to read from.
        :param indices: A list of the index and subindex of the sdo as in [index bit 0, index bit 1, subindex bit], e.g. [0x60, 0x60, 0x00] for index 0x6060 subindex 0x00
        return: Returns data with callback with node id 0x580 + nodeid or None if an error occurs.
        """
        data = None
        try:
            data = self.CAN_NODES[node_id].sdo.upload(indices[0], indices[1]) # TODO: Test
        except SdoCommunicationError as e:
            self.logger.error(f"Error writing to SDO {indices[0]}:{indices[1]} in node {node_id}, error: {e}")
            data = None
        finally:
            return data
    def PDOread(self, node_id: int, index: int, subindex: int = 0, timeout: int = 1, timeout_count: int = 5):
        """
        Read data from an SDO on the specified CANopen node.
        :param node_id: The ID of the node to read from.
        :param index: The index in the object dictionary to read from.
        :param subindex: The subindex in the object dictionary.
        :param timeout: Timeout in seconds for the operation.
        :param timeout_count: Amount of tries to read
        :return: Data read from the node, or raises an exception on failure.
        TODO: Does not work, please update if fix is found
        """
        timeout_counter = 0
        try:
            node = self.CAN_NODES[node_id - 1]
            # Read the data from the specified index and subindex using SDO
            data = node.sdo[index][subindex].read(timeout=timeout)
            return data
        except canopen.SdoCommunicationError as e:
            self.logger.error(f"SDO Read Error: {e}")
            raise
        except TimeoutError:
            self.logger.error(f"SDO read timed out after {timeout} seconds")
            while timeout_counter < timeout_count:
                try:
                    node = self.CAN_NODES[node_id - 1]
                    data = node.sdo[index][subindex].read(timeout=timeout)
                    timeout_counter += 1
                    return data
                except TimeoutError:
                    self.logger.error(f"SDO read timed out after {timeout} seconds")
            raise


def main():
    rclpy.init()
    can = RobuROC_Canopen()

    rclpy.spin(can)

    can.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

