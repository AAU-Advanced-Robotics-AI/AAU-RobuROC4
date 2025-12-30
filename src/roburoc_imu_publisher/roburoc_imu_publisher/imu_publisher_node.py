import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import serial
import math

'''
Following documentation on sensor_msgs/Imu.msg in the ros docs.
'''

class IMUPublisherNode(Node):
    def __init__(self):
        super().__init__('roburoc_imu_publisher_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Set your default serial port here
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to IMU on {serial_port} at {baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            exit(1)

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10 Hz publish rate

    def publish_imu_data(self):
        try:
            line = self.serial_connection.readline().decode('utf-8').strip()
            # Example raw data: "gx=0.0681,gy=0.2355,gz=-0.1903,d1=1478,d2=3386"
            data = self.parse_imu_data(line)
            if data:
                imu_msg = Imu()

                # Fill header
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'base_link'     # need to create an imu frame

                # Fill orientation (if unavailable, leave as default)
                imu_msg.orientation = Quaternion()
                imu_msg.orientation_covariance[0] = -1  # Mark as unknown

                # Fill angular velocity
                imu_msg.angular_velocity = Vector3(x=data['gx'], y=data['gy'], z=data['gz'])
                imu_msg.angular_velocity_covariance = [0.0, 0.0, 0.0,  # covariance assumed to be unknown
                                                       0.0, 0.0, 0.0,
                                                       0.0, 0.0, 0.0]

                # Fill linear acceleration
                imu_msg.linear_acceleration = Vector3(x=0.0,y=0.0, z=0.0)        # Unknown, hardset to 0
                imu_msg.linear_acceleration_covariance = [0.0] * 9  # Unknown

                # Publish the message
                self.imu_pub.publish(imu_msg)
                # self.get_logger().info(f"Published IMU data: {imu_msg}")
        except Exception as e:
            self.get_logger().error(f"Error reading or parsing IMU data: {e}")

    def parse_imu_data(self, line):
        try:
            parts = line.split(',')
            data = {}
            for part in parts:
                key, value = part.split('=')
                data[key.strip()] = float(value.strip())
            return data
        except ValueError:
            self.get_logger().warning(f"Failed to parse line: {line}")
            return None

def main(args=None):
    rclpy.init(args=args)
    roburoc_imu_publisher_node = IMUPublisherNode()
    rclpy.spin(roburoc_imu_publisher_node)
    roburoc_imu_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()