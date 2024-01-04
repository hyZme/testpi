import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2

class ImuReaderI2CNode(Node):
    def __init__(self):
        super().__init__('imu_reader_i2c')
        self.publisher = self.create_publisher(Imu, 'imu', 10)
        self.i2c_bus = smbus2.SMBus(1)  # Use the correct bus number for your system
        # Add your code to read raw values from the IMU using I2C

def main():
    rclpy.init()
    node = ImuReaderI2CNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
