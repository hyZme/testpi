import math
import time
import smbus
import struct
import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_about_axis
from mpu9250_i2c import*
import numpy as np

ADDR1 = None
ADDR = None
bus = None
bus1 = None
IMU_FRAME = None
MAG_FRAME = None

# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word1(adr1):
   # high = bus1.read_byte_data(ADDR1, adr1)
   #low = bus1.read_byte_data(ADDR1, adr1+1)
    high = bus1.read_byte_data(ADDR1, adr1)
    low = bus1.read_byte_data(ADDR1, adr1-1)
   # val = (high << 8) + low
    val = ((high << 8) | low)
   # if (val > 32768):
    #   value -= 65536
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def read_word_2c1(adr1):
    val = read_word1(adr1)
    if (val > 32768):
        val -= 65536
   # if (val >= 0x8000):
    #    return -((65535 - val) + 1)
   #else:
        return val

def publish_mag(timer_event):
    mx,my,mz= AK8963_conv()
    mag_msg = MagneticField()
    mag_msg.header.frame_id = MAG_FRAME
    mag_msg.magnetic_field.x = mx
    mag_msg.magnetic_field.y = my
    mag_msg.magnetic_field.z = mz
    mag_msg.header.stamp = rospy.Time.now()
    mag_pub.publish(mag_msg)

accel_x_1 = accel_x_2 = accel_x_3 = 0
accel_y_1 = accel_y_2 = accel_y_3 = 0
accel_z_1 = accel_z_2 = accel_z_3 = 0

gyro_x_1 = gyro_x_2 = gyro_x_3 = 0
gyro_y_1 = gyro_y_2 = gyro_y_3 = 0
gyro_z_1 = gyro_z_2 = gyro_z_3 = 0

accel_x_list = [] 
accel_y_list = []
accel_z_list = []
gyro_x_list = []
gyro_y_list = []
gyro_z_list = []

accel_x_offset = 0
accel_y_offset = 0
accel_z_offset = 0
gyro_x_offset = 0
gyro_y_offset = 0 
gyro_z_offset = 0

flag_calib = 0

def publish_imu(timer_event):
    global accel_x_1,accel_x_2,accel_x_3,accel_y_1,accel_y_2,accel_y_3,accel_z_1,accel_z_2,accel_z_3
    global gyro_x_1,gyro_x_2,gyro_x_3,gyro_y_1,gyro_y_2,gyro_y_3,gyro_z_1,gyro_z_2,gyro_z_3
    global accel_x_list, accel_y_list, accel_z_list, gyro_x_list, gyro_y_list, gyro_z_list
    global accel_x_offset, accel_y_offset, accel_z_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset
    global flag_calib

    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
    #low-pass filter
    accel_x_1 = 0.9753*accel_x_2 + 0.02469*accel_x_3
    accel_x_2 = accel_x_1
    accel_x_3 = accel_x

    accel_y_1 = 0.9753*accel_y_2 + 0.02469*accel_y_3
    accel_y_2 = accel_y_1
    accel_y_3 = accel_y

    accel_z_1 = 0.9753*accel_z_2 + 0.02469*accel_z_3
    accel_z_2 = accel_z_1
    accel_z_3 = accel_z
    
    accel_x_list.append(accel_x_1*9.8)
    accel_y_list.append(accel_y_1*9.8)
    accel_z_list.append(accel_z_1*9.8)

    # Read the gyro vals
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
    # low-pass filtered
    gyro_x_1 = 0.9753*gyro_x_2 + 0.02469*gyro_x_3
    gyro_x_2 = gyro_x_1
    gyro_x_3 = gyro_x

    gyro_y_1 = 0.9753*gyro_y_2 + 0.02469*gyro_y_3
    gyro_y_2 = gyro_y_1
    gyro_y_3 = gyro_y

    gyro_z_1 = 0.9753*gyro_z_2 + 0.02469*gyro_z_3
    gyro_z_2 = gyro_z_1
    gyro_z_3 = gyro_z
    
    gyro_x_list.append(gyro_x_1*0.0174)
    gyro_y_list.append(gyro_y_1*0.0174)
    gyro_z_list.append(gyro_z_1*0.0174)

    if len(gyro_x_list) <1000:
        rospy.logwarn("(%d samples): Autocalibing is being implemented" %(len(gyro_x_list)))
    elif len(gyro_x_list) == 1000:
        gyro_x_offset = np.average(gyro_x_list)
        gyro_y_offset = np.average(gyro_y_list)
        gyro_z_offset = np.average(gyro_z_list)
        accel_x_offset = np.average(accel_x_list)
        accel_y_offset = np.average(accel_y_list)
        accel_z_offset = np.average(accel_z_list)
    
    flag_calib = 1

    if flag_calib ==1:
        imu_msg.linear_acceleration.x = accel_x_1*9.81 - accel_x_offset
        imu_msg.linear_acceleration.y = accel_y_1*9.81 - accel_y_offset
        imu_msg.linear_acceleration.z = accel_z_1*9.81 + (9.81-accel_x_offset)

        imu_msg.angular_velocity.x = gyro_x_1*0.0174 - gyro_x_offset
        imu_msg.angular_velocity.y = gyro_y_1*0.0174 - gyro_y_offset
        imu_msg.angular_velocity.z = gyro_z_1*0.0174 - gyro_z_offset

        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.orientation_covariance = [2.6e-07, 0.0,0.0,0.0, 2.6e-07, 0.0,0.0,0.0,0.0]
        imu_msg.angular_velocity_covariance = [2.5e-04,0.0,0.0,0.0,2.5e-04,0.0,0.0,0.0,2.5e-04]
        imu_msg.linear_acceleration_covariance = [2.5e-04,0.0,0.0,0.0,2.5e-04,0.0,0.0,0.0,2.5e-04]
        imu_pub.publish(imu_msg)

mag_pub = None
imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    
    bus1 = smbus.SMBus(rospy.get_param('~bus1', 1))
    ADDR1 = rospy.get_param('~device_address', 0x0C)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)
    if type(ADDR1) == str:
        ADDR1 = int(ADDR1, 16)
        
    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
    bus1.write_byte_data(ADDR1, PWR_MGMT_1, 0)
    mag_pub = rospy.Publisher('imu/mag', MagneticField,queue_size=10)
    imu_pub = rospy.Publisher('imu/data_raw', Imu,queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    mag_timer = rospy.Timer(rospy.Duration(0.02), publish_mag)
    rospy.spin()