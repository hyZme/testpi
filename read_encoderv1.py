#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
tx_buffer = "+00.00/+00.00/y/n"
def Callback1(value):
    setpoint1 = value;
    if(setpoint1 >= 0):
        tx_buffer[0] = '+'
    else if(setpoint1 < 0):
        tx_buffer[0] = '-'
    setpoint1[1] = value/10 + 48
    setpoint1[2] = value%10 + 48
    ser.write(tx_buffer)
    rospy.loginfo(tx_buffer)
def main():
    # global x
    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    print(port_name)
    baud = int(rospy.get_param('~baud','115200'))
    ser = serial.Serial(port = port_name, baudrate = baud, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 

    rospy.init_node('read_encod', anonymous=True)
    encoder_pub = rospy.Publisher('/encod_val', Int32, queue_size=10)
    rospy.Subscriber('/cmd_vel',Int32, Callback1)
    encoder = Int32()

    while not rospy.is_shutdown():
        #encoder = ser.read()
        encoder = ser.read()
        encoder_pub.publish(encoder)
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass