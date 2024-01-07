#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32,Float32
from geometry_msgs.msg import Twist
tx_buffer = "+00.00/+00.00/y/n"
def Callback1(value):
    vel = value.data
    rospy.loginfo(vel)
    if vel >= 0:
        tx_buffer[0] = '+'
    else
        tx_buffer[0] = '-'
    tx_buffer[1] = str(vel/10)
    tx_buffer[2] = str(vel%10)
    ser.write(tx_buffer.encode())
    rospy.loginfo(tx_buffer)
def main():
    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','115200'))
    ser = serial.Serial(port = port_name, baudrate = baud, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 

    rospy.init_node('read_encoder', anonymous=True)
    encoder_pub = rospy.Publisher('/encod_val', Float32, queue_size=10)
    rospy.Subscriber('/cmd_vel',Int32, Callback1)
    encoder = Int32()

    while not rospy.is_shutdown():
        encoder_rx = ser.readline().decode('utf-8').strip() #Gia tri doc duoc dang o dang chuoi UART frame truyen +000.00/+000.00/n
        encoder_temp = encoder_rx[:7]
        if encoder_temp[0] = '+':
            encoder = float(encoder_temp[2:7])
        elif encoder_temp[0] = '-':
            encoder = -float(encoder_temp[2:7])
        rospy.loginfo(encoder)
        encoder_pub.publish(encoder)
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass