#!/usr/bin/env python

import serial

import rospy

from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse


ser = serial.Serial("/dev/arduino0", baudrate=9600, timeout=1)


def handle_set_suction(req):
    try:
        if req.data:
            ser.write(b"g")
            message = "Turned on"
        else:
            ser.write(b"s")
            message = "Turned off"
    except Exception as e:
        return SetBoolResponse(success=False, message=str(e))
    return SetBoolResponse(success=True, message=message)


def main():
    rospy.init_node("set_suction_server", anonymous=True)
    rospy.Service("set_suction", SetBool, handle_set_suction)
    rospy.spin()


if __name__ == "__main__":
    main()
