#!/usr/bin/env python2
############################################################################
# Copyright (C) 2020, Technaid S.L
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in bi0nary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Technaid S.L, Inc. nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
###############################################################################

# author Mauricio Echeverri

import rospy
from h3_msgs.msg import State
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT, initial=GPIO.HIGH) # Indicator node running
GPIO.setup(15, GPIO.OUT, initial=GPIO.LOW) # H3-CAN connection status
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW) # H3 is sttoping/recording
GPIO.setup(22, GPIO.OUT, initial=GPIO.LOW) # H3 is walking/Low battery
#GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #test
#GPIO.output(15, GPIO.HIGH)
def callback(data):
   # rospy.loginfo(rospy.get_caller_id() + " I heard %d", data.connection_status)
    GPIO.output(15, data.connection_status)
    if data.recording_status == 1:
        GPIO.output(18, GPIO.HIGH)
    else:
        GPIO.output(18, GPIO.LOW)
    if data.battery_voltage < 17.0:
        GPIO.output(22, GPIO.HIGH)
    else:
        GPIO.output(22, GPIO.LOW)
    
    #print(GPIO.input(13))

def listener():
    rospy.init_node('h3_indicator', anonymous=True)
    rospy.Subscriber("robot_states", State, callback)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        GPIO.output(15, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(22, GPIO.LOW)
        r.sleep()
    rospy.spin()
    GPIO.cleanup()

if __name__ == '__main__':
    listener()
