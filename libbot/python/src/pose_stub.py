#!/usr/bin/python

# file: pose_stub.py
# desc: periodically transmits a pose_state_t message, for applications that
#       need a pose_state_t, but don't necessarily need a meaningful one.

import sys
import time
import math

import lcm

from botlcm.pose_t import pose_t
from lcmtypes.gps_to_local_t import gps_to_local_t

default_latlon = [ 42.362572222, -71.089441666667 ]

lat, lon = default_latlon

pose_msg = pose_t()
gps_msg = gps_to_local_t()

lc = lcm.LCM()

theta = 0
while True:
    pose_msg.utime = int(time.time() * 1000000)
    pose_msg.pos = [ 0, 0, 0 ]
    pose_msg.vel = [ 0, 0, 0 ]
    pose_msg.orientation = [ 1, 0, 0, 0 ]
    pose_msg.rotation_rate = [ 0, 0, 0 ]
    pose_msg.accel = [ 0, 0, 0 ]
    lc.publish ("POSE", pose_msg.encode())

    gps_msg.utime = pose_msg.utime
    gps_msg.local = [ 0, 0, 0 ]
    gps_msg.lat_lon_el_theta = [ lat, lon, 0, theta ]
    gps_msg.gps_cov = [ [ 0.1, 0,   0,   0   ],
                        [ 0,   0.1, 0,   0   ],
                        [ 0,   0,   0.1, 0   ],
                        [ 0,   0,   0,   0.1 ] ]
    lc.publish("GPS_TO_LOCAL", gps_msg.encode())

    theta += math.pi/180
    if theta > 2*math.pi:
        theta = 0

    time.sleep (0.05)
