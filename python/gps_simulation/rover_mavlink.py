from pymavlink import mavutil
from gps_simulation.mavlink_interface import MAVLinkInterface
from pymavlink.dialects.v20 import common as mavlink2

import numpy as np
import math

# from geopy.distance import geodesic


class RoverMavlink(MAVLinkInterface):

    def __init__(self, connection_string="udp:127.0.0.1:14560"):
        super().__init__(connection_string, vehicle_type="rover")
        # self.mav_conn.target_system = 2
        # # self.mav_conn.target_component = mavutil.mavlink.MAV_TYPE_SURFACE_BOAT
        # self.mav_conn.target_component = mavlink2.MAV_TYPE_SURFACE_BOAT
