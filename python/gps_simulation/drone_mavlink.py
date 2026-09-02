import time
from pymavlink import mavutil
import numpy as np
import math
import time
from pymavlink.dialects.v20 import common as mavlink2

from gps_simulation.mavlink_interface import MAVLinkInterface


class DroneMavlink(MAVLinkInterface):

    def __init__(self, connection_string="udp:127.0.0.1:14550"):

        super().__init__(connection_string, vehicle_type="copter")
        self.mav_conn.target_system = 1
        self.mav_conn.target_component = 1
        # # self.mav_conn.target_component = mavutil.mavlink.MAV_TYPE_QUADROTOR
        # self.mav_conn.target_component = mavlink2.MAV_TYPE_QUADROTOR

    def takeoff(self, target_altitude, wait_for_altitude=True):
        """
        Send takeoff command

        Args:
            target_altitude (float): Altitude to takeoff to in meters

        Returns:
            bool: True if takeoff command was sent successfully
        """
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # COMMAND
            0,  # Confirmation
            0,  # Minimum pitch (if applicable)
            0,
            0,  # Unused parameters
            0,  # Yaw angle (0 = no rotation)
            0,  # Latitude
            0,  # Latitude, Longitude (0 = current position)
            target_altitude,  # Altitude
        )

        if wait_for_altitude:
            self.altitude_reached_wait(target_altitude)
        return True

    def altitude_reached_wait(self, target_alt):
        """
        Pauses the program execution till the target altitude is reached
        """
        while True:
            msg = self.request_message(
                33,
                "GLOBAL_POSITION_INT",
                self.mav_conn.target_system,
                self.mav_conn.target_component,
            )

            if msg is not None:
                alt = round((msg.relative_alt) / 1000)

                if alt == target_alt:
                    print("Altitude = %dm" % alt)
                    break

            time.sleep(0.1)

    def get_attitude(self):
        """
        Get the attitude of the drone

        Returns:
            dict: Dictionary containing roll, pitch, yaw, roll rate, pitch rate, yaw rate in NED frame
        """

        msg = self.request_message(30, "ATTITUDE")

        if not msg:
            return None

        return msg

    def set_gimbal_pitch_yaw(self, pitch, yaw):
        """
        Set the gimbal pitch and yaw angles

        Args:
            pitch (float): Pitch angle in degrees
            yaw (float): Yaw angle in degrees
        """
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,  # COMMAND
            0,  # Confirmation
            pitch,  # Pitch angle in degrees
            yaw,  # Yaw angle in degrees
            0,
            0,
            0,
            0,
            0,  # Unused parameters
        )

    def drone_yaw_alignment_fix(self):
        """
        Fixes yaw control issue during HITL simulation.
        Sends RC override command to center the yaw control stick
        """
        # Yaw control
        self.mav_conn.mav.rc_channels_override_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            65535,
            65535,
            65535,
            1500,
            65535,
            65535,
            65535,
            65535,
        )

    def is_armed(self):
        hb = self.mav_conn.recv_match(
            type='HEARTBEAT',
            blocking=False
        )

        if hb is None:
            return True   # No new heartbeat yet
        print(f"bool {hb.base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED}")
        return bool(
            hb.base_mode &
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )