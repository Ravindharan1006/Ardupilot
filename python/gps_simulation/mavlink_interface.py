#!/usr/bin/env python3
"""
MAVLink interface for communicating with ArduPilot SITL simulation.
"""
import time

from numpy import int32
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

# from gps_simulation.mavlink_interface import ConsoleLogger

class MAVLinkInterface:
    """
    Mavlink interface to communicate with Ardupilot
    Parent mavlink class that contains methods that are common for vehicles
    supported by Ardupilot
    """

    ARMING_TIMEOUT = 120  # seconds

    def __init__(self, connection_string="udp:127.0.0.1:14551", vehicle_type="copter"):
        """
        Initialize the MAVLink interface.

        Args:
            connection_string (str): MAVLink connection string
            vehicle_type (str): Type of vehicle ('copter' or 'ship')
        """
        self.vehicle_type = vehicle_type
        self.connection_string = connection_string
        self.mav_conn = None
        self.connected = False
        self.connect()

    def connect(self):
        """Connect to the MAVLink stream."""

        try:

            print(
                f"Waiting for heartbeat from {self.vehicle_type} in the address {self.connection_string}..."
            )

            self.mav_conn = mavutil.mavlink_connection(self.connection_string)
            
            # waiting for heartbeat
            self.mav_conn.wait_heartbeat()
            self.connected = True
            print("Heartbeat received!")
            print("Target system and component IDs:")
            print(self.mav_conn.target_system, self.mav_conn.target_component)
            print(f"Connected to {self.vehicle_type}")

            return True

        except Exception as e:
            print(f"Connection failed: {str(e)}")
            return False

    def disconnect(self):
        """Disconnect from the MAVLink stream."""

        if self.mav_conn:
            self.mav_conn.close()
        self.connected = False
        print(f"Disconnected from {self.vehicle_type}")

    def send_set_parameter_direct(self, name: str, value: float):
        """
        Send parameter values to the vehicle

        (Args):
            name: parameter name
            value: parameter value
        """
        self.mav_conn.mav.param_set_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            1,
            name.encode("ascii"),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    # Request Message
    def request_message(
        self, message_id, message_name, target_system=None, target_comp=None
    ):
        """
        Request a specific MAVLink message by its ID

        Args:
            message_id (int): MAVLink message ID to request
        """
        if not self.connected:
            print("Not connected to MAVLink stream")
            return None

        if target_comp is None:
            target_comp = self.mav_conn.target_component

        if target_system is None:
            target_system = self.mav_conn.target_system

        # Command to request a specific message
        self.mav_conn.mav.command_long_send(
            target_system,  # Target system
            target_comp,  # Target component
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command
            0,  # Confirmation
            message_id,  # Param 1: Message ID to request
            0,
            0,
            0,
            0,
            0,
            0,  # Additional parameters (set to 0)
        )

        # Wait for and return the requested message
        msg = self.mav_conn.recv_match(type=message_name, blocking=True, timeout=5)

        if not msg:
            return None
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                print(msg.data)
            return None
        else:

            if msg.get_srcSystem() == target_system:
                return msg
            else:
                return None

    def request_message_interval(self, message_id, message_interval, target_comp=None):

        # Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
        # param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
        # param2: 1000000 (Stream interval in microseconds)
        message = self.connection.mav.command_long_encode(
            self.connection.target_system,  # Target system ID
            target_comp,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            0,  # Confirmation
            message_id,  # param1: Message ID to be streamed
            message_interval,  # param2: Interval in microseconds
            0,  # param3 (unused)
            0,  # param4 (unused)
            0,  # param5 (unused)
            0,  # param5 (unused)
            0,  # param6 (unused)
        )

        # Send the COMMAND_LONG
        self.connection.mav.send(message)
        # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
        response = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        if (
            response
            and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        ):
            return response
        else:
            return None

    # def do_prearm_checks(self):
    #     """
    #     Perform pre-arm checks to ensure the vehicle is ready to be armed.

    #     Returns:
    #         bool: True if all checks pass, False otherwise
    #     """
    #     ConsoleLogger.log("Performing pre-arm checks...")

    #     # Check if the vehicle is connected
    #     if not self.connected:
    #         ConsoleLogger.log("Vehicle not connected")
    #         return False

    #     start_time = time.time()
    #     curr_time = start_time
    #     prearm_status = False

    #     while curr_time - start_time <= self.ARMING_TIMEOUT:
    #         # observe the SYS_STATUS messages
    #         message = self.mav_conn.recv_match(
    #             type=dialect.MAVLink_sys_status_message.msgname, blocking=True
    #         )
    #         message = message.to_dict()

    #         # get sensor health
    #         onboard_control_sensors_health = message["onboard_control_sensors_health"]

    #         # get pre-arm healthy bit
    #         prearm_status_bit = (
    #             onboard_control_sensors_health & dialect.MAV_SYS_STATUS_PREARM_CHECK
    #         )
    #         prearm_status = prearm_status_bit == dialect.MAV_SYS_STATUS_PREARM_CHECK

    #         curr_time = time.time()

    #         # check prearm
    #         if prearm_status:

    #             # vehicle can be armable
    #             ConsoleLogger.log("Prearm checks passed. Vehicle is armable")

    #             # break the prearm check loop
    #             break
    #         else:
    #             ConsoleLogger.log("Waiting for prearm checks to complete...")
    #             time.sleep(1)

    #     if curr_time - start_time > self.ARMING_TIMEOUT:
    #         ConsoleLogger.log("Prearm checks timed out")
    #     return prearm_status

    def arm_vehicle(self):
        """
        Arm the vehicle

        Returns:
            bool: True if arming was successful, False otherwise
        """
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,0,0,0,0,0
        )

        # msg = self.mav_conn.recv_match(type='COMMAND_ACK', blocking=True)
        # print(msg)
        # arm_message = self.mav_conn.mav.command_long_encode(
        #     self.mav_conn.target_system,  # Target system
        #     self.mav_conn.target_component,  # Target component
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command
        #     0,  # Confirmation
        #     1,  # Param 1: 1 to arm, 0 to disarm
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,  # Additional parameters
        # )

        # prearm_passed = self.do_prearm_checks()
        # is_armed = False
        # start_time = time.time()
        # curr_time = start_time

        # while prearm_passed and curr_time - start_time <= self.ARMING_TIMEOUT:
        #     ConsoleLogger.log("Arming vehicle...")
        # Send command to arm
        # self.mav_conn.mav.send(arm_message)

        #     # wait COMMAND_ACK message
        #     message = self.mav_conn.recv_match(
        #         type=dialect.MAVLink_command_ack_message.msgname, blocking=True
        #     )
        #     message = message.to_dict()
        #     curr_time = time.time()
        #     # check if the vehicle is armed
        #     if (
        #         message["result"] == dialect.MAV_RESULT_ACCEPTED
        #         and message["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM
        #     ):

        #         # print that vehicle is armed
        #         ConsoleLogger.log("Vehicle is armed!")
        #         is_armed = True
        #         break

        #     else:

        #         # print that vehicle is not armed
        #         ConsoleLogger.log("Waiting for vehicle to be armed...")

        #     # wait some time  
        #     time.sleep(10)

        # if curr_time - start_time > self.ARMING_TIMEOUT:
        #     ConsoleLogger.log("Arming timed out")

        # return is_armed

    def disarm_vehicle(self, force_disarm: bool):
        """
        Disarm the vehicle

        Returns:
            bool: True if disarming was successful, False otherwise
        """

        if force_disarm:
            param2 = 21196
        else:
            param2 = 0
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0 to disarm
            param2,
            0,
            0,
            0,
            0,
            0,
        )

        print("Waiting for motors to be disarmed ...")
        self.mav_conn.motors_disarmed_wait()
        print("Motors disarmed!")

        return True

    def set_mode(self, mode):
        """
        Set vehicle flight mode

        Args:
            mode (str): Flight mode (e.g., 'GUIDED', 'AUTO', 'STABILIZE')

        Returns:
            bool: True if mode change was successful, False otherwise
        """
        # Send MAV_CMD_DO_SET_MODE

        # Use MAV_MODE_FLAG constants

        if mode not in self.mav_conn.mode_mapping():
            print(f"Unknown mode : {mode}")
            return False

        mode_id = self.mav_conn.mode_mapping()[mode]
        self.mav_conn.set_mode(mode_id)

        while True:

            ack_msg = self.mav_conn.recv_match(type="COMMAND_ACK", blocking=True)
            ack_msg = ack_msg.to_dict()

            if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                continue
            print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
            break

        return True

    def set_local_target_position(self, pos_details, config):
        """
        Set local target position in Local NED frame for the vehicle to follow

        Args:
            pos_details (dict): Dictionary containing x, y, z, vx, vy, vz
            bitmask (int): Bitmask to specify which fields are valid
        """
        # print(f"Setting local target position: {pos_details}")
        # print(f"bitmask {config['bitmask']} frame {config['coordinate_frame']}")
        self.mav_conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav_conn.target_system,  # target_system
            self.mav_conn.target_component,  # target_component
            config["coordinate_frame"],  # frame_type (Local NED)
            int(config["bitmask"]),
            pos_details["x"],  # x (North position)
            pos_details["y"],  # y (East position)
            pos_details["z"],  # z (Down position)
            pos_details["vx"],  # vx (North velocity)
            pos_details["vy"],  # vy (East velocity)
            pos_details["vz"],  # vz (Down velocity)
            0,
            0,
            0,
            pos_details["yaw"],  # Yaw
            pos_details["yaw_rate"],
        )

    def get_local_position(self):
        """
        Get the current local position of the vehicle

        Returns:
            dict: Dictionary containing x, y, z, vx, vy, vz
        """
        msg = self.request_message(32, "LOCAL_POSITION_NED")
        if not msg:
            return None

        local_position = {
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "vx": msg.vx,
            "vy": msg.vy,
            "vz": msg.vz,
        }

        return local_position

    def set_gps_target(self, pos_details, bitmask=0b110111111000):
            """
            Set gps target in global frame for the vehicle to follow
            """
            self.mav_conn.mav.send(
                self.mav_conn.mav.set_position_target_global_int_encode(
                    0,
                    self.mav_conn.target_system,
                    self.mav_conn.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    int(bitmask),
                    int(pos_details["lat"] * 1e7),  # X Position in GPS frame
                    int(pos_details["lon"] * 1e7),  # Y Position in GPS frame
                    int(pos_details["alt"]),  #
                    float(pos_details["vx"]),  # vx
                    float(pos_details["vy"]),  # vy
                    float(pos_details["vz"]),  # vz
                    0,
                    0,
                    0,
                    0,  # Yaw
                    0,
                )
            )

    def get_gps_position(self):
        """
        Get the current GPS position of the vehicle

        Returns:
            dict: Dictionary containing latitude, longitude, altitude, and velocity
        """
        msg = self.request_message(33, "GLOBAL_POSITION_INT")
        if not msg:
            return None

        gps_data = {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1000.0,  # Convert from mm to m
            "relative_alt": msg.relative_alt / 1000.0,  # Convert from mm to m
            "vx": msg.vx / 100.0,  # Convert from cm/s to m/s
            "vy": msg.vy / 100.0,  # Convert from cm/s to m/s
            "vz": msg.vz / 100.0,  # Convert from cm/s to m/s
        }

        return gps_data

    # TODO: Check if the below methods can be placed here

    # Velocity
    def set_velocity(self, vx, vy, vz, yaw=0.0):
        """Send velocity command to the drone in Local NED frame."""
        # print(
        #     f"Sending velocity command: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}"
        # )

        # Ensure that the yaw_rate argument is handled properly by MAVLink
        self.mav_conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav_conn.target_system,  # target_system
            self.mav_conn.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame_type (Local NED)
            0b100111000111,
            0,  # x (North position)
            0,  # y (East position)
            0,  # z (Down position)
            vx,  # vx (North velocity)
            vy,  # vy (East velocity)
            vz,  # vz (Down velocity)
            0,
            0,
            0,
            yaw,
            0,  # yaw_rate (rotation velocity around the Z-axis)
        )

    def set_velocity_body(self, vx, vy, vz, yaw=0.0):
        """Send velocity command to the drone in Local NED frame."""
        # print(f"Sending velocity command: vx={vx}, vy={vy}, vz={vz}")

        # Ensure that the yaw_rate argument is handled properly by MAVLink
        self.mav_conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav_conn.target_system,  # target_system
            self.mav_conn.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame_type (Local NED)
            0b100111000111,
            0,  # x (North position)
            0,  # y (East position)
            0,  # z (Down position)
            vx,  # vx (North velocity)
            vy,  # vy (East velocity)
            vz,  # vz (Down velocity)
            0,
            0,
            0,
            yaw,
            0,  # yaw_rate (rotation velocity around the Z-axis)
        )

    def follow_target(self, lat, lon, vx, vy):

        self.mav_conn.mav.follow_target_send(
            int(time.time() * 1e6),  # timestamp (microseconds, int)
            0b00000011,  # est_capabilities (int)
            int(lat * 1e7),  # lat (degE7, int)
            int(lon * 1e7),  # lon (degE7, int)
            float(0),  # alt (float)
            [float(vx), float(vy), 0.0],  # vel (float[3])
            [0.0, 0.0, 0.0],  # acc (float[3])
            [0.0, 0.0, 0.0, 0.0],  # attitude_q (float[4])
            [0.0, 0.0, 0.0],  # rates (float[3])
            [0.0, 0.0, 0.0],  # position_cov (float[3])
            0,  # custom_state (int, not float!)
        )

    # GPS message
    def send_gps_position(self, lat, lon, alt, relative_alt, vx, vy, vz):

        # Send position to drone
        self.mav_conn.mav.global_position_int_send(
            0,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1000),  # it's in mm
            int(relative_alt * 1000),  # it's in mm
            int(vx * 100),  # vx
            int(vy * 100),  # vy
            int(vz * 100),
            65535,
        )

    def set_home_location(self, lat: int32, lon: int32, alt: float):

        message = self.mav_conn.mav.command_int_encode(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            dialect.MAV_FRAME_GLOBAL,
            dialect.MAV_CMD_DO_SET_HOME,
            0,
            0,
            0,  # Use specified location
            0,
            0,
            0,
            lat,
            lon,
            alt,
        )

        self.mav_conn.mav.send(message)

        response = self.mav_conn.recv_match(type="HOME_POSITION", blocking=True)

    def reboot_sitl(self):

        message = self.mav_conn.mav.command_long_encode(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            dialect.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1,  # Reboot autopilot
            0,  # Do nothing for companion computer
            1,  # Reboot given component
            self.mav_conn.target_component,
            0,
            0,  # 0 - Reboot only if safety checks allow, 20190226 - Force reboot
            0,
        )

        self.mav_conn.mav.send(message)


if __name__ == "__main__":
    # Connect to drone
    drone_interface = MAVLinkInterface(
        connection_string="udp:127.0.0.1:14551", vehicle_type="drone"
    )

    # Connect to ship
    ship_interface = MAVLinkInterface(
        connection_string="udp:127.0.0.1:14560", vehicle_type="ship"
    )
