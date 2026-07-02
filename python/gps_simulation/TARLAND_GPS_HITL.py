from gps_simulation.drone_mavlink import DroneMavlink
import time
import math

print("connecting....")
connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")

# Guided Mode
print("setting guided mode")
connection.set_mode('GUIDED')

#delaying
time.sleep(2)

# GPS Position
home_gps = connection.get_gps_position()
drone_home_lat = home_gps["lat"]
drone_home_lon = home_gps["lon"]
alt = home_gps["relative_alt"]
print("Lat:", drone_home_lat)
print("Lon:", drone_home_lon)
print("Relative Alt:", alt, "m")

# Arming the Drone
print("Arming the drone")
connection.arm_vehicle()

#delaying
time.sleep(2)

#Takeoff
print("Taking off...")
connection.drone_yaw_alignment_fix()
connection.takeoff(10)

