from gps_simulation.drone_mavlink import DroneMavlink
from pymavlink import mavutil
import time
import math
# import matplotlib.pyplot as plt
# import csv
# from datetime import datetime

print("connecting....")
drone_connection = DroneMavlink(connection_string="udpin:127.0.0.1:14550")

# Drone GPS Position
print("Waiting for GPS position...")

drone_home_gps = None                               

while drone_home_gps is None:
    drone_home_gps = drone_connection.get_gps_position()

    if drone_home_gps is None:
        print("Waiting for GPS data...")
        time.sleep(1)

print(f"Home GPS Position: {drone_home_gps}")

drone_home_lat = drone_home_gps["lat"]
drone_home_lon = drone_home_gps["lon"]
drone_home_alt = drone_home_gps["alt"]
drone_home_relative_alt = drone_home_gps["relative_alt"]
drone_home_Vx = drone_home_gps["vx"]
drone_home_Vy = drone_home_gps["vy"]
drone_home_Vz = drone_home_gps["vz"]

print(f"drone_Lat: {drone_home_lat}, drone_lon: {drone_home_lon}, drone_Alt: {drone_home_alt}, drone_Relative Alt: {drone_home_relative_alt}, drone_Vx: {drone_home_Vx}, drone_Vy: {drone_home_Vy}, drone_Vz: {drone_home_Vz}")

#delaying
time.sleep(2)

# Set initial target position
offset_north = 0      # meters
offset_east  = 0     # meters
offset_distance = math.sqrt(offset_north**2 + offset_east**2)   # shouldn't be more than 100 meters
print("Target offset distance = %.2f m" % offset_distance)

# Target location
target_lat = drone_home_lat + offset_north / 111111.0
target_lon = drone_home_lon + offset_east / (111111.0 * math.cos(math.radians(drone_home_lat)))
target_alt = drone_home_alt 
target_relative_alt = drone_home_relative_alt
target_Vx = drone_home_Vx + 0     # North (m/s)
target_Vy = drone_home_Vy + 0     # East (m/s)
target_Vz = drone_home_Vz
dt = 0.1    # time step in seconds

print(f"target_Lat: {target_lat}, target_lon: {target_lon}, target_Alt: {target_alt}, target_Relative Alt: {target_relative_alt}, target_Vx: {target_Vx}, target_Vy: {target_Vy}, target_Vz: {target_Vz}")


# time.sleep(30)

print(drone_connection.is_armed())

while drone_connection.is_armed():
    
    # Drone's current GPS position
    drone_gps = drone_connection.get_gps_position()    
    drone_lat = drone_gps["lat"]
    drone_lon = drone_gps["lon"]
    drone_alt = drone_gps["alt"]
    drone_relative_alt = drone_gps["relative_alt"]
    drone_vx = drone_gps["vx"]
    drone_vy = drone_gps["vy"]
    drone_vz = drone_gps["vz"]

    print(f"drone_lat: {drone_lat}, drone_lon: {drone_lon}, drone_alt: {drone_alt}, drone_relative_alt: {drone_relative_alt}, drone_vx: {drone_vx}, drone_vy: {drone_vy}, drone_vz: {drone_vz}")

    print(f"target_Lat: {target_lat}, target_lon: {target_lon}, target_Alt: {target_alt}, target_Relative Alt: {target_relative_alt}, target_Vx: {target_Vx}, target_Vy: {target_Vy}, target_Vz: {target_Vz}")

    # Convert drone GPS coordinates to NED coordinates relative to the drone's home position
    drone_north = (drone_lat - drone_home_lat) * 111111.0
    drone_east = (drone_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))
   
    # Convert target GPS coordinates to NED coordinates relative to the drone's home position
    target_north = (target_lat - drone_home_lat) * 111111.0
    target_east = (target_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))


    # calculating distance to target (Error)
    dlat = target_lat - drone_lat
    dlon = target_lon - drone_lon
    distance = math.sqrt((dlat * 111111.0) ** 2 + (dlon * 111111.0 * math.cos(math.radians(target_lat))) ** 2)

    print(f"Distance to target: {distance} m")

    
    drone_connection.follow_target(
        target_lat,
        target_lon,
        target_alt,
        target_Vx,
        target_Vy,         
    )

    if not drone_connection.is_armed():
        print("Vehicle is disarmed. Exiting loop.")  
        break

    # Calculate Tracking Errors
    north_error = target_north - drone_north
    east_error = target_east - drone_east
    
    time.sleep(dt)
 