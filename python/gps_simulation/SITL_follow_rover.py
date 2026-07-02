from gps_simulation.drone_mavlink import DroneMavlink
from gps_simulation.rover_mavlink import RoverMavlink

import time
import math

print("connecting....")
drone_connection = DroneMavlink(connection_string="udpin:127.0.0.1:14550")
rover_connection = RoverMavlink(connection_string="udpin:127.0.0.1:14560")

# Guided Mode of drone
print("setting guided mode")
drone_connection.set_mode('GUIDED')

# Guided Mode of rover
print("setting guided mode")
rover_connection.set_mode('GUIDED')

#delaying
time.sleep(2)

# GPS Position of drone
home_gps = drone_connection.get_gps_position()
drone_home_lat = home_gps["lat"]
drone_home_lon = home_gps["lon"]
alt = home_gps["relative_alt"]
print("drone_Lat:", drone_home_lat)
print("drone_Lon:", drone_home_lon)

# GPS Position of rover
home_gps = rover_connection.get_gps_position()
rover_home_lat = home_gps["lat"]
rover_home_lon = home_gps["lon"]
alt = home_gps["relative_alt"]
print("rover_Lat:", rover_home_lat)
print("rover_Lon:", rover_home_lon)
print("Relative Alt:", alt, "m")

# Rover Velocity
rover_Vx=5
rover_Vy=2
rover_Vz=0

# Arming the Drone
print("Arming the drone")
drone_connection.arm_vehicle()

#delaying
time.sleep(2)

# Arming the rover
print("Arming the rover")
rover_connection.arm_vehicle()

#delaying
time.sleep(2)

# Send velocity to drone
rover_connection.set_velocity_body(rover_Vx,rover_Vy,rover_Vz)

#Takeoff
print("Taking off...")
drone_connection.drone_yaw_alignment_fix()
drone_connection.takeoff(10)

# rover_connection.drone_yaw_alignment_fix()

while True:
    # Getting drone location
    msg=drone_connection.request_message(33,'GLOBAL_POSITION_INT')
    takeoff_alt=round(msg.relative_alt/1000)
    print("takeoff altitude = %dm" % takeoff_alt)

    # Altitude checking
    if takeoff_alt >= 9:
        print("Reached the takeoff altitude")
        break

# Move forward
print("Moving forward...")

# # Target
# rover_lat = rover_home_lat + 0.0008
# rover_lon = rover_home_lon + 0.0008

# Initialing 
Kp = 0.2  # gain 
Ki = 0.05
sum_dlat_m = 0
sum_dlon_m = 0

start_time = time.time()

while True:
    # Send velocity to drone
    rover_connection.set_velocity_body(rover_Vx,rover_Vy,rover_Vz)

    # Receiving current rover GPS
    rover_curr_gps = rover_connection.get_gps_position()
    if rover_curr_gps is None:
        continue

    rover_lat = rover_curr_gps["lat"]
    rover_lon = rover_curr_gps["lon"]

    print(f"rover_lat: {rover_lat} rover_lon: {rover_lon}")
   
    time.sleep(0.1)
   
    # rover_lat += 0.0000009 
    # rover_lon += 0.0000009 
    drone_target_alt = takeoff_alt

    drone_curr_gps = drone_connection.get_gps_position()
    if drone_curr_gps is None:
        continue

    dlat = rover_lat - drone_curr_gps["lat"]
    dlon = rover_lon - drone_curr_gps["lon"]

    print(f"dlat: {dlat} dlon: {dlon}")

    R = 6378137
    dlat_m = dlat * (math.pi/180) * R
    dlon_m = dlon * (math.pi/180) * R * math.cos(drone_curr_gps["lat"] * math.pi/180)
    
    print(f"dlat: {dlat_m}m dlon: {dlon_m}m")
    
    distance = math.sqrt(dlat_m**2 + dlon_m**2)
    print(f"Distance between drone and target: {distance}m")

    # integral
    if distance < 20:
        sum_dlat_m += dlat_m
    else:
        sum_dlat_m = 0

    if distance < 20:
        sum_dlon_m += dlon_m
    else:
        sum_dlon_m = 0

    # anti-windup
    sum_dlat_m = max(min(sum_dlat_m, 20), -20)
    sum_dlon_m = max(min(sum_dlon_m, 20), -20)

    # reset near target
    if distance < 2:
        sum_dlat_m = 0

    if distance < 2:
        sum_dlon_m = 0

    print(f"sum_dlat: {dlat_m}m sum_dlon: {dlon_m}m")

    # PI controller
    drone_Vx = Kp*dlat_m + Ki*sum_dlat_m
    drone_vy = Kp*dlon_m + Ki*sum_dlon_m

    # Velocity limit
    vx = max(min(drone_Vx, 10), -10)
    vy = max(min(drone_vy, 10), -10)

    # Send velocity to rover
    rover_connection.set_velocity_body(vx,vy,0)

    # delay
    time.sleep(1)

    # Send velocity to drone
    drone_connection.set_velocity_body(vx,vy,0)

    if distance < 2:
        print("Reached target")

        # Stoping the rover
        rover_connection.set_velocity(0, 0, 0)

        #delaying the rover
        time.sleep(1)

        # Stoping the drone
        drone_connection.set_velocity(0, 0, 0)
        time.sleep(2)

        # Land
        drone_connection.set_mode("LAND")
        break
    
    time.sleep(0.1)