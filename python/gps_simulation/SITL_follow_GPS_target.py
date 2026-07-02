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

while True:
    msg=connection.request_message(33,'GLOBAL_POSITION_INT')
    takeoff_alt=round(msg.relative_alt/1000)
    print("takeoff altitude = %dm" % takeoff_alt)

    # Altitude checking
    if takeoff_alt >= 9:
        print("Reached the takeoff altitude")
        break

# Move forward
print("Moving forward...")

# Target
drone_target_lat = drone_home_lat + 0.0008
drone_target_lon = drone_home_lon + 0.0008

# Initialing 
Kp = 0.2  # gain 
Ki = 0.05
sum_dlat_m = 0
sum_dlon_m = 0

start_time = time.time()

while True:
   
    drone_target_lat += 0.0000009 
    drone_target_lon += 0.0000009 
    drone_target_alt = takeoff_alt

    curr_gps = connection.get_gps_position()
    if curr_gps is None:
        continue

    dlat = drone_target_lat - curr_gps["lat"]
    dlon = drone_target_lon - curr_gps["lon"]

    print(f"dlat: {dlat} dlon: {dlon}")

    R = 6378137
    dlat_m = dlat * (math.pi/180) * R
    dlon_m = dlon * (math.pi/180) * R * math.cos(curr_gps["lat"] * math.pi/180)
    
    print(f"dlat: {dlat_m}m dlon: {dlon_m}m")
    
    distance = math.hypot(dlat_m, dlon_m)
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
    vx = Kp*dlat_m + Ki*sum_dlat_m
    vy = Kp*dlon_m + Ki*sum_dlon_m

    # Velocity limit
    vx = max(min(vx, 10), -10)
    vy = max(min(vy, 10), -10)

    # Send velocity to drone
    connection.set_velocity_body(vx,vy,0)

    if distance < 2:
        print("Reached target")

        connection.set_velocity(0, 0, 0)
        time.sleep(2)

        # Land
        connection.set_mode("LAND")
        break
    
    time.sleep(0.1)