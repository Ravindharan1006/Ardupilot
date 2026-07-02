from gps_simulation.drone_mavlink import DroneMavlink
import time

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

# Velocity
Vx=1
Vy=1
Vz=1

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
start_time = time.time()

# Target location
drone_target_lat = drone_home_lat + 0.001
drone_target_lon = drone_home_lon + 0.001
drone_target_alt = takeoff_alt 

# while time.time() - start_time < 50:
while True:

    # Set velocity
    # connection.set_velocity(1,1,1)

    # connection.set_gps_target(drone_home_lat,drone_home_lon,takeoff_alt,Vx,Vy,Vz)
    connection.set_gps_target({
        "lat": drone_target_lat,
        "lon": drone_target_lon,
        "alt": drone_target_alt,
        "vx": Vx,
        "vy": Vy,
        "vz": Vz
    })

    curr_gps = connection.get_gps_position()
    
    if curr_gps is None:
        continue

    dlat = abs(curr_gps["lat"] - drone_target_lat)
    dlon = abs(curr_gps["lon"] - drone_target_lon)

    print(f"Error: {dlat}, {dlon}")

    if dlat < 1e-5 and dlon < 1e-5:
        print("Reached target")

        connection.set_velocity(0, 0, 0)
        time.sleep(2)

        # Land
        connection.set_mode("LAND")
        break
    time.sleep(0.1)



