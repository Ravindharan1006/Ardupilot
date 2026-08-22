from gps_simulation.drone_mavlink import DroneMavlink
import time
import math
import matplotlib.pyplot as plt
import csv
from datetime import datetime

print("connecting....")
drone_connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")
target_connection = DroneMavlink(connection_string="udp:127.0.0.1:14560")

# Guided Mode
print("setting guided mode")
# drone_connection.set_mode('GUIDED')

#delaying
time.sleep(2)

# Drone GPS Position
drone_home_gps = drone_connection.get_gps_position()
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

# Arming the Drone
print("Arming the drone")
# drone_connection.arm_vehicle()

#delaying
time.sleep(2)

#Takeoff
print("Taking off...")
# drone_connection.drone_yaw_alignment_fix()
# drone_connection.takeoff(10)

while True:
    msg=drone_connection.request_message(33,'GLOBAL_POSITION_INT')
    takeoff_alt=round(msg.relative_alt/1000)
    print("takeoff altitude = %dm" % takeoff_alt)

    # Altitude checking
    if takeoff_alt >= 9:
        print("Reached the takeoff altitude")

        break

# Target GPS Position
target_gps = drone_connection.get_gps_position()
target_lat = target_gps["lat"]
target_lon = target_gps["lon"]
target_alt = target_gps["alt"]
target_relative_alt = target_gps["relative_alt"]
target_Vx = target_gps["vx"]
target_Vy = target_gps["vy"]
target_Vz = target_gps["vz"]

print(f"target_Lat: {target_lat}, target_lon: {target_lon}, target_Alt: {target_alt}, target_Relative Alt: {target_relative_alt}, target_Vx: {target_Vx}, target_Vy: {target_Vy}, target_Vz: {target_Vz}")

dt = 0.1    # time step in seconds

print("Sending Target Coordinates...")

# Lists for NED coordinates
drone_north_log = []
drone_east_log = []

target_north_log = []
target_east_log  = []

# ============================
# Create Flight Log File
# ============================

filename = datetime.now().strftime("Drone_Target_Flight_Log_%Y%m%d_%H%M%S.csv")

log_file = open(filename, mode="w", newline="")
writer = csv.writer(log_file)

writer.writerow([
    "Time",

    # Drone Data
    "Drone_Latitude",
    "Drone_Longitude",
    "Drone_Altitude(m)",
    "Drone_Relative_Altitude(m)",
    "Drone_Vx(m/s)",
    "Drone_Vy(m/s)",
    "Drone_Vz(m/s)",

    # Target Data
    "Target_Latitude",
    "Target_Longitude",
    "Target_Altitude(m)",

    # Relative Information
    "Distance_to_Target(m)",

    # NED Coordinates
    "Drone_North(m)",
    "Drone_East(m)",
    "Target_North(m)",
    "Target_East(m)",

    # Errors
    "North_Error(m)",
    "East_Error(m)"
])

print(f"Logging flight data to: {filename}")


# Set drone to follow a moving target
print(f"Drone ready to switch to mode TARLAND")
# drone_connection.set_mode('TARLAND')

while True:
    
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
    
    # Store drone position in NED coordinates
    drone_north_log.append(drone_north)
    drone_east_log.append(drone_east)

    # Convert target GPS coordinates to NED coordinates relative to the drone's home position
    target_north = (target_lat - drone_home_lat) * 111111.0
    target_east = (target_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))

    # Store target position in NED coordinates
    target_north_log.append(target_north)
    target_east_log.append(target_east)

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

    print("sending target location through follow_target")
    # # Failsafe command
    # if distance > 100:
    #     drone_connection.set_mode('RTL')
    #     break

    if not drone_connection.is_armed():
        print("Vehicle is disarmed. Exiting loop.")  
        break

    # Calculate Tracking Errors
    north_error = target_north - drone_north
    east_error = target_east - drone_east

    # Log data into CSV
    writer.writerow([
        datetime.now().strftime("%H:%M:%S.%f"),

        # Drone
        drone_lat,
        drone_lon,
        drone_alt,
        drone_relative_alt,
        drone_vx,
        drone_vy,
        drone_vz,

        # Target
        target_lat,
        target_lon,
        target_alt,

        # Relative
        distance,

        # NED
        drone_north,
        drone_east,
        target_north,
        target_east,

        # Errors
        north_error,
        east_error
    ])

    # Save immediately
    log_file.flush()
    
    time.sleep(dt)
 
# Close log file
log_file.close()

print(f"\nFlight log saved successfully as: {filename}")

# Plot Trajectories

plt.figure(figsize=(10, 10))

# Drone trajectory
plt.plot(drone_east_log,
         drone_north_log,
         'b',
         linewidth=2,
         label='Drone')

# Target trajectory
plt.plot(target_east_log,
         target_north_log,
         'r',
         linewidth=2,
         label='Target')

# Starting points
plt.scatter(
    drone_east_log[0],
    drone_north_log[0],
    color='blue',
    marker='o',
    s=80,
    label='Drone Start'
)

plt.scatter(
    target_east_log[0],
    target_north_log[0],
    color='red',
    marker='o',
    s=80,
    label='Target Start'
)

# Ending points
plt.scatter(
    drone_east_log[-1],
    drone_north_log[-1],
    color='blue',
    marker='x',
    s=100,
    label='Drone End'
)

plt.scatter(
    target_east_log[-1],
    target_north_log[-1],
    color='red',
    marker='x',
    s=100,
    label='Target End'
)

plt.xlabel("East")
plt.ylabel("North")
plt.title("Drone and Target Trajectories")

plt.grid(True)
plt.legend()
plt.axis("equal")

plt.show()