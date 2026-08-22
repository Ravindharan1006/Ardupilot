from gps_simulation.drone_mavlink import DroneMavlink
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt
import csv
from datetime import datetime

print("connecting....")
connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")
target = mavutil.mavlink_connection("/dev/ttyUSB1",baud=57600)

target.wait_heartbeat()
print("Connected to Target Cube")

# Guided Mode
print("setting guided mode")
connection.set_mode('GUIDED')

#delaying
time.sleep(2)

# GPS Position
home_gps = connection.get_gps_position()
drone_home_lat = home_gps["lat"]
drone_home_lon = home_gps["lon"]
drone_home_alt = home_gps["alt"]
drone_home_relative_alt = home_gps["relative_alt"]
drone_home_Vx = home_gps["vx"]
drone_home_Vy = home_gps["vy"]
drone_home_Vz = home_gps["vz"]
print(f"Lat: {drone_home_lat}, lon: {drone_home_lon}, Alt: {drone_home_alt}, Relative Alt: {drone_home_relative_alt}, Vx: {drone_home_Vx}, Vy: {drone_home_Vy}, Vz: {drone_home_Vz}")

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


msg = target.recv_match(
    type="GLOBAL_POSITION_INT",
    blocking=True,
    timeout=1
)

if msg:
    target_lat = msg.lat / 1e7
    target_lon = msg.lon / 1e7
    target_alt = msg.alt / 1000.0
    target_relative_alt = msg.relative_alt / 1000.0
    target_vx = msg.vx / 100.0  # Convert from cm/s to m/s
    target_vy = msg.vy / 100.0  # Convert from cm/s to m/s
    target_vz = msg.vz / 100.0  # Convert from cm/s to m/s
    target_hdg = msg.hdg / 100.0  # Convert from centi-degrees to degrees
    
else:
    print("No target GPS data received.")
 
Vx = 0     # North (m/s)
Vy = 0     # East (m/s)
dt = 0.1    # time step in seconds

print(f"Target Lat: {target_lat}, Target Lon: {target_lon}, Target Alt: {target_alt}, Target Relative Alt: {target_relative_alt}, Target Vx: {target_vx}, Target Vy: {target_vy}, Target Vz: {target_vz}, Target Hdg: {target_hdg}")
print("Sending Target Coordinates...")

# Lists for NED coordinates
drone_north_log = []
drone_east_log = []

target_north_log = []
target_east_log  = []

# ============================
# Create Flight Log File
# ============================

filename = datetime.now().strftime("Drone_TARLAND_HITL_FIXED_GPS_Log_%Y%m%d_%H%M%S.csv")

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
    "Target_relative_Altitude(m)",
    "Target_Vx(m/s)",
    "Target_Vy(m/s)",
    "Target_Vz(m/s)",

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
connection.set_mode('TARLAND')

while True:

    print(f"Target Lat: {target_lat}, Target Lon: {target_lon}, Target Alt: {target_alt}, Target Relative Alt: {target_relative_alt}, Target Vx: {target_vx}, Target Vy: {target_vy}, Target Vz: {target_vz}, Target Hdg: {target_hdg}")

    # Convert target GPS coordinates to NED coordinates relative to the drone's home position
    target_north = (target_lat - drone_home_lat) * 111111.0
    target_east = (target_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))

    # Store target position in NED coordinates
    target_north_log.append(target_north)
    target_east_log.append(target_east)

    # Drone's current GPS position
    drone_gps = connection.get_gps_position()    
    drone_lat = drone_gps["lat"]
    drone_lon = drone_gps["lon"]
    drone_alt = drone_gps["alt"]
    drone_relative_alt = drone_gps["relative_alt"]
    drone_vx = drone_gps["vx"]
    drone_vy = drone_gps["vy"]
    drone_vz = drone_gps["vz"]
    print(f"drone_lat: {drone_lat}, drone_lon: {drone_lon}, drone_alt: {drone_alt}, drone_relative_alt: {drone_relative_alt}, drone_vx: {drone_vx}, drone_vy: {drone_vy}, drone_vz: {drone_vz}")

    # Convert drone GPS coordinates to NED coordinates relative to the drone's home position
    drone_north = (drone_lat - drone_home_lat) * 111111.0
    drone_east = (drone_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))
    
    # Store drone position in NED coordinates
    drone_north_log.append(drone_north)
    drone_east_log.append(drone_east)

    # calculating distance to target (Error)
    dlat = target_lat - drone_lat
    dlon = target_lon - drone_lon
    distance = math.sqrt((dlat * 111111.0) ** 2 + (dlon * 111111.0 * math.cos(math.radians(target_lat))) ** 2)

    print(f"Distance to target: {distance} m")

    connection.follow_target(
        target_lat,
        target_lon,
        target_alt,
        Vx,
        Vy,
    )

    # # Failsafe command
    # if distance > 100:
    #     connection.set_mode('RTL')
    #     break

    if not connection.is_armed():
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
        target_relative_alt,
        target_vx,
        target_vy,
        target_vz,

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
