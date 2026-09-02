from gps_simulation.drone_mavlink import DroneMavlink
import time
import math
import matplotlib.pyplot as plt
import csv
from datetime import datetime

print("connecting....")
drone_connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")

# Drone GPS Position
if drone_connection:
    drone_home_gps = drone_connection.get_gps_position()
    drone_home_lat = drone_home_gps["lat"]
    drone_home_lon = drone_home_gps["lon"]
    drone_home_alt = drone_home_gps["alt"]
    drone_home_relative_alt = drone_home_gps["relative_alt"]
    drone_home_Vx = drone_home_gps["vx"]
    drone_home_Vy = drone_home_gps["vy"]
    drone_home_Vz = drone_home_gps["vz"]

else:
    print("No target GPS data received.")

print(f"drone_Lat: {drone_home_lat}, drone_lon: {drone_home_lon}, drone_Alt: {drone_home_alt}, drone_Relative Alt: {drone_home_relative_alt}, drone_Vx: {drone_home_Vx}, drone_Vy: {drone_home_Vy}, drone_Vz: {drone_home_Vz}")

#delaying
time.sleep(2)

# Target GPS Position
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
target_Vx = drone_home_Vx + 0.707     # North (m/s)
target_Vy = drone_home_Vy + 0.707     # East (m/s)
target_Vz = drone_home_Vz
dt = 0.1    # time step in seconds

print(f"target_Lat: {target_lat}, target_lon: {target_lon}, target_Alt: {target_alt}, target_Relative Alt: {target_relative_alt}, target_Vx: {target_Vx}, target_Vy: {target_Vy}, target_Vz: {target_Vz}")

dt = 0.1    # time step in seconds

# Lists for NED coordinates
drone_north_log = []
drone_east_log = []

target_north_log = []
target_east_log  = []

# ============================
# Create Flight Log File
# ============================

filename = datetime.now().strftime("Flight_test_fixed_GPS_target_Log_%d/%m/%Y_%H:%M:%S.csv")

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

time.sleep(30)

while drone_connection.is_armed():
        

    # distance travelled in this time step
    north = target_Vx * dt
    east  = target_Vy * dt

    target_lat += north / 111111.0     # north
    target_lon += east  / (111111.0 * math.cos(math.radians(target_lat)))    # east
    
    print(f"Target Lat: {target_lat}, Target Lon: {target_lon}, Target Alt: {target_alt}, Target Relative Alt: {target_relative_alt}, Target Vx: {target_vx}, Target Vy: {target_vy}, Target Vz: {target_vz}, Target Hdg: {target_hdg}")
    
    # Convert target GPS coordinates to NED coordinates relative to the drone's home position
    target_north = (target_lat - drone_home_lat) * 111111.0
    target_east = (target_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))

    # Store target position in NED coordinates
    target_north_log.append(target_north)
    target_east_log.append(target_east)

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

    drone_connection.follow_target(
        target_lat,
        target_lon,
        target_alt,
        target_Vx,
        target_Vy,
    )

    # # Failsafe command
    # if distance > 100:
    #     connection.set_mode('RTL')
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
        target_relative_alt,
        target_Vx,
        target_Vy,
        target_Vz,

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
