from gps_simulation.drone_mavlink import DroneMavlink
import time
import math
import matplotlib.pyplot as plt
import csv
from datetime import datetime

print("connecting....")
drone_connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")
target_connection = DroneMavlink(connection_string="udp:127.0.0.1:14560")

print("Waiting for heartbeat...")
print("Drone connected")


# target_connection = DroneMavlink(connection_string="udp:127.0.0.1:14560")

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


    # NED Coordinates
    "Drone_North(m)",
    "Drone_East(m)",
 
])

print(f"Logging flight data to: {filename}")


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
    # Convert drone GPS coordinates to NED coordinates relative to the drone's home position
    drone_north = (drone_lat - drone_home_lat) * 111111.0
    drone_east = (drone_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))
    
    # Store drone position in NED coordinates
    drone_north_log.append(drone_north)
    drone_east_log.append(drone_east)
    
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

        # NED Coordinates
        Drone_North(m),
        Drone_East(m),

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


# Starting points
plt.scatter(
    drone_east_log[0],
    drone_north_log[0],
    color='blue',
    marker='o',
    s=80,
    label='Drone Start'
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


plt.xlabel("East")
plt.ylabel("North")
plt.title("Drone Trajectory")

plt.grid(True)
plt.legend()
plt.axis("equal")

plt.show()