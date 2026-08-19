from gps_simulation.drone_mavlink import DroneMavlink
import time
import math
import matplotlib.pyplot as plt

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

# Set initial target position
offset_north = 60      # meters
offset_east  = -60     # meters
offset_distance = math.sqrt(offset_north**2 + offset_east**2)   # shouldn't be more than 100 meters
print("Target offset distance = %.2f m" % offset_distance)

rand_lat = drone_home_lat + offset_north / 111111.0
rand_lon = drone_home_lon + offset_east / (111111.0 * math.cos(math.radians(drone_home_lat)))
pos_details = {"lat": rand_lat,"lon": rand_lon,"alt": 10,"vx": 0,"vy": 0,"vz": 0}

while True:
    msg=connection.request_message(33,'GLOBAL_POSITION_INT')
    takeoff_alt=round(msg.relative_alt/1000)
    print("takeoff altitude = %dm" % takeoff_alt)

    # Altitude checking
    if takeoff_alt >= 9:
        print("Reached the takeoff altitude")

    break


# Set drone to follow a moving target
print(f"Drone ready to switch to mode TARLAND")
connection.set_mode('TARLAND')

# Target location
target_lat = rand_lat  
target_lon = rand_lon
target_alt = drone_home_alt 
target_Vx = drone_home_Vx + 0.707         # North (m/s)
target_Vy = drone_home_Vy + 0.707       # East (m/s)
dt = 0.1        # time step in seconds

print("Sending Target Coordinates...")

# Lists for NED coordinates
drone_north_log = []
drone_east_log = []

target_north_log = []
target_east_log  = []

while True:

    # distance travelled in this time step
    north = target_Vx * dt
    east  = target_Vy * dt

    target_lat += north / 111111.0     # north
    target_lon += east  / (111111.0 * math.cos(math.radians(target_lat)))    # east
    
    print(f"Target Lat: {target_lat}, Target Lon: {target_lon}, Target Alt: {target_alt}")
    
    # Convert target GPS coordinates to NED coordinates relative to the drone's home position
    target_north = (target_lat - drone_home_lat) * 111111.0
    target_east = (target_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))

    # Store target position in NED coordinates
    target_north_log.append(target_north)
    target_east_log.append(target_east)
    
    drone_gps = connection.get_gps_position()    # Drone's current GPS position
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
        target_Vx,
        target_Vy,
    )

    # # Failsafe command
    # if distance > 100:
    #     connection.set_mode('RTL')
    #     break

    if not connection.is_armed():
        print("Vehicle is disarmed. Exiting loop.")  
        break
    
    time.sleep(dt)
 

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


