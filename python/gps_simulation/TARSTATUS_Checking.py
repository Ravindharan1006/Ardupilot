from pymavlink import mavutil

print("Connecting...")

master = mavutil.mavlink_connection("/dev/ttyUSB1", baud=57600)

master.wait_heartbeat()

print("Heartbeat received!")
print("System ID:", master.target_system)
print("Component ID:", master.target_component)
# while True:
#     msg = master.recv_match(blocking=True, timeout=2)
#     if msg:
#         print(msg.get_type())
while True:
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    if msg:
        print(msg)
# # from pymavlink import mavutil

# # master = mavutil.mavlink_connection("udpin:0.0.0.0:14551")

# # master.wait_heartbeat()
# # print("Heartbeat")

# # while True:
# #     msg = master.recv_match(blocking=True)

# #     if msg:
# #         print(msg.get_type())
# #         if msg.get_type() == "TARSTATUS":
# #             print(msg)
# #             break

# from gps_simulation.drone_mavlink import DroneMavlink
# import time
# import math

# print("connecting....")
# connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")

# # print(hasattr(connection.mav, "tarstatus_send"))

# # Guided Mode
# print("setting guided mode")
# connection.set_mode('GUIDED')

# #delaying
# time.sleep(2)

# # GPS Position
# home_gps = connection.get_gps_position()
# drone_home_lat = home_gps["lat"]
# drone_home_lon = home_gps["lon"]
# alt = home_gps["alt"]
# print("Lat:", drone_home_lat)
# print("Lon:", drone_home_lon)
# print("Alt:", alt, "m")

# # Arming the Drone
# print("Arming the drone")
# connection.arm_vehicle()

# #delaying
# time.sleep(2)

# #Takeoff
# print("Taking off...")
# connection.drone_yaw_alignment_fix()
# connection.takeoff(10)

# v_x = 7.071     # North (m/s)
# v_y = 7.071     # East (m/s)
# v_z = 0.0       # Down (m/s)
# t = 1          # time step in seconds
# rand_lat = drone_home_lat + (v_x*t)/ 111111.0    
# rand_lon = drone_home_lon + (v_y*t)/(111111.0 * math.cos(math.radians(drone_home_lat)))

# while True:
#     msg=connection.request_message(33,'GLOBAL_POSITION_INT')
#     takeoff_alt=round(msg.relative_alt/1000)
#     print("takeoff altitude = %dm" % takeoff_alt)

#     # Altitude checking
#     if takeoff_alt >= 9:
#         print("Reached the takeoff altitude")

#     # connection.set_gps_target(rand_lat, rand_lon, alt, v_x, v_y, v_z)
#     break

# # Target location
# target_lat = drone_home_lat  
# target_lon = drone_home_lon 
# target_alt = alt 
# Vx = 0.7071     # North (m/s)
# Vy = 0.7071     # East (m/s)
# dt = 0.1          # time step in seconds


# print("Sending Target Coordinates...")

# while True:

#     # distance travelled in this time step
#     north = Vx * dt
#     east  = Vy * dt

#     target_lat += north / 111111.0     # north
#     target_lon += east  / (111111.0 * math.cos(math.radians(target_lat)))    # east

#     connection.follow_target(
#         target_lat,
#         target_lon,
#         target_alt,
#         Vx,
#         Vy,
#     )

#     tar = connection.get_tarstatus()

#     if tar is not None:

#         print("TARSTATUS")
    
#         print(f"Dist_x : {tar.dist_x}, Dist_y : {tar.dist_y}, Dist_z : {tar.dist_z}")
        
#         print("Distance =", tar.distance_to_target)
#         print("Drone Altitude =", tar.drone_alt)

#         print("target vx =", tar.target_vx)
#         print("target vy =", tar.target_vy)

#         print("desired vx =", tar.desired_vx)
#         print("desired vy =", tar.desired_vy)

#     time.sleep(dt)
