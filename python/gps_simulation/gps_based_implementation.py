# from gps_simulation.drone_mavlink import DroneMavlink
# import time

# if __name__ == "__main__":

#     print("connecting....")
#     connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")

#     # Guided Mode
#     print("setting guided mode")
#     connection.set_mode('GUIDED')

#     #delaying
#     time.sleep(2)

#     msg=connection.request_message(24,'GPS_RAW_INT')
#     GPS_status=msg.fix_type
#     print(msg)

#     if GPS_status==3:
#         print('GPS is simulated')

#         # Arming the Drone
#         print("Arming the drone")
#         connection.arm_vehicle()
        
#         #delaying
#         time.sleep(2)

#         #Takeoff
#         print("Taking off...")
#         connection.drone_yaw_alignment_fix()
#         connection.takeoff(10)

#         # Altitude checking
#         connection.altitude_check(10)
#         print("Reached the takeoff altitude")
        
#         msg=connection.request_message(33,'GLOBAL_POSITION_INT')
#         takeoff_alt=round((msg.relative_alt)/1000)

#         # move to target position
#         if takeoff_alt==10:
#             connection.simulate_target_motion(12.99146,80.23389)


# GPS based rover navigation
from gps_simulation.drone_mavlink import DroneMavlink
import time
import math

print("connecting....")
rover_connection = DroneMavlink(connection_string="udp:127.0.0.1:14560")

# Guided Mode of rover
print("setting guided mode")
rover_connection.set_mode('GUIDED')

#delaying
time.sleep(2)

# GPS Position of rover
home_gps = rover_connection.get_gps_position()
rover_home_lat = home_gps["lat"]
rover_home_lon = home_gps["lon"]
alt = home_gps["relative_alt"]
print("Lat:", rover_home_lat)
print("Lon:", rover_home_lon)
print("Relative Alt:", alt, "m")

# Rover Velocity
rover_Vx=5
rover_Vy=2
rover_Vz=0

# Arming the rover
print("Arming the rover")
rover_connection.arm_vehicle()

#delaying
time.sleep(2)

while True:
    # Send velocity to Rover
    rover_connection.set_velocity_body(rover_Vx,rover_Vy,rover_Vz)

    time.sleep(1)

    # Receiving current rover GPS
    msg=rover_connection.request_message(33,'GLOBAL_POSITION_INT')
    rover_lat = msg.lat
    rover_lon = msg.lon
    print(f"curr_rover lat : {rover_lat} curr_rover lon {rover_lon}")

    time.sleep(0.1)



 


                