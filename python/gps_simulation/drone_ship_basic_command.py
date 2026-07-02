from gps_simulation.drone_mavlink import DroneMavlink
from gps_simulation.rover_mavlink import RoverMavlink
import time

if __name__ == "__main__":

    print("connecting....")
    drone_connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")
    ship_connection = RoverMavlink(connection_string="udp:127.0.0.1:14560")

    # Drone home GPS
    msg=drone_connection.request_message(33,'GLOBAL_POSITION_INT')
    drone_home_lat=msg.lat / 1e7
    drone_home_lon=msg.lon / 1e7

    print("Home location of drone lat: %.6f, lon: %.6f" %(drone_home_lat,drone_home_lon))

    # Ship home GPS
    msg=ship_connection.request_message(33,'GLOBAL_POSITION_INT')
    ship_home_lat=msg.lat / 1e7
    ship_home_lon=msg.lon / 1e7
    print("Home location of ship lat: %.6f, lon: %.6f" %(ship_home_lat,ship_home_lon))

    # Guided Mode - Ship
    print("setting ship to guided mode")
    ship_connection.set_mode('GUIDED')    

    # delaying
    time.sleep(5)

    # Arming the Ship
    print("Arming the ship")
    ship_connection.arm_vehicle()

    # delaying
    time.sleep(5)

    # Guided Mode - Drone
    print("setting drone to guided mode")
    drone_connection.set_mode('GUIDED')

    # delaying
    time.sleep(2)

    # Arming the Drone
    print("Arming the drone")
    drone_connection.arm_vehicle()
    
    # delaying
    time.sleep(2)

    #Takeoff
    print("Taking off...")
    drone_connection.takeoff(20)

    # Altitude check
    alt_check=drone_connection.altitude_check(20)
    alt = 20

    Vx = 2
    Vy = 2

    alt_s = 0

    starting_time = time.time()

    while True:
        
        now = time.time()

        t = now - starting_time

        # ship_connection.moving_target(ship_home_lat,ship_home_lon)

        app_lat = 0.000001
        app_lon = 0.000001

        ship_home_lat += app_lat
        ship_home_lon += app_lon

        Vx += 0.5
        Vy += 0.5

        ship_connection.set_target(ship_home_lat,ship_home_lon,alt_s,Vx,Vy,Vz=0)


        # Get the current ship location
        msg=ship_connection.request_message(33,'GLOBAL_POSITION_INT')
        curr_ship_lat=msg.lat / 1e7
        curr_ship_lon=msg.lon / 1e7
        curr_ship_alt=round((msg.alt) / 1000)
        # curr_ship_relative_alt=round((msg.relative_alt) / 1000)
        curr_ship_Vx=int(msg.vx)
        curr_ship_Vy=int(msg.vy) 
        curr_ship_Vz=int(msg.vz)
        curr_ship_hdg=msg.hdg

        print("Ship current locaion lat: %.6f, lon: %.6f, alt: %d" % (curr_ship_lat,curr_ship_lon,alt))


        if t>=30:
            
            drone_connection.set_target(curr_ship_lat,curr_ship_lon,alt,curr_ship_Vx,curr_ship_Vy,curr_ship_Vz)

            # Get the current drone location
            msg=drone_connection.request_message(33,'GLOBAL_POSITION_INT')
            curr_drone_lat=msg.lat / 1e7
            curr_drone_lon=msg.lon / 1e7
            curr_drone_alt=round((msg.relative_alt) / 1000)

            print("Drone current locaion lat: %.6f, lon: %.6f, alt: %d" % (curr_drone_lat,curr_drone_lon,curr_drone_alt))

            time.sleep(0.05)

        else:
            time.sleep(0.05)

                
