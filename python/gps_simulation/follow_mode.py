from gps_simulation.drone_mavlink import DroneMavlink

# from mavlink.rover_mavlink import RoverMavlink
from gps_simulation.platform import Platform
import time
import numpy as np


class GPSLandingTest:

    drone_origin = np.array([129914470, 802360990, 0])
    target_v0 = np.array([2.5, 0, 0])
    target_state = np.zeros((6,))
    drone_state = np.zeros((6,))
    target: Platform
    target_distance_from_drone = 10

    def initialize_target(self, target_distance_from_drone):
        """
        Intialize the position and velocity of the moving target. Initial position
        of the target is defined as the origin

        """
        self.target = Platform()
        self.target_origin = self.calculate_target_origin(target_distance_from_drone)
        self.target.spawn(self.target_origin, self.target_v0)

        self.target_state[0:3] = self.target_origin
        self.target_state[3:6] = self.target_v0

    def move_target(self):

        self.target.dt = 1
        self.target.an = 0
        self.target.at = 0
        self.target_state = self.target.move(
            self.target_state[0:3], self.target_state[3:6]
        )

        # z = self.mavlink.get_elevation(target_state[0], target_state[1])
        # if z == None:
        #     z = 0
        # target_state[2] = z

    def calculate_target_origin(self, distance_from_drone: float):
        """
        Calculate the origin of target from given distance from the drone start location

        (Args):
            distance_from_drone in meters
        (Return):
            lat (degE7) and lng (degE7) and alt (m) of target origin
        """

        theta = np.random.uniform(low=0, high=2 * np.pi)

        dx = distance_from_drone * np.cos(theta)
        dy = distance_from_drone * np.sin(theta)

        dlat = self.target._dx_to_dlat(dx)
        dlng = self.target._dy_to_dlng(dy, self.drone_origin[0])

        target_origin = np.array(
            [self.drone_origin[0] + dlat, self.drone_origin[1] + dlng, 0]
        )
        return target_origin

    def run_test(self):
        print("Connecting to drone ...")
        drone_connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")

        # Drone home GPS
        msg = drone_connection.request_message(33, "GLOBAL_POSITION_INT")
        drone_home_lat = msg.lat / 1e7
        drone_home_lon = msg.lon / 1e7
        print("Connected to drone")
        print(
            "Home location of drone lat: %.6f, lon: %.6f"
            % (drone_home_lat, drone_home_lon)
        )

        print("Connecting to ship ...")
        # ship_connection = RoverMavlink(connection_string="udp:127.0.0.1:14560")
        print("Connected to ship")

        print("setting drone to guided mode")
        drone_connection.set_mode("GUIDED")

        # TODO do a check if the vehicle is ready to arm programmatically
        drone_connection.arm_vehicle()
        drone_connection.takeoff(10)
        drone_connection.send_target_pos(0, self.target_state)
        self.initialize_target(self.target_distance_from_drone)

        drone_connection.set_mode("FOLLOW")
        while True:

            self.move_target()
            drone_connection.send_target_pos(0, self.target_state)
            print(f"target lat: {self.target_state[0]}")
            print(f"target lng: {self.target_state[1]}")


if __name__ == "__main__":

    test = GPSLandingTest()
    test.run_test()

# Ship home GPS
msg=ship_connection.request_message(33,'GLOBAL_POSITION_INT')
ship_home_lat=msg.lat / 1e7
ship_home_lon=msg.lon / 1e7
print("Home location of ship lat: %.6f, lon: %.6f" %(ship_home_lat,ship_home_lon))

# # # # Guided Mode - Ship
print("setting ship to guided mode")
ship_connection.set_mode('GUIDED')

# # # # delaying
time.sleep(5)

# # Arming the Ship
print("Arming the ship")
ship_connection.arm_vehicle()

Vx = 2
Vy = 2
Vz = 3

home_lat = drone_home_lat + 0.0002
home_lon = drone_home_lon + 0.0002

alt_g = 584
alt_rel = 0

drone_connection.set_mode('FOLLOW')
print("mode switched to Follow")

starting_time = time.time()

while True:

    now = time.time()

    t = now - starting_time

    ship_connection.moving_target(ship_home_lat,ship_home_lon)

    app_lat = 0.000001
    app_lon = 0.000001

    home_lat += app_lat
    home_lon += app_lon

    Vx += 0.5
    Vy += 0.5
    Vz += 0.5

    ship_connection.set_target(home_lat,home_lon,alt_rel,Vx,Vy,Vz=0)

    print("current gps location lat: %.6f, lon: %.6f" % (home_lat,home_lon))

    drone_connection.gps_msg(home_lat,home_lon,alt_g,alt_rel,Vx,Vy,Vz)
    drone_connection.follow_target(home_lat,home_lon,Vx,Vy)

    # Get the current drone location
    msg=drone_connection.request_message(33,'GLOBAL_POSITION_INT')
    curr_drone_lat=msg.lat / 1e7
    curr_drone_lon=msg.lon / 1e7
    curr_drone_alt=round((msg.relative_alt) / 1000)

    print("Drone current locaion lat: %.6f, lon: %.6f, alt: %d" % (curr_drone_lat,curr_drone_lon,curr_drone_alt))

    time.sleep(0.1)

    ship_connection.set_target(ship_home_lat,ship_home_lon,alt_s,Vx,Vy,Vz=0)

    # Get the current ship location
    msg=ship_connection.request_message(33,'GLOBAL_POSITION_INT')
    curr_ship_lat=msg.lat / 1e7
    curr_ship_lon=msg.lon / 1e7
    curr_ship_alt=round((msg.alt) / 1000)
    curr_ship_relative_alt=round((msg.relative_alt) / 1000)
    curr_ship_Vx=int((msg.vx) / 100)
    curr_ship_Vy=int((msg.vy) / 100)
    curr_ship_Vz=int((msg.vz) / 100)
    curr_ship_hdg=msg.hdg

    print("Ship current locaion lat: %.6f, lon: %.6f,Vx: %d,Vy: %d" % (curr_ship_lat,curr_ship_lon,curr_ship_Vx,curr_ship_Vy))

    if t>=30:

        drone_connection.gps_msg(curr_ship_lat,curr_ship_lon,curr_ship_alt,curr_ship_relative_alt,curr_ship_Vx,curr_ship_Vy,curr_ship_Vz,curr_ship_hdg)

        drone_connection.follow_target(curr_ship_lat,curr_ship_lon,curr_ship_Vx,curr_ship_Vy)

        # Get the current drone location
        msg=drone_connection.request_message(33,'GLOBAL_POSITION_INT')
        curr_drone_lat=msg.lat / 1e7
        curr_drone_lon=msg.lon / 1e7
        curr_drone_alt=round((msg.relative_alt) / 1000)

        print("Drone current locaion lat: %.6f, lon: %.6f, alt: %d" % (curr_drone_lat,curr_drone_lon,curr_drone_alt))

        time.sleep(0.05)

    else:

        time.sleep(0.05)

        # distance between ship and drone
        distance = ship_connection.gps_distance(curr_ship_lat,curr_ship_lon,curr_drone_lat,curr_drone_lon)
        print("distance between drone and ship : %dm" %distance)

        if distance <= 3:

            print("Drone has reached within the landing zone")

            # reducing vertical velocity
            Vz-=0.2

            msg = drone_connection.request_message(33,'GLOBAL_POSITION_INT')
            alt_1 = round((msg.relative_alt) / 1000)
            print(alt_1)

            if alt_1 == 2:

                drone_connection.set_mode('LAND')
                print("Drone set to land")

                # delaying
                time.sleep(10)

                drone_connection.set_mode('STABILIZE')

                time.sleep(0.1)

        break

            # else:
            # time.sleep(0.1)
