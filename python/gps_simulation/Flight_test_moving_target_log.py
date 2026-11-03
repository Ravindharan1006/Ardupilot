from gps_simulation.drone_mavlink import DroneMavlink
import csv
import math
import os
import time
from datetime import datetime
import matplotlib.pyplot as plt

# ============================================================
# Configuration
# ============================================================

DT = 0.2  # Target update interval: nominally 5 Hz
TAKEOFF_ALT_M = 10
TARGET_ALT = None  # Set after reading the home GPS altitude

RUN_ID = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
LOG_FILE = f"tarland_hitl_{RUN_ID}.csv"

log_start = time.monotonic()
previous_loop_time = None
last_observed_mode = None

# Latest TarLand diagnostics received from ArduPilot.
# Populated when DroneMavlink exposes its received diagnostic messages.
tarland_latest = {}

LOG_FIELDS = [
    "elapsed_s",
    "loop_dt_s",
    "scheduled_dt_s",
    "flight_mode",
    "mode_change",
    "target_lat",
    "target_lon",
    "target_alt",
    "target_vx",
    "target_vy",
    "target_vz",
    "target_north_m",
    "target_east_m",
    "drone_lat",
    "drone_lon",
    "drone_alt",
    "drone_relative_alt",
    "drone_vx",
    "drone_vy",
    "drone_vz",
    "drone_north_m",
    "drone_east_m",
    "horizontal_error_m",
    "target_send_start_s",
    "target_send_end_s",
    "details",
]

# ============================================================
# CSV logging
# ============================================================

LOG_DIR = "/mnt/c/Tarland/LOG files"
# Create the directory if it does not exist
os.makedirs(LOG_DIR, exist_ok=True)

# Use system local time for the filename
RUN_ID = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")

LOG_FILE = os.path.join(LOG_DIR, f"tarland_hitl_{RUN_ID}.csv")

# --------------------------------------------------
# Initialize CSV logger
# --------------------------------------------------
log_file = open(LOG_FILE, "w", newline="", buffering=1)

csv_writer = csv.DictWriter(log_file, fieldnames=LOG_FIELDS)
csv_writer.writeheader()


def log_event(event, **values):
    row = {field: "" for field in LOG_FIELDS}

    row.update(
        {
            "elapsed_s": round(time.monotonic() - log_start, 6),
        }
    )

    row.update(values)
    csv_writer.writerow(row)
    log_file.flush()


def get_mode_safely():
    try:
        return connection.get_mode()
    except Exception as exc:
        return f"UNAVAILABLE: {exc}"


def get_tarland_snapshot() -> dict:
    method_name = "get_latest_tarland_diagnostics"

    if not hasattr(connection, method_name):
        return {}

    try:
        data = connection.get_latest_tarland_diagnostics()
        return data if isinstance(data, dict) else {}

    except Exception as exc:
        print(f"TarLand diagnostic read failed: {exc}")
        return {}


def log_mode_if_changed(current_mode):
    global last_observed_mode

    if current_mode != last_observed_mode:
        log_event(
            "MODE_OBSERVED",
            flight_mode=current_mode,
            mode_change=f"{last_observed_mode} -> {current_mode}",
        )
        last_observed_mode = current_mode


def close_log():
    if not log_file.closed:
        log_file.flush()
        log_file.close()

    print(f"\nCSV log saved: {os.path.abspath(LOG_FILE)}")


# ============================================================
# Trajectory arrays
# ============================================================

drone_north_log = []
drone_east_log = []
target_north_log = []
target_east_log = []


# ============================================================
# Main HITL experiment
# ============================================================

print("Connecting to vehicle...")
connection = DroneMavlink(connection_string="udp:127.0.0.1:14550")

try:
    log_event("SCRIPT_START", details="HITL target-follow experiment started")

    # --------------------------------------------------------
    # GUIDED
    # --------------------------------------------------------

    print("Setting GUIDED mode...")
    log_event("MODE_CHANGE_REQUEST", mode_change="GUIDED")

    connection.set_mode("GUIDED")
    time.sleep(2)

    log_event(
        "MODE_AFTER_GUIDED_REQUEST",
        flight_mode=get_mode_safely(),
    )

    # --------------------------------------------------------
    # Initial GPS snapshot
    # --------------------------------------------------------

    home_gps = connection.get_gps_position()

    drone_home_lat = home_gps["lat"]
    drone_home_lon = home_gps["lon"]
    drone_home_alt = home_gps["alt"]
    drone_home_relative_alt = home_gps["relative_alt"]

    drone_home_vx = home_gps["vx"]
    drone_home_vy = home_gps["vy"]
    drone_home_vz = home_gps["vz"]

    print(
        f"Home lat={drone_home_lat}, lon={drone_home_lon}, "
        f"alt={drone_home_alt}, relative_alt={drone_home_relative_alt}, "
        f"vx={drone_home_vx}, vy={drone_home_vy}, vz={drone_home_vz}"
    )

    log_event(
        "HOME_GPS",
        drone_lat=drone_home_lat,
        drone_lon=drone_home_lon,
        drone_alt=drone_home_alt,
        drone_relative_alt=drone_home_relative_alt,
        drone_vx=drone_home_vx,
        drone_vy=drone_home_vy,
        drone_vz=drone_home_vz,
    )

    # --------------------------------------------------------
    # Arm and take off
    # --------------------------------------------------------

    print("Arming vehicle...")
    log_event("ARM_REQUEST")
    connection.arm_vehicle()
    time.sleep(2)

    print("Taking off...")
    connection.drone_yaw_alignment_fix()
    connection.takeoff(TAKEOFF_ALT_M)

    log_event("TAKEOFF_REQUEST", details=f"Altitude={TAKEOFF_ALT_M} m")

    while True:
        msg = connection.request_message(33, "GLOBAL_POSITION_INT")
        takeoff_alt = round(msg.relative_alt / 1000.0)

        print(f"Takeoff altitude = {takeoff_alt} m")

        log_event(
            "TAKEOFF_ALTITUDE_CHECK",
            drone_relative_alt=msg.relative_alt / 1000.0,
            details=f"Rounded altitude={takeoff_alt} m",
        )

        if takeoff_alt >= 9:
            print("Reached the takeoff altitude")
            break

        time.sleep(0.2)

    # --------------------------------------------------------
    # Initial target location and velocity
    # --------------------------------------------------------

    offset_north = 0
    offset_east = 0

    offset_distance = math.hypot(offset_north, offset_east)
    print(f"Target initial offset distance = {offset_distance:.2f} m")

    rand_lat = drone_home_lat + offset_north / 111111.0
    rand_lon = drone_home_lon + (offset_east / (111111.0 * math.cos(math.radians(drone_home_lat))))

    target_lat = rand_lat
    target_lon = rand_lon
    target_alt = drone_home_alt

    # IMPORTANT:
    # These velocities must use the units expected by follow_target().
    # Verify whether get_gps_position() returns m/s or cm/s before
    # deriving target velocity from the drone's reported velocity.
    target_vx = drone_home_vx + 0.707
    target_vy = drone_home_vy + 0.707
    target_vz = 0.0

    log_event(
        "TARGET_INITIALIZED",
        target_lat=target_lat,
        target_lon=target_lon,
        target_alt=target_alt,
        target_vx=target_vx,
        target_vy=target_vy,
        target_vz=target_vz,
    )

    # # --------------------------------------------------------
    # # Switch to TARLAND
    # # --------------------------------------------------------

    # print("Switching to TARLAND...")
    # log_event("MODE_CHANGE_REQUEST", mode_change="TARLAND")

    # connection.set_mode("TARLAND")
    # time.sleep(0.2)

    mode_after_request = get_mode_safely()
    log_event(
        "MODE_AFTER_TARLAND_REQUEST",
        flight_mode=mode_after_request,
    )

    print(f"Mode reported after request: {mode_after_request}")
    print("Starting target updates...")

    # --------------------------------------------------------
    # Target tracking loop
    # --------------------------------------------------------
    time.sleep(30)
    while connection.is_armed():
        loop_start = time.monotonic()

        if previous_loop_time is None:
            loop_dt = DT
        else:
            loop_dt = loop_start - previous_loop_time

        previous_loop_time = loop_start

        # Integrate target movement using actual elapsed time.
        # This avoids assuming each iteration takes exactly DT.
        north_step = target_vx * loop_dt
        east_step = target_vy * loop_dt

        target_lat += north_step / 111111.0
        target_lon += east_step / (111111.0 * math.cos(math.radians(target_lat)))

        target_north = (target_lat - drone_home_lat) * 111111.0
        target_east = (target_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))

        # Read drone position and velocity.
        drone_gps = connection.get_gps_position()

        drone_lat = drone_gps["lat"]
        drone_lon = drone_gps["lon"]
        drone_alt = drone_gps["alt"]
        drone_relative_alt = drone_gps["relative_alt"]

        drone_vx = drone_gps["vx"]
        drone_vy = drone_gps["vy"]
        drone_vz = drone_gps["vz"]

        drone_north = (drone_lat - drone_home_lat) * 111111.0
        drone_east = (drone_lon - drone_home_lon) * 111111.0 * math.cos(math.radians(drone_home_lat))

        # Horizontal distance between target and drone.
        north_error = target_north - drone_north
        east_error = target_east - drone_east
        distance = math.hypot(north_error, east_error)

        print(
            f"Target=({target_lat:.8f}, {target_lon:.8f}) | "
            f"Drone=({drone_lat:.8f}, {drone_lon:.8f}) | "
            f"XY error={distance:.3f} m | "
            f"Relative altitude={drone_relative_alt}"
        )

        target_north_log.append(target_north)
        target_east_log.append(target_east)
        drone_north_log.append(drone_north)
        drone_east_log.append(drone_east)

        # Send the target update and measure the wrapper-call duration.
        send_start = time.monotonic()

        connection.follow_target(
            target_lat,
            target_lon,
            target_alt,
            target_vx,
            target_vy,
        )

        send_end = time.monotonic()

        current_mode = get_mode_safely()
        log_mode_if_changed(current_mode)

        # Read the latest internal diagnostics received from ArduPilot.
        # These fields remain empty until the receive-path integration
        # in DroneMavlink is implemented.
        diag = get_tarland_snapshot()

        log_event(
            "TARGET_UPDATE_AND_DRONE_SNAPSHOT",
            scheduled_dt_s=DT,
            loop_dt_s=round(loop_dt, 6),
            flight_mode=current_mode,
            target_lat=target_lat,
            target_lon=target_lon,
            target_alt=target_alt,
            target_vx=target_vx,
            target_vy=target_vy,
            target_vz=target_vz,
            target_north_m=target_north,
            target_east_m=target_east,
            drone_lat=drone_lat,
            drone_lon=drone_lon,
            drone_alt=drone_alt,
            drone_relative_alt=drone_relative_alt,
            drone_vx=drone_vx,
            drone_vy=drone_vy,
            drone_vz=drone_vz,
            drone_north_m=drone_north,
            drone_east_m=drone_east,
            horizontal_error_m=distance,
            target_send_start_s=round(send_start - log_start, 6),
            target_send_end_s=round(send_end - log_start, 6),
        )

        # Stop condition retained from your original experiment.
        if drone_relative_alt <= 0:
            print("Drone is at ground level. Stopping simulation.")
            log_event(
                "SIMULATION_STOP_CONDITION",
                flight_mode=current_mode,
                drone_relative_alt=drone_relative_alt,
                details="Relative altitude <= 0",
            )
            break

        # Aim for DT seconds between loop starts.
        remaining_time = DT - (time.monotonic() - loop_start)
        if remaining_time > 0:
            time.sleep(remaining_time)

except KeyboardInterrupt:
    print("\nSimulation interrupted by user.")
    log_event("USER_INTERRUPT")

except Exception as exc:
    log_event("PYTHON_EXCEPTION", details=repr(exc))
    raise

finally:
    close_log()


# ============================================================
# Plot trajectories if data was collected
# ============================================================

if drone_north_log and target_north_log:
    plt.figure(figsize=(10, 10))

    plt.plot(
        drone_east_log,
        drone_north_log,
        "b",
        linewidth=2,
        label="Drone",
    )
    plt.plot(
        target_east_log,
        target_north_log,
        "r",
        linewidth=2,
        label="Target",
    )

    plt.scatter(
        drone_east_log[0],
        drone_north_log[0],
        color="blue",
        marker="o",
        s=80,
        label="Drone Start",
    )
    plt.scatter(
        target_east_log[0],
        target_north_log[0],
        color="red",
        marker="o",
        s=80,
        label="Target Start",
    )

    plt.scatter(
        drone_east_log[-1],
        drone_north_log[-1],
        color="blue",
        marker="x",
        s=100,
        label="Drone End",
    )
    plt.scatter(
        target_east_log[-1],
        target_north_log[-1],
        color="red",
        marker="x",
        s=100,
        label="Target End",
    )

    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.title("Drone and Target Trajectories")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()
