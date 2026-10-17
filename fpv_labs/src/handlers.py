import math
from collections.abc import Callable

from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from std_srvs.srv import Trigger

from drone import Drone, Handler, MissionError, Position, Step
from links.mavlink import (
    MissionAcknowledgement,
    MissionItem,
    MissionRequest,
    MissionRequestInt,
    ParameterValue,
    mission_accepted,
)

AUTO = 3
GUIDED = 4
LAND = 9

MAVLINK_TIMEOUT = 30.0
PARAMETER_TOLERANCE = 1e-3

# /odometry arrives 50 times per second
ODOMETRY_TIMEOUT = 0.5
ODOMETRY_ABORT_TIMEOUT = 5.0

ALTITUDE_GAIN = 0.8
MAX_CLIMB_SPEED = 0.5


# --- helpers ---


def require_armed(drone: Drone) -> None:
    if drone.status is None or not drone.status.armed:
        raise MissionError(f"Drone disarmed unexpectedly during {drone.step.name}")


def fail_after(drone: Drone, message: str, timeout: float = MAVLINK_TIMEOUT) -> None:
    if drone.time_in_step() > timeout:
        raise MissionError(message)


def distance_to_target(drone: Drone) -> float:
    """2D: x y"""
    x, y, _ = drone.position()
    return math.hypot(drone.target[0] - x, drone.target[1] - y)


def altitude_correction(drone: Drone) -> float:
    error = drone.target[2] - drone.position()[2]
    return max(-MAX_CLIMB_SPEED, min(MAX_CLIMB_SPEED, ALTITUDE_GAIN * error))


def hold(drone: Drone) -> None:
    drone.publish_velocity(0.0, 0.0, altitude_correction(drone))


def stale_odometry(drone: Drone, abort: Step) -> tuple[bool, Step | None]:
    age = drone.odometry_age()

    if age <= ODOMETRY_TIMEOUT:
        return False, None

    drone.publish_velocity(0.0, 0.0, 0.0)
    drone.get_logger().warn(
        "Odometry lost, holding position", throttle_duration_sec=2.0
    )

    if age > ODOMETRY_ABORT_TIMEOUT:
        drone.get_logger().error("Odometry not restored, aborting")
        return True, abort

    return True, None


# --- preparation ---


def wait_for_system(then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        if drone.status is None or drone.odometry is None:
            return None

        if not drone.services_ready():
            return None

        drone.get_logger().info("ArduPilot and all services are available")
        return then

    return handler


def prearm_check(then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        response: Trigger.Response | None = drone.request(
            drone.prearm_client, Trigger.Request()
        )

        if response is None:
            return None

        if not response.success:
            raise MissionError(f"Pre-arm check failed: {response.message}")

        drone.get_logger().info(f"Pre-arm check passed: {response.message}")
        return then

    return handler


def set_mode(mode: int, then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        response: ModeSwitch.Response | None = drone.request(
            drone.mode_client, ModeSwitch.Request(mode=mode)
        )

        if response is None:
            return None

        if not response.status or response.curr_mode != mode:
            raise MissionError(
                f"Could not enable mode {mode}, current mode: {response.curr_mode}"
            )

        drone.get_logger().info(f"Flight mode {mode} enabled")
        return then

    return handler


def arm(then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        response: ArmMotors.Response | None = drone.request(
            drone.arm_client, ArmMotors.Request(arm=True)
        )

        if response is None:
            return None

        if not response.result:
            raise MissionError("ArduPilot rejected the arming request")

        drone.get_logger().info("Motors armed")
        return then

    return handler


# --- takeoff and hover ---


def take_off(altitude: float, then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        if drone.entering():
            drone.home = drone.position()
            x, y, z = drone.home
            drone.target = (x, y, z + altitude)

        response: Takeoff.Response | None = drone.request(
            drone.takeoff_client, Takeoff.Request(alt=float(altitude))
        )

        if response is None:
            return None

        if not response.status:
            raise MissionError("ArduPilot rejected the takeoff request")

        drone.get_logger().info(f"Takeoff to {altitude:.1f} m accepted")
        return then

    return handler


def climb(
    then: Step, abort: Step, tolerance: float = 0.15, timeout: float = 30.0
) -> Handler:
    def handler(drone: Drone) -> Step | None:
        require_armed(drone)

        altitude = drone.position()[2]

        if drone.target[2] - altitude <= tolerance:
            drone.get_logger().info(f"Altitude reached: {altitude:.2f} m")
            return then

        if drone.time_in_step() > timeout:
            drone.get_logger().error(f"Altitude not reached, current: {altitude:.2f} m")
            return abort

        return None

    return handler


def hover(duration: float, then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        require_armed(drone)

        if drone.time_in_step() >= duration:
            drone.get_logger().info(f"Hovered for {duration:.1f} s")
            return then

        return None

    return handler


# --- flight to drone.target ---


def aim(compute_target: Callable[[Position], Position], then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        drone.target = compute_target(drone.home)

        drone.get_logger().info(
            f"Start position: x={drone.home[0]:.2f}, y={drone.home[1]:.2f}"
        )
        drone.get_logger().info(
            f"Target position: x={drone.target[0]:.2f}, y={drone.target[1]:.2f}, "
            f"z={drone.target[2]:.2f}"
        )
        return then

    return handler


def fly_to_target(
    *,
    then: Step,
    abort: Step,
    max_speed: float = 1.0,
    gain: float = 0.6,
    arrival_radius: float = 0.08,
    arrival_speed: float = 0.1,
    timeout: float = 120.0,
) -> Handler:
    def handler(drone: Drone) -> Step | None:
        require_armed(drone)

        stale, next_step = stale_odometry(drone, abort)
        if stale:
            return next_step

        if drone.time_in_step() > timeout:
            drone.get_logger().error("Target was not reached in time")
            return abort

        x, y, _ = drone.position()
        error_x = drone.target[0] - x
        error_y = drone.target[1] - y
        distance = math.hypot(error_x, error_y)
        speed = drone.horizontal_speed()

        if distance <= arrival_radius and speed <= arrival_speed:
            hold(drone)
            drone.get_logger().info(f"Target reached, distance {distance:.2f} m")
            return then

        # Far from the target the speed is capped by max_speed, nearby it decreases
        command_speed = min(max_speed, gain * distance)

        drone.publish_velocity(
            command_speed * error_x / distance,
            command_speed * error_y / distance,
            altitude_correction(drone),
        )
        drone.get_logger().info(
            f"x={x:.2f}, y={y:.2f}, distance {distance:.2f} m, speed {speed:.2f} m/s",
            throttle_duration_sec=1.0,
        )
        return None

    return handler


def stop_at_target(
    *,
    then: Step,
    retry: Step,
    abort: Step,
    settle: float = 2.0,
    tolerance: float = 0.2,
    arrival_speed: float = 0.1,
) -> Handler:

    def handler(drone: Drone) -> Step | None:
        require_armed(drone)

        stale, next_step = stale_odometry(drone, abort)
        if stale:
            return next_step

        hold(drone)

        if drone.time_in_step() < settle:
            return None

        if drone.horizontal_speed() > arrival_speed:
            return None

        x, y, _ = drone.position()
        error = distance_to_target(drone)

        if error > tolerance:
            drone.get_logger().warn(f"Drifted {error:.2f} m from target, correcting")
            return retry

        drone.arrival_error = error
        drone.get_logger().info(f"Position before landing: x={x:.2f}, y={y:.2f}")
        drone.get_logger().info(
            f"Positioning error: {error:.3f} m (tolerance {tolerance:.2f} m)"
        )
        return then

    return handler


# --- MAVLink: parameters and missions ---


def wait_for_mavlink(then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        link = drone.link()

        if link.connected:
            drone.get_logger().info(f"MAVLink: vehicle {link.system} on {link.address}")
            return then

        fail_after(drone, f"No MAVLink HEARTBEAT on {link.address}")
        return None

    return handler


def set_parameter(name: str, value: float, then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        link = drone.link()

        if drone.entering():
            link.set_parameter(name, value)

        for message in link.received(ParameterValue):
            if (
                message.param_id == name
                and abs(message.param_value - value) < PARAMETER_TOLERANCE
            ):
                drone.get_logger().info(f"Parameter {name} = {message.param_value:g}")
                return then

        fail_after(drone, f"ArduPilot did not confirm {name} = {value:g}")
        return None

    return handler


def upload_mission(items: list[MissionItem], then: Step) -> Handler:
    plan = [MissionItem.home(), *items]

    def handler(drone: Drone) -> Step | None:
        link = drone.link()

        if drone.entering():
            link.send_mission_count(len(plan))

        # ArduPilot asks for each item by its index, then acknowledges the whole plan
        requests = link.received(MissionRequest) + link.received(MissionRequestInt)
        for request in requests:
            link.send_mission_item(request.seq, plan[request.seq])

        for acknowledgement in link.received(MissionAcknowledgement):
            if not mission_accepted(acknowledgement):
                raise MissionError(
                    f"ArduPilot rejected the mission: {acknowledgement.type}"
                )

            drone.get_logger().info(f"Mission uploaded: {len(items)} items")
            return then

        fail_after(drone, "ArduPilot did not answer during mission upload")
        return None

    return handler


def start_mission(then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        drone.link().start_mission()
        drone.get_logger().info("Mission start sent")
        return then

    return handler


# --- flight controlled by ArduPilot (AUTO) ---


def wait_for_airborne(then: Step, abort: Step, timeout: float = 30.0) -> Handler:
    def handler(drone: Drone) -> Step | None:
        require_armed(drone)

        if drone.status is not None and drone.status.flying:
            drone.get_logger().info("Airborne")
            return then

        if drone.time_in_step() > timeout:
            drone.get_logger().error("Vehicle did not take off")
            return abort

        return None

    return handler


def follow_mission(then: Step, abort: Step, timeout: float = 600.0) -> Handler:
    landed = wait_for_landing(then)

    def handler(drone: Drone) -> Step | None:
        next_step = landed(drone)
        if next_step is not None:
            return next_step

        if drone.time_in_step() > timeout:
            drone.get_logger().error("Mission did not finish in time")
            return abort

        x, y, z = drone.position()
        drone.get_logger().info(
            f"x={x:.1f}, y={y:.1f}, altitude {z:.1f} m, "
            f"speed {drone.horizontal_speed():.1f} m/s",
            throttle_duration_sec=2.0,
        )
        return None

    return handler


def wait_for_disarm(then: Step, timeout: float = 30.0) -> Handler:
    def handler(drone: Drone) -> Step | None:
        if drone.status is not None and not drone.status.armed:
            drone.get_logger().info("Disarmed")
            return then

        fail_after(drone, "Vehicle landed but did not disarm", timeout)
        return None

    return handler


# --- landing ---


def wait_for_landing(then: Step) -> Handler:
    def handler(drone: Drone) -> Step | None:
        if drone.status is None or drone.status.flying:
            return None

        if drone.odometry_age() > ODOMETRY_TIMEOUT:
            drone.get_logger().info("Landed, position unknown: no odometry")
            return then

        x, y, z = drone.position()
        drone.get_logger().info(f"Landed at x={x:.2f}, y={y:.2f}, altitude {z:.2f} m")
        return then

    return handler
