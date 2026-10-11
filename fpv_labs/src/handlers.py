import math
from collections.abc import Callable

from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from std_srvs.srv import Trigger

from drone import Drone, Handler, MissionError, Position, Step

GUIDED = 4
LAND = 9

# /odometry arrives 50 times per second
ODOMETRY_TIMEOUT = 0.5
ODOMETRY_ABORT_TIMEOUT = 5.0

ALTITUDE_GAIN = 0.8
MAX_CLIMB_SPEED = 0.5


# --- helpers ---


def require_armed(drone: Drone) -> None:
    if drone.status is None or not drone.status.armed:
        raise MissionError(f"Drone disarmed unexpectedly during {drone.step.name}")


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
        if drone.future is None:
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
