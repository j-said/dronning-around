import time
from collections.abc import Callable
from enum import Enum, auto
from typing import Any

import rclpy
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger

from links.mavlink import MavlinkLink, Message, MissionItemReached, StatusText


class End(Enum):
    DONE = auto()
    FAILED = auto()


class MissionError(Exception):
    pass


type Step = Enum
type Handler = Callable[["Drone"], Step | None]
type Mission = dict[Step, Handler]
type Phase = Callable[[Step, Step], Mission]
type Position = tuple[float, float, float]


class Drone(Node):
    TICK = 0.2

    def __init__(
        self, name: str, mission: Mission, mavlink: MavlinkLink | None = None
    ) -> None:
        super().__init__(name)

        self.mission = mission
        self.mavlink = mavlink

        self.step: Step = next(iter(mission))
        self.entered_at = time.monotonic()
        self.first_tick = True
        self.future: Future | None = None

        self.status: Status | None = None
        self.odometry: Odometry | None = None
        self.odometry_received_at = 0.0

        self.home: Position = (0.0, 0.0, 0.0)
        self.target: Position = (0.0, 0.0, 0.0)
        self.arrival_error: float | None = None

        # Subs
        self.create_subscription(Status, "/ap/v1/status", self.on_status, 10)
        self.create_subscription(Odometry, "/odometry", self.on_odometry, 10)

        self.velocity_publisher = self.create_publisher(
            TwistStamped, "/ap/v1/cmd_vel", 10
        )

        # Clients
        self.prearm_client = self.create_client(Trigger, "/ap/v1/prearm_check")
        self.mode_client = self.create_client(ModeSwitch, "/ap/v1/mode_switch")
        self.arm_client = self.create_client(ArmMotors, "/ap/v1/arm_motors")
        self.takeoff_client = self.create_client(Takeoff, "/ap/v1/experimental/takeoff")

        self.create_timer(self.TICK, self.tick)
        self.get_logger().info(f"Mission started, step {self.step.name}")

    def on_status(self, message: Status) -> None:
        self.status = message

    def on_odometry(self, message: Odometry) -> None:
        self.odometry = message
        self.odometry_received_at = time.monotonic()

    def on_mavlink(self, message: Message) -> None:
        if isinstance(message, StatusText):
            self.get_logger().info(f"ArduPilot: {message.text}")
        elif isinstance(message, MissionItemReached):
            self.get_logger().info(f"Mission item {message.seq} reached")

    def tick(self) -> None:
        if isinstance(self.step, End):
            return

        if self.mavlink is not None:
            for message in self.mavlink.poll():
                self.on_mavlink(message)

        next_step = self.mission[self.step](self)
        self.first_tick = False

        if next_step is not None:
            self.go(next_step)

    def go(self, next_step: Step) -> None:
        self.get_logger().info(f"Step {self.step.name} -> {next_step.name}")
        self.step = next_step
        self.entered_at = time.monotonic()
        self.first_tick = True
        self.future = None

    def time_in_step(self) -> float:
        return time.monotonic() - self.entered_at

    def entering(self) -> bool:
        return self.first_tick

    def link(self) -> MavlinkLink:
        if self.mavlink is None:
            raise MissionError(f"Step {self.step.name} needs a MAVLink connection")

        return self.mavlink

    def services_ready(self) -> bool:
        return (
            self.prearm_client.service_is_ready()
            and self.mode_client.service_is_ready()
            and self.arm_client.service_is_ready()
            and self.takeoff_client.service_is_ready()
        )

    def request(self, client: Client, message: object) -> Any | None:  # noqa: ANN401
        """Once per Step"""
        if self.entering():
            self.future = client.call_async(message)
            return None

        if self.future is None or not self.future.done():
            return None

        return self.future.result()

    def current_odometry(self) -> Odometry:
        if self.odometry is None:
            raise MissionError("No odometry received")

        return self.odometry

    def position(self) -> Position:
        point = self.current_odometry().pose.pose.position
        return (point.x, point.y, point.z)

    def horizontal_speed(self) -> float:
        velocity = self.current_odometry().twist.twist.linear
        return (velocity.x**2 + velocity.y**2) ** 0.5

    def odometry_age(self) -> float:
        return time.monotonic() - self.odometry_received_at

    def publish_velocity(self, east: float, north: float, up: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = east
        message.twist.linear.y = north
        message.twist.linear.z = up

        self.velocity_publisher.publish(message)


def run(name: str, mission: Mission, mavlink: MavlinkLink | None = None) -> int:
    rclpy.init()
    drone = Drone(name, mission, mavlink)

    try:
        while rclpy.ok() and not isinstance(drone.step, End):
            rclpy.spin_once(drone, timeout_sec=Drone.TICK)
    except MissionError as error:
        drone.get_logger().error(str(error))
    except KeyboardInterrupt:
        pass
    finally:
        succeeded = drone.step is End.DONE
        drone.destroy_node()

        if mavlink is not None:
            mavlink.close()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if succeeded else 1
