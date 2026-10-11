"""Lab 2, task 4: fly to a point and land."""

import math
import sys

from drone import Position, run
from missions.chain import chain
from missions.fly_to import fly_to
from missions.landing import landing
from missions.takeoff import takeoff
from missions.wait import wait

# Variant 21
DELTA_X = 7.0
DELTA_Y = -0.4
ALTITUDE = 2.0

# Rotation of the task frame relative to /odometry about the Z axis
TASK_FRAME_YAW = math.radians(180.0)


def target(home: Position) -> Position:
    x0, y0, z0 = home
    cos_yaw = math.cos(TASK_FRAME_YAW)
    sin_yaw = math.sin(TASK_FRAME_YAW)

    return (
        x0 - DELTA_X * cos_yaw + DELTA_Y * sin_yaw,
        y0 - DELTA_X * sin_yaw - DELTA_Y * cos_yaw,
        z0 + ALTITUDE,
    )


MISSION = chain(
    takeoff(ALTITUDE),
    wait(3.0),
    fly_to(target, tolerance=0.2),
    landing(),
)

if __name__ == "__main__":
    sys.exit(run("lab_2_go_to", MISSION))
