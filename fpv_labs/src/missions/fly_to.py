from collections.abc import Callable
from enum import Enum, auto

from drone import Mission, Phase, Position, Step
from handlers import aim, fly_to_target, stop_at_target


class FlyToStep(Enum):
    AIM = auto()
    FLY = auto()
    STOP = auto()


def fly_to(
    compute_target: Callable[[Position], Position],
    max_speed: float = 1.0,
    tolerance: float = 0.2,
) -> Phase:
    """compute_target(home) -> (x, y, z): target in the /odometry frame."""

    def phase(then: Step, abort: Step) -> Mission:
        return {
            FlyToStep.AIM: aim(compute_target, then=FlyToStep.FLY),
            FlyToStep.FLY: fly_to_target(
                then=FlyToStep.STOP, abort=abort, max_speed=max_speed
            ),
            FlyToStep.STOP: stop_at_target(
                then=then,
                retry=FlyToStep.FLY,
                abort=abort,
                tolerance=tolerance,
            ),
        }

    return phase
