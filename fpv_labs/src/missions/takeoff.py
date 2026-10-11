from enum import Enum, auto

from drone import Mission, Phase, Step
from handlers import (
    GUIDED,
    arm,
    climb,
    prearm_check,
    set_mode,
    take_off,
    wait_for_system,
)


class TakeoffStep(Enum):
    WAIT_FOR_SYSTEM = auto()
    PREARM_CHECK = auto()
    SET_GUIDED = auto()
    ARM = auto()
    TAKE_OFF = auto()
    CLIMB = auto()


def takeoff(altitude: float) -> Phase:
    def phase(then: Step, abort: Step) -> Mission:
        return {
            TakeoffStep.WAIT_FOR_SYSTEM: wait_for_system(then=TakeoffStep.PREARM_CHECK),
            TakeoffStep.PREARM_CHECK: prearm_check(then=TakeoffStep.SET_GUIDED),
            TakeoffStep.SET_GUIDED: set_mode(GUIDED, then=TakeoffStep.ARM),
            TakeoffStep.ARM: arm(then=TakeoffStep.TAKE_OFF),
            TakeoffStep.TAKE_OFF: take_off(altitude, then=TakeoffStep.CLIMB),
            TakeoffStep.CLIMB: climb(then=then, abort=abort),
        }

    return phase
