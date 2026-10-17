from enum import Enum, auto

from drone import Mission, Phase, Step
from handlers import climb, take_off
from missions.prepare import prepare


class TakeoffStep(Enum):
    TAKE_OFF = auto()
    CLIMB = auto()


def takeoff(altitude: float) -> Phase:
    def phase(then: Step, abort: Step) -> Mission:
        return {
            **prepare()(TakeoffStep.TAKE_OFF, abort),
            TakeoffStep.TAKE_OFF: take_off(altitude, then=TakeoffStep.CLIMB),
            TakeoffStep.CLIMB: climb(then=then, abort=abort),
        }

    return phase
