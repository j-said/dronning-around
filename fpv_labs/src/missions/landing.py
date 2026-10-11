from enum import Enum, auto

from drone import Mission, Phase, Step
from handlers import LAND, set_mode, wait_for_landing


class LandingStep(Enum):
    LAND = auto()
    TOUCHDOWN = auto()


def landing() -> Phase:
    def phase(then: Step, _abort: Step) -> Mission:
        return {
            LandingStep.LAND: set_mode(LAND, then=LandingStep.TOUCHDOWN),
            LandingStep.TOUCHDOWN: wait_for_landing(then=then),
        }

    return phase
