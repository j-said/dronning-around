from enum import Enum, auto

from drone import Mission, Phase, Step
from handlers import GUIDED, arm, prearm_check, set_mode, wait_for_system


class PrepareStep(Enum):
    WAIT_FOR_SYSTEM = auto()
    PREARM_CHECK = auto()
    SET_GUIDED = auto()
    ARM = auto()


def prepare() -> Phase:
    """System ready, pre-arm check passed, GUIDED mode, motors armed."""

    def phase(then: Step, _abort: Step) -> Mission:
        return {
            PrepareStep.WAIT_FOR_SYSTEM: wait_for_system(then=PrepareStep.PREARM_CHECK),
            PrepareStep.PREARM_CHECK: prearm_check(then=PrepareStep.SET_GUIDED),
            PrepareStep.SET_GUIDED: set_mode(GUIDED, then=PrepareStep.ARM),
            PrepareStep.ARM: arm(then=then),
        }

    return phase
