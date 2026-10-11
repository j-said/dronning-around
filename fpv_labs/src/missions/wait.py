from enum import Enum, auto

from drone import Mission, Phase, Step
from handlers import hover


class WaitStep(Enum):
    WAIT = auto()


def wait(duration: float) -> Phase:
    def phase(then: Step, _abort: Step) -> Mission:
        return {WaitStep.WAIT: hover(duration, then=then)}

    return phase
