"""Joins phases into one mission and adds a shared abort branch."""

from enum import Enum, auto

from drone import End, Mission, Phase, Step
from handlers import LAND, set_mode, wait_for_landing


class AbortStep(Enum):
    ABORT = auto()
    ABORT_TOUCHDOWN = auto()


def chain(*phases: Phase) -> Mission:
    """Phases run in order; the mission succeeds after the last one."""

    mission: Mission = {
        AbortStep.ABORT: set_mode(LAND, then=AbortStep.ABORT_TOUCHDOWN),
        AbortStep.ABORT_TOUCHDOWN: wait_for_landing(then=End.FAILED),
    }
    then: Step = End.DONE

    for phase in reversed(phases):
        steps = phase(then, AbortStep.ABORT)

        if steps.keys() & mission.keys():
            raise ValueError("The same phase is used twice in one mission")

        mission = {**steps, **mission}
        then = next(iter(steps))

    return mission
