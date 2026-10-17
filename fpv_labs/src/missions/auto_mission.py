from enum import Enum, auto

from drone import Mission, Phase, Step
from handlers import (
    AUTO,
    follow_mission,
    set_mode,
    set_parameter,
    start_mission,
    upload_mission,
    wait_for_airborne,
    wait_for_disarm,
    wait_for_mavlink,
)
from links.mavlink import MissionItem
from missions.prepare import PrepareStep, prepare

# m/s; WPNAV_SPEED (cm/s) before ArduPilot renamed it
SPEED_PARAMETER = "WP_SPD"
# SITL Iris enables it without a beacon: the landing hovers at 10 m looking for one
PRECISION_LANDING_PARAMETER = "PLND_ENABLED"


class AutoMissionStep(Enum):
    WAIT_FOR_MAVLINK = auto()
    SET_SPEED = auto()
    SET_PRECISION_LANDING = auto()
    UPLOAD = auto()
    SET_AUTO = auto()
    START = auto()
    TAKE_OFF = auto()
    FLY = auto()
    DISARM = auto()


def auto_mission(
    items: list[MissionItem], speed: float, precision_landing: bool
) -> Phase:

    def phase(then: Step, abort: Step) -> Mission:
        return {
            AutoMissionStep.WAIT_FOR_MAVLINK: wait_for_mavlink(
                then=AutoMissionStep.SET_SPEED
            ),
            AutoMissionStep.SET_SPEED: set_parameter(
                SPEED_PARAMETER, speed, then=AutoMissionStep.SET_PRECISION_LANDING
            ),
            AutoMissionStep.SET_PRECISION_LANDING: set_parameter(
                PRECISION_LANDING_PARAMETER,
                float(precision_landing),
                then=AutoMissionStep.UPLOAD,
            ),
            AutoMissionStep.UPLOAD: upload_mission(
                items, then=PrepareStep.WAIT_FOR_SYSTEM
            ),
            **prepare()(AutoMissionStep.SET_AUTO, abort),
            AutoMissionStep.SET_AUTO: set_mode(AUTO, then=AutoMissionStep.START),
            AutoMissionStep.START: start_mission(then=AutoMissionStep.TAKE_OFF),
            AutoMissionStep.TAKE_OFF: wait_for_airborne(
                then=AutoMissionStep.FLY, abort=abort
            ),
            AutoMissionStep.FLY: follow_mission(
                then=AutoMissionStep.DISARM, abort=abort
            ),
            AutoMissionStep.DISARM: wait_for_disarm(then=then),
        }

    return phase
