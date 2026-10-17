"""Lab 7: automatic mission Takeoff -> Waypoint -> Land over MAVLink"""

import sys

from drone import run
from links.mavlink import MavlinkLink, MissionItem
from missions.auto_mission import auto_mission
from missions.chain import chain

CONNECTION = "udpin:127.0.0.1:14551"

# Variant 21
ALTITUDE = 50.0
SPEED = 5.0
TARGET = (-35.3633350, 149.1639641)

PRECISION_LANDING = False

MISSION = chain(
    auto_mission(
        [
            MissionItem.takeoff(ALTITUDE),
            MissionItem.waypoint(TARGET, ALTITUDE),
            MissionItem.land(),
        ],
        speed=SPEED,
        precision_landing=PRECISION_LANDING,
    ),
)

if __name__ == "__main__":
    sys.exit(run("lab_7_flight", MISSION, MavlinkLink(CONNECTION)))
