"""Lab 2, task 3: take off to 2 m, hover, land."""

import sys

from drone import run
from missions.chain import chain
from missions.landing import landing
from missions.takeoff import takeoff
from missions.wait import wait

MISSION = chain(
    takeoff(altitude=2.0),
    wait(10.0),
    landing(),
)

if __name__ == "__main__":
    sys.exit(run("lab_2_plain_start", MISSION))
