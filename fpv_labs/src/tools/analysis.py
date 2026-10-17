"""Flight indicators and plots from QGroundControl telemetry logs."""

import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import cast

import pandas as pd
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from pymavlink import mavutil

from links.mavlink import Heartbeat, dialect

# QGC CSV: one row per second, vehicle facts by name, groups as "<group>.<fact>"
TIME = "Timestamp"
ALTITUDE = "altitudeRelative"
AIRSPEED = "airSpeed"
GROUNDSPEED = "groundSpeed"
CLIMB_RATE = "climbRate"
ROLL = "roll"
LATITUDE = "gps.lat"
LONGITUDE = "gps.lon"

GROUND_LEVEL = 0.5  # m
CRUISE_FRACTION = 0.95  # of the maximum altitude

PHASE_LABELS = {
    "takeoff": "зліт",
    "cruise_start": "досягнуто висоту",
    "cruise_end": "початок посадки",
    "touchdown": "посадка",
}


@dataclass(frozen=True)
class Telemetry:
    time: list[float]  # s from the first row
    altitude: list[float]
    airspeed: list[float]
    groundspeed: list[float]
    climb_rate: list[float]  # m/s, negative when descending
    roll: list[float]
    latitude: list[float]
    longitude: list[float]


@dataclass(frozen=True)
class VehicleStates:
    modes: list[str]
    armed_before: bool
    armed_after: bool


def column(table: pd.DataFrame, name: str) -> list[float]:
    """cast: Ubuntu's pandas and matplotlib have no type annotations"""
    # Non-numeric cells (QGC writes them for missing values) become NaN
    numbers = cast(
        pd.Series, pd.to_numeric(cast(pd.Series, table[name]), errors="coerce")
    )
    return cast(list[float], numbers.tolist())


def load(path: Path) -> Telemetry:
    table = pd.read_csv(path)
    names = [
        TIME,
        ALTITUDE,
        AIRSPEED,
        GROUNDSPEED,
        CLIMB_RATE,
        ROLL,
        LATITUDE,
        LONGITUDE,
    ]
    missing = [name for name in names if name not in table.columns]
    if missing:
        raise SystemExit(
            f"{path.name} has no columns {missing}; available: {list(table.columns)}"
        )

    timestamps = [
        datetime.fromisoformat(text)
        for text in cast(list[str], table[TIME].astype(str).tolist())
    ]
    return Telemetry(
        time=[(stamp - timestamps[0]).total_seconds() for stamp in timestamps],
        altitude=column(table, ALTITUDE),
        airspeed=column(table, AIRSPEED),
        groundspeed=column(table, GROUNDSPEED),
        climb_rate=column(table, CLIMB_RATE),
        roll=column(table, ROLL),
        latitude=column(table, LATITUDE),
        longitude=column(table, LONGITUDE),
    )


def vehicle_states(tlog_path: Path) -> VehicleStates:
    """QGC CSV has no mode or arm state"""
    log = mavutil.mavlink_connection(str(tlog_path), dialect="all")
    modes: list[str] = []
    armed: list[bool] = []

    while (message := log.recv_match(type="HEARTBEAT")) is not None:
        # The log also holds HEARTBEATs of QGC itself
        if (
            not isinstance(message, Heartbeat)
            or message.autopilot == dialect.MAV_AUTOPILOT_INVALID
        ):
            continue
        modes.append(mavutil.mode_string_v10(message))
        armed.append(bool(message.base_mode & dialect.MAV_MODE_FLAG_SAFETY_ARMED))

    if not armed:
        raise SystemExit(f"No autopilot HEARTBEAT in {tlog_path.name}")

    # dict.fromkeys: unique modes in the order they were first seen
    return VehicleStates(list(dict.fromkeys(modes)), armed[0], armed[-1])


def peak(values: list[float]) -> float:
    return round(max(value for value in values if not math.isnan(value)), 2)


def phases(telemetry: Telemetry) -> dict[str, float]:
    samples = list(zip(telemetry.time, telemetry.altitude, strict=True))
    airborne = [time for time, altitude in samples if altitude > GROUND_LEVEL]
    if not airborne:
        raise SystemExit("The vehicle never left the ground, nothing to analyse")
    cruise_altitude = CRUISE_FRACTION * peak(telemetry.altitude)
    at_cruise = [time for time, altitude in samples if altitude >= cruise_altitude]
    return {
        "takeoff": airborne[0],
        "cruise_start": at_cruise[0],
        "cruise_end": at_cruise[-1],
        "touchdown": airborne[-1],
    }


def indicators(
    telemetry: Telemetry, phase_times: dict[str, float], states: VehicleStates
) -> pd.Series:
    # Rows before the GPS fix hold zero or NaN coordinates
    fixes = [
        (latitude, longitude)
        for latitude, longitude in zip(
            telemetry.latitude, telemetry.longitude, strict=True
        )
        if latitude and longitude and not math.isnan(latitude + longitude)
    ]
    (start_latitude, start_longitude), (end_latitude, end_longitude) = (
        fixes[0],
        fixes[-1],
    )
    return pd.Series(
        {
            "Час зльоту, с": round(
                phase_times["cruise_start"] - phase_times["takeoff"], 1
            ),
            "Найбільша висота, м": peak(telemetry.altitude),
            "Найбільша повітряна швидкість, м/с": peak(telemetry.airspeed),
            "Найбільша наземна швидкість, м/с": peak(telemetry.groundspeed),
            # Magnitude: the fastest climb or descent, whichever is larger
            "Найбільша вертикальна швидкість, м/с": peak(
                [abs(rate) for rate in telemetry.climb_rate]
            ),
            "Час польоту, с": round(
                phase_times["touchdown"] - phase_times["takeoff"], 1
            ),
            "Початкова координата": f"{start_latitude:.7f}, {start_longitude:.7f}",
            "Кінцева координата": f"{end_latitude:.7f}, {end_longitude:.7f}",
            "Максимальний roll, град": peak([abs(roll) for roll in telemetry.roll]),
            "Режими польоту": ", ".join(states.modes),
            "Arm до і після місії": f"{states.armed_before} / {states.armed_after}",
        }
    )


def plot(
    telemetry: Telemetry, phase_times: dict[str, float], output_directory: Path
) -> None:
    charts = {
        "altitude": (telemetry.altitude, "Висота, м"),
        "airspeed": (telemetry.airspeed, "Повітряна швидкість, м/с"),
        "groundspeed": (telemetry.groundspeed, "Наземна швидкість, м/с"),
        "climb_rate": (telemetry.climb_rate, "Вертикальна швидкість, м/с"),
    }
    for name, (series, label) in charts.items():
        figure = Figure(figsize=(8, 4))
        axes = cast(Axes, figure.add_subplot())
        axes.plot(telemetry.time, series, label=label)
        # Lowest value via peak of the negated series: climb rate goes below zero
        bottom = min(0.0, -peak([-value for value in series]))
        top = peak(series)
        # A vertical dashed line at the start of each flight phase
        for key, text in PHASE_LABELS.items():
            axes.plot([phase_times[key]] * 2, [bottom, top], linestyle="--", label=text)
        axes.set(xlabel="Час, с", ylabel=label, title=label)
        axes.legend()
        figure.tight_layout()
        figure.savefig(output_directory / f"{name}.png", dpi=150)


def latest(log_directory: Path, pattern: str, skip: Path) -> Path:
    # QGC keeps logs in subdirectories; our own results live under skip
    files = [path for path in log_directory.rglob(pattern) if skip not in path.parents]
    if not files:
        # QGC moves the .tlog out of /tmp only once the vehicle disconnects
        raise SystemExit(
            f"No {pattern} in {log_directory}; after the flight run make stop "
            "or close QGC so it saves the logs"
        )
    return max(files, key=lambda path: path.stat().st_mtime)


def analyze(log_directory: Path) -> int:
    output_directory = log_directory / "analysis"
    csv_path = latest(log_directory, "*.csv", skip=output_directory)
    tlog_path = latest(log_directory, "*.tlog", skip=output_directory)

    telemetry = load(csv_path)
    phase_times = phases(telemetry)
    result = indicators(telemetry, phase_times, vehicle_states(tlog_path))

    output_directory.mkdir(parents=True, exist_ok=True)
    result.to_csv(
        output_directory / f"{csv_path.stem}_indicators.csv", header=["Значення"]
    )
    plot(telemetry, phase_times, output_directory)

    print(f"Logs: {csv_path.name}, {tlog_path.name}")
    print(result.to_string())
    print(f"Results and plots: {output_directory}")
    return 0
