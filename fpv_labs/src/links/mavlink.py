"""MAVLink channel for what the ArduPilot DDS interface lacks: missions, parameters."""

import os
from dataclasses import dataclass

from pymavlink import mavutil
from pymavlink.dialects.v20 import all as dialect

os.environ["MAVLINK20"] = "1"

type Message = dialect.MAVLink_message
type GeoPoint = tuple[float, float]
"""latitude, longitude: degrees WGS 84"""

Heartbeat = dialect.MAVLink_heartbeat_message
StatusText = dialect.MAVLink_statustext_message
ParameterValue = dialect.MAVLink_param_value_message
MissionRequest = dialect.MAVLink_mission_request_message
MissionRequestInt = dialect.MAVLink_mission_request_int_message
MissionAcknowledgement = dialect.MAVLink_mission_ack_message
MissionItemReached = dialect.MAVLink_mission_item_reached_message


@dataclass(frozen=True)
class MissionItem:
    command: int
    point: GeoPoint = (0.0, 0.0)
    altitude: float = 0.0
    """m above home"""

    @classmethod
    def home(cls) -> "MissionItem":
        """Item 0: ArduPilot replaces it with the home position."""
        return cls(dialect.MAV_CMD_NAV_WAYPOINT)

    @classmethod
    def takeoff(cls, altitude: float) -> "MissionItem":
        return cls(dialect.MAV_CMD_NAV_TAKEOFF, altitude=altitude)

    @classmethod
    def waypoint(cls, point: GeoPoint, altitude: float) -> "MissionItem":
        return cls(dialect.MAV_CMD_NAV_WAYPOINT, point, altitude)

    @classmethod
    def land(cls) -> "MissionItem":
        """Zero coordinates: land where the vehicle is."""
        return cls(dialect.MAV_CMD_NAV_LAND)


class MavlinkLink:
    """Non-blocking: Drone polls it every tick"""

    def __init__(self, connection: str) -> None:
        self.address = connection
        self.connection = mavutil.mavlink_connection(connection, dialect="all")
        self.system = 0
        self.component = 0
        self.messages: list[Message] = []

    @property
    def connected(self) -> bool:
        return self.system != 0

    def poll(self) -> list[Message]:
        self.messages = []

        while (message := self.connection.recv_match(blocking=False)) is not None:
            # Undecodable bytes arrive as BAD_DATA
            if not isinstance(message, dialect.MAVLink_message) or isinstance(
                message, dialect.MAVLink_bad_data
            ):
                continue

            # Ground stations send HEARTBEAT too; only the autopilot is the target
            if (
                isinstance(message, Heartbeat)
                and message.autopilot != dialect.MAV_AUTOPILOT_INVALID
            ):
                self.system = message.get_srcSystem()
                self.component = message.get_srcComponent()

            self.messages.append(message)

        return self.messages

    def received[MessageType: dialect.MAVLink_message](
        self, kind: type[MessageType]
    ) -> list[MessageType]:
        """Messages of one class from the last poll, with typed fields"""
        return [message for message in self.messages if isinstance(message, kind)]

    def set_parameter(self, name: str, value: float) -> None:
        self.connection.mav.param_set_send(
            target_system=self.system,
            target_component=self.component,
            param_id=name.encode(),
            param_value=value,
            param_type=dialect.MAV_PARAM_TYPE_REAL32,
        )

    def send_mission_count(self, count: int) -> None:
        self.connection.mav.mission_count_send(
            target_system=self.system,
            target_component=self.component,
            count=count,
        )

    def send_mission_item(self, sequence: int, item: MissionItem) -> None:
        latitude, longitude = item.point
        self.connection.mav.mission_item_int_send(
            target_system=self.system,
            target_component=self.component,
            seq=sequence,
            frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=item.command,
            current=0,
            autocontinue=1,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            x=round(latitude * 1e7),
            y=round(longitude * 1e7),
            z=item.altitude,
        )

    def start_mission(self) -> None:
        self.connection.mav.command_long_send(
            target_system=self.system,
            target_component=self.component,
            command=dialect.MAV_CMD_MISSION_START,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )

    def close(self) -> None:
        self.connection.close()


def mission_accepted(acknowledgement: MissionAcknowledgement) -> bool:
    return acknowledgement.type == dialect.MAV_MISSION_ACCEPTED
