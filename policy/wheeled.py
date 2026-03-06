"""Wheeled robot policy translator — diff drive.

Converts skill names + args into ActuatorCmd for a differential drive robot.
Channels: [speed_l, speed_r]  (-max_speed .. +max_speed)
"""

import re
from protocol import ActuatorCmd, ACT_DIFF_DRIVE, FLAG_EMERGENCY, FLAG_ALERT


def _extract_number(text: str, default: int = 50) -> int:
    match = re.search(r"-?\d+", text)
    return int(match.group()) if match else default


class WheeledPolicy:
    """Policy translator for differential drive robots."""

    def __init__(self, max_speed: int = 80):
        self.max_speed = max_speed

    def translate(self, skill: str, args: dict | None = None, sensors: dict | None = None) -> ActuatorCmd:
        """Translate a skill name into an ActuatorCmd.

        Args:
            skill:   Skill name (FORWARD, TURN_LEFT, STOP, ...) — case insensitive.
            args:    Optional skill arguments (speed, degrees, duration...).
            sensors: Latest sensor readings (for reactive skills).

        Returns:
            ActuatorCmd for diff drive (type=0, 2 channels).
        """
        args = args or {}
        s = skill.strip().upper()

        if s in ("EMERGENCY", "E_STOP"):
            return ActuatorCmd.wheeled(0, 0, flags=FLAG_EMERGENCY)

        if s in ("STOP", "WAIT"):
            return ActuatorCmd.wheeled(0, 0)

        if s == "FORWARD":
            speed = min(int(args.get("speed", 60)), self.max_speed)
            return ActuatorCmd.wheeled(speed, speed)

        if s in ("BACKWARD", "REVERSE"):
            speed = min(int(args.get("speed", 30)), self.max_speed)
            return ActuatorCmd.wheeled(-speed, -speed)

        if s == "TURN_RIGHT":
            degrees = int(args.get("degrees", 45))
            intensity = min(degrees * self.max_speed // 90, self.max_speed)
            return ActuatorCmd.wheeled(intensity, -intensity)

        if s == "TURN_LEFT":
            degrees = int(args.get("degrees", 45))
            intensity = min(degrees * self.max_speed // 90, self.max_speed)
            return ActuatorCmd.wheeled(-intensity, intensity)

        if s == "INVESTIGATE":
            speed = int(args.get("speed", 20))
            return ActuatorCmd.wheeled(speed, speed)

        if s == "TRACK":
            speed = int(args.get("speed", 30))
            return ActuatorCmd.wheeled(speed, speed)

        if s == "ALERT":
            return ActuatorCmd.wheeled(0, 0, flags=FLAG_ALERT)

        if s == "SCAN_360":
            # Rotate in place at low speed — caller manages timing per step
            speed = int(args.get("speed", 25))
            return ActuatorCmd.wheeled(speed, -speed)

        if s == "FOLLOW_WALL":
            # Slow forward — caller keeps wall at distance via sensor feedback
            speed = int(args.get("speed", 30))
            return ActuatorCmd.wheeled(speed, speed)

        # Unknown skill — safe default
        return ActuatorCmd.wheeled(0, 0)

    def from_text(self, action_text: str) -> ActuatorCmd:
        """Translate a free-text LLM action string (legacy / direct LLM output).

        Supports: "FORWARD 60", "TURN_RIGHT 45", "STOP", "ALERT ...", etc.
        """
        action = action_text.strip().upper()

        if "EMERGENCY" in action or "E_STOP" in action:
            return ActuatorCmd.wheeled(0, 0, flags=FLAG_EMERGENCY)
        if "STOP" in action:
            return ActuatorCmd.wheeled(0, 0)
        if "FORWARD" in action:
            speed = min(_extract_number(action, 60), self.max_speed)
            return ActuatorCmd.wheeled(speed, speed)
        if "BACKWARD" in action or "REVERSE" in action:
            speed = min(_extract_number(action, 30), self.max_speed)
            return ActuatorCmd.wheeled(-speed, -speed)
        if "TURN_RIGHT" in action or "RIGHT" in action:
            degrees = _extract_number(action, 45)
            intensity = min(degrees * self.max_speed // 90, self.max_speed)
            return ActuatorCmd.wheeled(intensity, -intensity)
        if "TURN_LEFT" in action or "LEFT" in action:
            degrees = _extract_number(action, 45)
            intensity = min(degrees * self.max_speed // 90, self.max_speed)
            return ActuatorCmd.wheeled(-intensity, intensity)
        if "INVESTIGATE" in action:
            return ActuatorCmd.wheeled(20, 20)
        if "ALERT" in action:
            return ActuatorCmd.wheeled(0, 0, flags=FLAG_ALERT)

        return ActuatorCmd.wheeled(0, 0)
