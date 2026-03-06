"""LLM decision maker — connects to LM Studio for action planning."""

from openai import OpenAI

SYSTEM_PROMPT = """You are the decision-making brain of an autonomous robot.
You receive scene descriptions from a camera and sensor data.
You must decide the SINGLE next action.

Available actions (respond with EXACTLY one):
- FORWARD <speed 0-100>
- TURN_LEFT <degrees 1-180>
- TURN_RIGHT <degrees 1-180>
- STOP
- INVESTIGATE <direction: left/right/forward>
- ALERT <short message>

Rules:
- If obstacle is closer than 300mm, STOP or TURN to avoid.
- If path is clear, FORWARD at appropriate speed.
- If task is patrol, follow waypoints in order.
- If task is surveillance, INVESTIGATE anything unusual.
- Always prioritize safety over task completion.
- Respond with ONLY the action, nothing else.

Current task: {task}
"""


class Planner:
    """Decides the next robot action using an LLM."""

    def __init__(self, host: str = "127.0.0.1", port: int = 1234, model: str = "llama-3.2-3b"):
        self.client = OpenAI(
            base_url=f"http://{host}:{port}/v1",
            api_key="not-needed",
        )
        self.model = model

    def decide(self, scene: str, sensors: dict, task: str, odom: dict) -> str:
        """Decide the next action based on perception and sensor data.

        Args:
            scene: Text description from VLM.
            sensors: Dict with accel, gyro, range_front, range_right, battery.
            task: Current task description (e.g., "patrol A-B-C").
            odom: Dict with dist_mm, heading_cdeg.

        Returns:
            Action string (e.g., "FORWARD 60", "TURN_RIGHT 45", "STOP").
        """
        user_msg = (
            f"Scene: {scene}\n"
            f"Range front: {sensors.get('range_front_mm', '?')}mm\n"
            f"Range right: {sensors.get('range_right_mm', '?')}mm\n"
            f"Odometry: dist={odom.get('dist_mm', 0)}mm, "
            f"heading={odom.get('heading_cdeg', 0) / 100:.1f} deg\n"
            f"Battery: {sensors.get('battery_mv', 0)}mV\n"
            f"\nWhat is your next action?"
        )

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT.format(task=task)},
                {"role": "user", "content": user_msg},
            ],
            max_tokens=30,
            temperature=0.1,
        )
        return response.choices[0].message.content.strip()
