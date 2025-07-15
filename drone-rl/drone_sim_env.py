from typing import Any, Optional

import gymnasium as gym
import numpy as np

from gymnasium.spaces import Box

import logging, time

try:
    from ros_interface import DroneROSInterface
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

ResetReturn = tuple[np.ndarray, dict[str, Any]]
StepReturn = tuple[np.ndarray, float, bool, bool, dict[str, Any]]


def _distance(a: "np.ndarray | tuple[float, ...]", b: "np.ndarray | tuple[float, ...]") -> float:
    """Euclidean distance with proper typing."""
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


# ==============================================================================
# Main environment
# ==============================================================================
class DroneSimEnv(gym.Env):
    def __init__(self, instance_id: int = 0, reward_mode: str = "default", episode_max_steps: int = 2000,
                 use_ros: bool = True, drone_ns: str = "/drone0", target_pos=None, init_pos=None,
                 randomization_params: Optional[dict] = None, state_timeout: float = 1.0):
        super().__init__()
        # Logger ----------------------------------------------------------------
        self.logger = logging.getLogger(f"{self.__class__.__name__}_{instance_id}")
        if not self.logger.handlers:
            # Basic configuration – ensure no duplicate handlers in multiple envs
            logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

        self.instance_id = instance_id
        self.reward_mode = reward_mode
        self.episode_max_steps = episode_max_steps
        self.current_step = 0
        self.use_ros = use_ros and ROS_AVAILABLE
        self.drone_ns = drone_ns
        self.randomization_params = randomization_params or {}
        self.target_pos = np.array(target_pos) if target_pos is not None else np.array([5.0, 0.0, 4.0])
        self.init_pos = np.array(init_pos) if init_pos is not None else np.array([0.0, 0.0, 4.0])
        self.mass = self.randomization_params.get("mass", 0.65)
        self.wind = np.array(self.randomization_params.get("wind", [0.0, 0.0, 0.0]))
        self.state_timeout = state_timeout  # seconds
        self.state = None
        self.last_obs = None
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32)  # [x,y,z,roll,pitch,yaw,vx,vy,vz,battery]
        self.action_space = Box(low=np.array([0, -1, -1, -1]), high=np.array([1, 1, 1, 1]), dtype=np.float32)  # [thrust, roll, pitch, yaw_rate]
        if self.use_ros:
            self.ros = DroneROSInterface(drone_ns)
            self.ros.set_state_callback(self._ros_state_cb)
        else:
            self.ros = None

    # ----------------------------------------------------------------------
    # Internal helpers
    # ----------------------------------------------------------------------
    def _wait_for_state(self) -> np.ndarray:
        """Block until the latest state arrives or a timeout is reached."""
        start = time.time()
        while time.time() - start < self.state_timeout:
            if self.last_obs is not None:
                return self.last_obs
            time.sleep(0.01)
        self.logger.warning("State not received within timeout (%.2fs)", self.state_timeout)
        raise TimeoutError("State not received within timeout")

    def _ros_state_cb(self, msg):
        # geometry_msgs/Point position, Vector3 orientation, Vector3 velocity, float32 battery_level
        obs = np.array([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.velocity.x, msg.velocity.y, msg.velocity.z,
            msg.battery_level
        ], dtype=np.float32)
        self.state = msg
        self.last_obs = obs

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> ResetReturn:
        import numpy as np
        self.current_step = 0
        self.state = None
        self.last_obs = None
        # --- Domain Randomization ---
        rng = np.random.default_rng(seed)
        # 初期位置
        if self.randomization_params.get("init_pos_range"):
            low, high = self.randomization_params["init_pos_range"]
            self.init_pos = rng.uniform(low, high)
        # 目標位置
        if self.randomization_params.get("target_pos_range"):
            low, high = self.randomization_params["target_pos_range"]
            self.target_pos = rng.uniform(low, high)
        # 質量
        if self.randomization_params.get("mass_range"):
            low, high = self.randomization_params["mass_range"]
            self.mass = float(rng.uniform(low, high))
        # 風
        if self.randomization_params.get("wind_range"):
            low, high = self.randomization_params["wind_range"]
            self.wind = rng.uniform(low, high)
        # Optionally: ROS2に初期位置・風を送る（未実装）
        if self.use_ros:
            self.ros.reset_sim()  # TODO: implement actual reset with params
        # Wait for first state (with timeout)
        try:
            obs = self._wait_for_state()
        except TimeoutError:
            self.logger.error("Timeout while waiting for initial state; returning zeros")
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        return obs, {}

    def step(self, action: np.ndarray) -> StepReturn:
        self.current_step += 1
        # Action: [thrust, roll, pitch, yaw_rate]
        if self.use_ros:
            self.ros.publish_control(*action)
        # Wait for new state (with timeout handling)
        try:
            obs = self._wait_for_state()
        except TimeoutError:
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            self.logger.error("Timeout waiting for state update – using zeros")
        # Reward: negative distance to target, penalize high velocity, encourage height maintenance
        pos = obs[:3]
        dist = _distance(pos, self.target_pos)
        vel = obs[6:9]
        speed = np.linalg.norm(vel)
        battery = obs[9]
        height = obs[2]
        reward = -dist - 0.1*speed + 0.05*max(0, height)
        # Done criteria
        terminated = (
            dist < 0.5 or
            height < 0.2 or
            battery < 0.05 or
            self.current_step >= self.episode_max_steps or
            speed > 25.0 or
            np.any(np.abs(pos) > 100.0)
        )
        truncated = False  # Could add logic for timeout-based truncation
        info = {"distance": dist, "speed": speed, "battery": battery, "height": height, "step": self.current_step}
        return obs, reward, terminated, truncated, info

    def render(self) -> None:
        pass

    def close(self) -> None:
        pass 