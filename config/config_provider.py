"""
Configuration Provider for Duckiebot Navigation System

This module loads configuration from bot_config_template.json and provides
a clean API for accessing config values throughout the codebase.

Usage:
    from config.config_provider import config

    # Get simple values
    start_node = config.get('navigation.start_node')

    # Get environment-specific values (auto-detects sim vs real)
    creep_speed = config.get_speed('creep_speed')
    turn_time = config.get_timing('turn_time_left')

    # Update values at runtime
    config.set('navigation.start_node', 2)

    # Save changes back to file
    config.save()
"""

import json
import os
from typing import Any, Dict, List, Optional


class ConfigProvider:
    """Centralized configuration provider with support for nested keys and environment detection."""

    def __init__(self, config_path: Optional[str] = None, bot_name: str = "default"):
        """
        Initialize the configuration provider.

        Args:
            config_path: Path to the configuration JSON file. If None, uses default location.
            bot_name: Name of the bot configuration to load (default: "default")
        """
        self._bot_name = bot_name
        self.config_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "bots"
        )

        if config_path is None:
            config_path = os.path.join(self.config_dir, f"{bot_name}.json")

        self.config_path = config_path
        self._config: Dict[str, Any] = {}
        self._is_real: Optional[bool] = None
        self._load_config()
        self._detect_environment()

    def _get_config_path(self, bot_name: str = "default"):
        config_dir = os.path.dirname(os.path.abspath(__file__)) + "/bots"
        config_path = os.path.join(config_dir, f"{bot_name}.json")
        return config_path

    def get_bots(self) -> List[str]:
        """Return list of available bot configuration names."""
        try:
            files = os.listdir(self.config_dir)
            return sorted(
                [f.replace(".json", "") for f in files if f.endswith(".json")]
            )
        except Exception as e:
            print(f"[ConfigProvider] Error listing bots: {e}")
            return ["default"]

    def get_current_bot_name(self) -> str:
        """Return the currently loaded bot name."""
        return self._bot_name

    def _load_config(self, robot_name: Optional[str] = None):
        """Load configuration from JSON file."""
        try:
            with open(self.config_path, "r") as f:
                self._config = json.load(f)
            print(f"[ConfigProvider] Loaded config from: {self.config_path}")
        except FileNotFoundError:
            print(
                f"[ConfigProvider] WARNING: Config file not found: {self.config_path}"
            )
            print("[ConfigProvider] Using empty configuration with defaults")
            self._config = {}
        except json.JSONDecodeError as e:
            print(f"[ConfigProvider] ERROR: Invalid JSON in config file: {e}")
            print("[ConfigProvider] Using empty configuration with defaults")
            self._config = {}

    def _detect_environment(self):
        """Detect whether running on real robot or in simulation."""
        # Check environment variables first
        env_real = os.environ.get("DUCKIEBOT_REAL", "")
        env_sim = os.environ.get("DUCKIEBOT_SIM", "")

        if env_real == "1":
            self._is_real = True
            print("[ConfigProvider] Environment: REAL ROBOT (from env var)")
            return

        if env_sim == "1":
            self._is_real = False
            print("[ConfigProvider] Environment: SIMULATION (from env var)")
            return

        # Auto-detect by checking if godot module is available
        try:
            import godot

            self._is_real = False
            print("[ConfigProvider] Environment: SIMULATION (auto-detected)")
        except ImportError:
            self._is_real = True
            print("[ConfigProvider] Environment: REAL ROBOT (auto-detected)")

    @property
    def is_real(self) -> bool:
        """Return True if running on real robot, False if in simulation."""
        return self._is_real if self._is_real is not None else True

    @property
    def is_simulation(self) -> bool:
        """Return True if running in simulation, False if on real robot."""
        return not self.is_real

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value using dot notation.

        Args:
            key: Configuration key in dot notation (e.g., 'navigation.start_node')
            default: Default value if key is not found

        Returns:
            Configuration value or default

        Example:
            >>> config.get('navigation.start_node')
            1
            >>> config.get('lane_following.p_gain')
            0.1
        """
        keys = key.split(".")
        value = self._config

        for k in keys:
            if isinstance(value, dict):
                value = value.get(k)
                if value is None:
                    return default
            else:
                return default

        return value

    def get_navigation(self) -> Dict[str, Any]:
        """
        Get navigation configuration (start_node, start_direction, goal_node).

        Returns:
            Dictionary with navigation parameters

        Example:
            >>> config.get_navigation()
            {'start_node': 2, 'start_direction': 'N', 'goal_node': 3}
        """
        return self._config.get("navigation", {})

    def set_navigation(
        self, start_node: int = None, start_direction: str = None, goal_node: int = None
    ):
        """
        Update navigation parameters.

        Args:
            start_node: Starting intersection node ID
            start_direction: Starting direction ('N', 'E', 'S', 'W')
            goal_node: Goal intersection node ID

        Example:
            >>> config.set_navigation(start_node=2, start_direction='N', goal_node=3)
        """
        if start_node is not None:
            self.set("navigation.start_node", start_node)
        if start_direction is not None:
            self.set("navigation.start_direction", start_direction)
        if goal_node is not None:
            self.set("navigation.goal_node", goal_node)

    def get_hsv_range(self, color) -> Dict[str, int]:
        """
        Get HSV range for a specific color.

        Args:
            color: Color name ('yellow' or 'white')

        Returns:
            Dictionary with lower_h, upper_h, lower_s, upper_s, lower_v, upper_v

        Example:
            >>> config.get_hsv_range('yellow')
            {'lower_h': 20, 'upper_h': 35, 'lower_s': 80, ...}
        """
        hsv_cal = self._config.get("hsv_calibration", {})
        color_config = hsv_cal.get(color, {})

        # Return defaults if not found
        if not color_config:
            if color == "yellow":
                return {
                    "lower_h": 20,
                    "upper_h": 35,
                    "lower_s": 80,
                    "upper_s": 255,
                    "lower_v": 80,
                    "upper_v": 255,
                }
            elif color == "white":
                return {
                    "lower_h": 0,
                    "upper_h": 180,
                    "lower_s": 0,
                    "upper_s": 50,
                    "lower_v": 150,
                    "upper_v": 255,
                }

        return color_config

    def update_hsv_range(self, color: str, hsv_dict: Dict[str, int]):
        """
        Update HSV range for a specific color.

        Args:
            color: Color name ('yellow' or 'white')
            hsv_dict: Dictionary with HSV values to update

        Example:
            >>> config.update_hsv_range('yellow', {'lower_h': 25, 'upper_h': 40})
        """
        for key, value in hsv_dict.items():
            self.set(f"hsv_calibration.{color}.{key}", int(value))

    def get_hsv_calibration(self) -> Dict[str, Dict[str, int]]:
        """
        Get all HSV calibration parameters.

        Returns:
            Dictionary with 'yellow' and 'white' HSV ranges

        Example:
            >>> config.get_hsv_calibration()
            {'yellow': {...}, 'white': {...}}
        """
        return self._config.get("hsv_calibration", {})

    def get_lane_control(self) -> Dict[str, float]:
        """
        Get lane control parameters (PID and speed).

        Returns:
            Dictionary with p_gain, d_gain, base_speed

        Example:
            >>> config.get_lane_control()
            {'p_gain': 0.60, 'd_gain': 0.80, 'base_speed': 0.21}
        """
        defaults = {"p_gain": 0.60, "d_gain": 0.80, "base_speed": 0.21}
        return self._config.get("lane_control", defaults)

    def set_lane_control(
        self, p_gain: float = None, d_gain: float = None, base_speed: float = None
    ):
        """
        Update lane control parameters.

        Args:
            p_gain: Proportional gain for PID controller
            d_gain: Derivative gain for PID controller
            base_speed: Base driving speed

        Example:
            >>> config.set_lane_control(p_gain=0.65, d_gain=0.85)
        """
        if p_gain is not None:
            self.set("lane_control.p_gain", float(p_gain))
        if d_gain is not None:
            self.set("lane_control.d_gain", float(d_gain))
        if base_speed is not None:
            self.set("lane_control.base_speed", float(base_speed))

    def get_timing(self) -> Dict[str, float]:
        """
        Get intersection timing parameters.

        Returns:
            Dictionary with timing parameters (creep_time, exit_timeout, etc.)

        Example:
            >>> config.get_timing()
            {'creep_time': 0.80, 'exit_timeout': 4.0, ...}
        """
        defaults = {
            "creep_time": 0.80,
            "exit_timeout": 4.0,
            "forward_through": 1.0,
            "left_turn": 1.10,
            "right_turn": 0.80,
            "turnaround": 3.20,
        }
        return self._config.get("timing", defaults)

    def set_timing(self, **kwargs):
        """
        Update timing parameters.

        Args:
            creep_time: Time to creep forward at intersection
            exit_timeout: Timeout for exiting intersection
            forward_through: Time to drive straight through
            left_turn: Time for left turn
            right_turn: Time for right turn
            turnaround: Time for U-turn

        Example:
            >>> config.set_timing(left_turn=1.15, right_turn=0.85)
        """
        valid_keys = [
            "creep_time",
            "exit_timeout",
            "forward_through",
            "left_turn",
            "right_turn",
            "turnaround",
        ]
        for key, value in kwargs.items():
            if key in valid_keys:
                self.set(f"timing.{key}", float(value))

    def set(self, key: str, value: Any):
        """
        Set a configuration value using dot notation.

        Args:
            key: Configuration key in dot notation (e.g., 'navigation.start_node')
            value: Value to set

        Example:
            >>> config.set('navigation.start_node', 2)
            >>> config.set('lane_following.p_gain', 0.15)
        """
        keys = key.split(".")
        current = self._config

        # Navigate to the parent of the target key
        for k in keys[:-1]:
            if k not in current:
                current[k] = {}
            current = current[k]

        # Set the value
        current[keys[-1]] = value

    def update(self, updates: Dict[str, Any]):
        """
        Update multiple configuration values at once.

        Args:
            updates: Dictionary of key-value pairs to update

        Example:
            >>> config.update({
            ...     'navigation.start_node': 2,
            ...     'lane_following.p_gain': 0.15
            ... })
        """
        for key, value in updates.items():
            self.set(key, value)

    def save(self, bot_name: Optional[str] = None):
        """
        Save current configuration to JSON file.

        Args:
            bot_name: Name of bot config to save to. If None, saves to current bot.
        """
        if bot_name is None:
            save_path = self.config_path
        else:
            save_path = os.path.join(self.config_dir, f"{bot_name}.json")
            self._bot_name = bot_name
            self.config_path = save_path

        try:
            with open(save_path, "w") as f:
                json.dump(self._config, f, indent=2)
            print(
                f"[ConfigProvider] Configuration saved to: {save_path} (bot: {self._bot_name})"
            )
        except Exception as e:
            print(f"[ConfigProvider] ERROR: Failed to save config: {e}")

    def load(self, bot_name: str):
        """
        Load a different bot configuration.

        Args:
            bot_name: Name of the bot configuration to load
        """
        self._bot_name = bot_name
        self.config_path = os.path.join(self.config_dir, f"{bot_name}.json")
        self._load_config()
        print(f"[ConfigProvider] Loaded bot configuration: {bot_name}")

    def reload(self):
        """Reload configuration from current file."""
        self._load_config()
        self._detect_environment()

    def get_all(self) -> Dict[str, Any]:
        """
        Get the entire configuration dictionary.

        Returns:
            Complete configuration dictionary
        """
        return self._config.copy()

    def __repr__(self) -> str:
        env = "REAL" if self.is_real else "SIM"
        return f"<ConfigProvider(environment={env}, config_path={self.config_path})>"


# Global singleton instance
config = ConfigProvider()


# Convenience functions for backward compatibility
def get_config() -> ConfigProvider:
    """Get the global configuration provider instance."""
    return config


def reload_config():
    """Reload the global configuration from file."""
    config.load()


if __name__ == "__main__":
    # Test the configuration provider
    print("=" * 80)
    print("Configuration Provider Test")
    print("=" * 80)
    print()

    print(f"Config instance: {config}")
    print(f"Is real robot: {config.is_real}")
    print(f"Is simulation: {config.is_simulation}")
    print()

    print("Navigation settings:")
    nav_config = config.get_navigation()
    print(f"  Start node: {nav_config.get('start_node')}")
    print(f"  Goal node: {nav_config.get('goal_node')}")
    print(f"  Start direction: {nav_config.get('start_direction')}")
    print()

    print("HSV calibration:")
    yellow_hsv = config.get_hsv_range("yellow")
    print(
        f"  Yellow H range: [{yellow_hsv.get('lower_h')}, {yellow_hsv.get('upper_h')}]"
    )
    white_hsv = config.get_hsv_range("white")
    print(f"  White V range: [{white_hsv.get('lower_v')}, {white_hsv.get('upper_v')}]")
    print()

    # Test setting values
    print("Testing set operations:")
    print("  Setting navigation.goal_node = 5")
    config.set("navigation.goal_node", 5)
    print(f"  New goal node: {config.get('navigation.goal_node')}")
    print()

    print("  Setting HSV yellow lower_h = 25")
    config.set("hsv_calibration.yellow.lower_h", 25)
    yellow_hsv = config.get_hsv_range("yellow")
    print(f"  New yellow lower_h: {yellow_hsv.get('lower_h')}")
    print()

    # Test batch update
    print("Testing batch update with set_navigation():")
    config.set_navigation(start_node=1, goal_node=4)
    nav_config = config.get_navigation()
    print(f"  Start node: {nav_config.get('start_node')}")
    print(f"  Goal node: {nav_config.get('goal_node')}")
    print()

    print("=" * 80)
