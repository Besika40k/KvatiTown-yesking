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
from typing import Any, Dict, Optional


class ConfigProvider:
    """Centralized configuration provider with support for nested keys and environment detection."""

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the configuration provider.

        Args:
            config_path: Path to the configuration JSON file. If None, uses default location.
        """
        if config_path is None:
            # Default path relative to this file
            config_dir = os.path.dirname(os.path.abspath(__file__)) + "/bots"
            config_path = os.path.join(config_dir, "bot_default.json")

        self._bot_name = "default"
        self.config_path = config_path
        self._config: Dict[str, Any] = {}
        self._is_real: Optional[bool] = None
        self._load_config()
        self._detect_environment()

    def _load_config(self):
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

    def get_speed(self, speed_name: str, default: Any = None) -> float:
        """
        Get a speed parameter, automatically selecting simulation or real value.

        Args:
            speed_name: Name of the speed parameter (e.g., 'creep_speed', 'turn_speed')
            default: Default value if not found

        Returns:
            Speed value appropriate for current environment

        Example:
            >>> config.get_speed('creep_speed')
            0.06  # if in simulation
            0.3   # if on real robot
        """
        speeds = self._config.get("speeds", {})
        speed_config = speeds.get(speed_name, {})

        if isinstance(speed_config, dict):
            env_key = "real" if self.is_real else "simulation"
            return speed_config.get(env_key, default)

        # If it's a direct value (not environment-specific)
        return speed_config if speed_config is not None else default

    def get_timing(self, timing_name: str, default: Any = None) -> float:
        """
        Get a timing parameter, automatically selecting simulation or real value.

        Args:
            timing_name: Name of the timing parameter (e.g., 'turn_time_left')
            default: Default value if not found

        Returns:
            Timing value appropriate for current environment

        Example:
            >>> config.get_timing('turn_time_left')
            0.04  # if in simulation
            0.7   # if on real robot
        """
        timings = self._config.get("timing", {})
        timing_config = timings.get(timing_name, {})

        if isinstance(timing_config, dict):
            env_key = "real" if self.is_real else "simulation"
            return timing_config.get(env_key, default)

        return timing_config if timing_config is not None else default

    def get_hsv_range(self, color) -> dict[str, int]:
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

    def get_lane_following(self) -> Dict[str, float]:
        """
        Get all lane following parameters as a dictionary.

        Returns:
            Dictionary with all lane following parameters
        """
        return self._config.get("lane_following", {})

    def get_red_line_detection(self) -> Dict[str, Any]:
        """
        Get all red line detection parameters as a dictionary.

        Returns:
            Dictionary with all red line detection parameters
        """
        return self._config.get("red_line_detection", {})

    def get_object_detection(self) -> Dict[str, Any]:
        """
        Get all object detection parameters as a dictionary.

        Returns:
            Dictionary with all object detection parameters
        """
        return self._config.get("object_detection", {})

    def get_apriltag(self) -> Dict[str, Any]:
        """
        Get all AprilTag detection parameters as a dictionary.

        Returns:
            Dictionary with all AprilTag parameters
        """
        return self._config.get("apriltag", {})

    def get_road_map(self) -> Dict[str, Any]:
        """
        Get road map configuration (nodes and edges).

        Returns:
            Dictionary with 'nodes' and 'edges'
        """
        return self._config.get("road_map", {})

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

    def save(self, path: Optional[str] = None):
        """
        Save current configuration to JSON file.

        Args:
            path: Path to save to. If None, uses the original config path.
        """
        save_path = path or self.config_path

        try:
            with open(save_path, "w") as f:
                json.dump(self._config, f, indent=2)
            print(f"[ConfigProvider] Configuration saved to: {save_path}")
        except Exception as e:
            print(f"[ConfigProvider] ERROR: Failed to save config: {e}")

    def reload(self):
        """Reload configuration from file."""
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
    config.reload()


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
    print(f"  Start node: {config.get('navigation.start_node')}")
    print(f"  Goal node: {config.get('navigation.goal_node')}")
    print(f"  Start direction: {config.get('navigation.start_direction')}")
    print()

    print("Speed parameters:")
    print(f"  Creep speed: {config.get_speed('creep_speed')}")
    print(f"  Exit speed: {config.get_speed('exit_speed')}")
    print(f"  Turn speed: {config.get_speed('turn_speed')}")
    print()

    print("Timing parameters:")
    print(f"  Forward clear time: {config.get_timing('forward_clear_time')}")
    print(f"  Turn time left: {config.get_timing('turn_time_left')}")
    print(f"  Turn time right: {config.get_timing('turn_time_right')}")
    print()

    print("Lane following:")
    lane_config = config.get_lane_following()
    print(f"  P gain: {lane_config.get('p_gain')}")
    print(f"  D gain: {lane_config.get('d_gain')}")
    print(f"  Base speed: {lane_config.get('base_speed')}")
    print()

    print("HSV calibration:")
    yellow_hsv = config.get_hsv_range("yellow")
    print(
        f"  Yellow H range: [{yellow_hsv.get('lower_h')}, {yellow_hsv.get('upper_h')}]"
    )
    white_hsv = config.get_hsv_range("white")
    print(f"  White V range: [{white_hsv.get('lower_v')}, {white_hsv.get('upper_v')}]")
    print()

    print("Object detection:")
    obj_det = config.get_object_detection()
    print(f"  Obstacle classes: {obj_det.get('obstacle_classes')}")
    print(f"  Min area: {obj_det.get('obstacle_min_area')}")
    print()

    print("Red line detection:")
    red_det = config.get_red_line_detection()
    print(f"  Window size: {red_det.get('window_size')}")
    print(f"  Vote threshold: {red_det.get('vote_threshold')}")
    print()

    print("=" * 80)
