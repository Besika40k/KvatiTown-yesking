"""
Configuration Package for Duckiebot Navigation System

This package provides centralized configuration management for the navigation system.

Quick Start:
    from config.config_provider import config

    # Get configuration values
    start_node = config.get('navigation.start_node')
    creep_speed = config.get_speed('creep_speed')

See README.md for full documentation.
"""

from .config_provider import ConfigProvider, config, get_config, reload_config

__all__ = [
    "ConfigProvider",
    "config",
    "get_config",
    "reload_config",
]

__version__ = "1.0.0"
