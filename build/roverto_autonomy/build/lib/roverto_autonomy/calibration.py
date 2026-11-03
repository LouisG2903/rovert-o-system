"""Loader for the shared rover_calibration.yaml (single source of physical constants)."""
import os

from ament_index_python.packages import get_package_share_directory
import yaml


def default_calibration_path() -> str:
    """Return the installed path of roverto_description/config/rover_calibration.yaml."""
    return os.path.join(
        get_package_share_directory('roverto_description'), 'config', 'rover_calibration.yaml')


def load_calibration(path: str = '') -> dict:
    """Load the calibration YAML into a dict (defaults to the installed description copy)."""
    with open(path or default_calibration_path(), 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)
