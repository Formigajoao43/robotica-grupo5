"""Verificações locais, sem rede, ROS, Docker ou hardware."""

from pathlib import Path

from fontTools import config
import yaml

from lite6.session import DEFAULT_CONFIG, RobotConfigurationError, load_robot_config, safe_session


def test_project_configuration_loads() -> None:
    config = load_robot_config()
    assert config["robot"]["model"] == "lite6"
    assert config["robot"]["ip"] == "127.0.0.1"
    assert config["safety"]["tcp_boundary_mm"] is None


def test_default_configuration_blocks_hardware_session() -> None:
    try:
        safe_session()
    except RobotConfigurationError as error:
        assert "safety.tcp_boundary_mm" in str(error)
    else:
        raise AssertionError("A configuração por omissão nunca pode ligar ao robô.")


def test_yaml_is_a_mapping() -> None:
    data = yaml.safe_load(Path(DEFAULT_CONFIG).read_text(encoding="utf-8"))
    assert isinstance(data, dict)
