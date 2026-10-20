"""Sessão segura para o Lite 6 definida pelo LW0.1, §7.

O módulo não estabelece ligação ao ser importado. ``safe_session`` só pode ser
usado depois de a configuração da bancada ter sido medida e aprovada.
"""

from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "robot.yaml"


class RobotConfigurationError(RuntimeError):
    """A configuração necessária para uma sessão segura está incompleta."""


def load_robot_config(path: str | Path = DEFAULT_CONFIG) -> dict[str, Any]:
    """Carrega a configuração YAML sem estabelecer ligação ao robô."""
    with Path(path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise RobotConfigurationError("robot.yaml tem de conter um mapeamento YAML.")
    return config


def _require(value: Any, name: str) -> Any:
    if value is None or value == "":
        raise RobotConfigurationError(
            f"{name} não está configurado em config/robot.yaml; "
            "não é seguro abrir uma sessão de hardware."
        )
    return value


def _check_code(code: int, operation: str) -> None:
    if code != 0:
        raise RuntimeError(f"{operation} falhou (código SDK {code}).")


def safe_session(config_path: str | Path = DEFAULT_CONFIG):
    """Liga e prepara o robô conforme LW0.1 §4.1 e §7.

    Exige IP e fronteira TCP reais antes de abrir a ligação. A sequência é
    limpar avisos/erros, ativar movimento, modo 0, estado 0 e modo reduzido.
    Não comanda qualquer trajetória ou pose.
    """
    config = load_robot_config(config_path)
    robot = config.get("robot", {})
    safety = config.get("safety", {})
    ip = _require(robot.get("ip"), "robot.ip")
    boundary = _require(safety.get("tcp_boundary_mm"), "safety.tcp_boundary_mm")
    if not isinstance(boundary, list) or len(boundary) != 6:
        raise RobotConfigurationError(
            "safety.tcp_boundary_mm tem de conter seis limites na ordem da API SDK."
        )

    # Importação tardia: validar o YAML não exige o SDK nem rede.
    from xarm.wrapper import XArmAPI

    arm = XArmAPI(ip, is_radian=False)
    try:
        _check_code(arm.clean_warn(), "clean_warn")
        _check_code(arm.clean_error(), "clean_error")
        _check_code(arm.motion_enable(enable=True), "motion_enable")
        _check_code(arm.set_mode(0), "set_mode(0)")
        _check_code(arm.set_state(0), "set_state(0)")
        _check_code(
            arm.set_reduced_max_joint_speed(
                safety["reduced_max_joint_speed_deg_s"], is_radian=False
            ),
            "set_reduced_max_joint_speed",
        )
        _check_code(
            arm.set_reduced_max_tcp_speed(safety["reduced_max_tcp_speed_mm_s"]),
            "set_reduced_max_tcp_speed",
        )
        _check_code(arm.set_reduced_tcp_boundary(boundary), "set_reduced_tcp_boundary")
        _check_code(arm.set_reduced_mode(True), "set_reduced_mode(True)")
    except Exception:
        arm.disconnect()
        raise
    return arm
