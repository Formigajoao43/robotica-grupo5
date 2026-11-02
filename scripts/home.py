
from math import isfinite

from lite6.session import (
    DEFAULT_CONFIG,
    RobotConfigurationError,
    load_robot_config,
    safe_session,
)


def main(config_path=DEFAULT_CONFIG):
    config = load_robot_config(config_path)

    # Durante os testes, permitir apenas o simulador.
    if config["robot"]["ip"] != "127.0.0.1":
        raise RobotConfigurationError(
            "Este script apenas pode ser utilizado no simulador."
        )

    home = config["home"]["joints_deg"]
    velocidade = config["home"]["speed_deg_s"]

    # Validar a configuração antes de estabelecer ligação.
    if (
        not isinstance(home, list)
        or len(home) != 6
        or any(
            isinstance(a, bool)
            or not isinstance(a, (int, float))
            or not isfinite(a)
            for a in home
        )
    ):
        raise RobotConfigurationError(
            "É necessário configurar os seis ângulos home."
        )

    if (
        isinstance(velocidade, bool)
        or not isinstance(velocidade, (int, float))
        or not isfinite(velocidade)
        or not 0 < velocidade <= config["safety"]["reduced_max_joint_speed_deg_s"]
    ):
        raise RobotConfigurationError(
            "A velocidade home é inválida."
        )

    # A sessão segura valida os restantes parâmetros.
    arm = safe_session(config_path)

    try:
        code = arm.set_servo_angle(
            angle=home,
            speed=velocidade,
            wait=True
        )

        if code != 0:
            raise RuntimeError(
                f"Movimento home falhou: código {code}"
            )

        code, angulos = arm.get_servo_angle()

        if code != 0:
            raise RuntimeError(
                f"Leitura das juntas falhou: código {code}"
            )

        print("Home definida:", home)
        print("Ângulos finais:", angulos[:6])

    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
