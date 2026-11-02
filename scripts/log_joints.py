
import time

from xarm.wrapper import XArmAPI
from lite6.session import load_robot_config


def main():
    config = load_robot_config()
    ip = config["robot"]["ip"]

    # Durante a validação, permitir apenas o simulador.
    if ip != "127.0.0.1":
        raise RuntimeError(
            "Este teste só pode comunicar com o simulador local."
        )

    duracao = 30

    arm = XArmAPI(
        ip,
        is_radian=False,
        check_joint_limit=False
    )

    print("A iniciar registo dos ângulos durante 30 segundos...")
    print("Tempo(s) | J1 | J2 | J3 | J4 | J5 | J6")

    inicio = time.perf_counter()
    numero_leituras = 0
    primeira_leitura = None
    ultima_leitura = None

    try:
        while time.perf_counter() - inicio < duracao:

            code, angulos = arm.get_servo_angle()

            if code != 0:
                print("Erro ao ler os ângulos. Código:", code)
                break

            tempo = time.perf_counter() - inicio

            print(
                f"{tempo:7.3f} | "
                + " | ".join(f"{a:8.2f}" for a in angulos[:6])
            )

            if primeira_leitura is None:
                primeira_leitura = tempo

            ultima_leitura = tempo
            numero_leituras += 1

    finally:
        arm.disconnect()

    if (
        numero_leituras > 1
        and ultima_leitura > primeira_leitura
    ):
        frequencia = (
            (numero_leituras - 1)
            / (ultima_leitura - primeira_leitura)
        )

        print("\nRegisto terminado.")
        print("Número de leituras:", numero_leituras)
        print(f"Frequência média: {frequencia:.2f} Hz")

    else:
        print("\nNão foram obtidas leituras suficientes.")


if __name__ == "__main__":
    main()
