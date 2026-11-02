
from pathlib import Path
import yaml
from xarm.wrapper import XArmAPI

# Ler o IP a partir do ficheiro de configuração
config_path = Path(__file__).resolve().parents[1] / "config" / "robot.yaml"

with open(config_path, "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

ip = config["robot"]["ip"]

# Este teste deve ligar apenas ao simulador local
if ip != "127.0.0.1":
    raise ValueError("O Teste 8 deve utilizar o simulador local.")

# Ligar ao firmware virtual
arm = XArmAPI(ip, is_radian=False, check_joint_limit=False)

try:
    # Ler os ângulos das juntas
    code_angles, angles = arm.get_servo_angle()
    print("Código de retorno (juntas):", code_angles)
    print("Ângulos das juntas:", angles)

    # Ler a posição e orientação da ferramenta
    code_pose, pose = arm.get_position()
    print("Código de retorno (pose):", code_pose)
    print("Pose do robô:", pose)

    # Cinemática direta para a configuração inicial (Pose A)
    angulos_A = [0, 0, 0, 0, 0, 0]

    codigo_fk, pose_fk = arm.get_forward_kinematics(
        angulos_A,
        input_is_radian=False,
        return_is_radian=False
    )

    print("Código de retorno FK:", codigo_fk)
    print("Pose A calculada pelo SDK:", pose_fk)


finally:
    arm.disconnect()
