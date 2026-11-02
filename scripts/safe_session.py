# Exemplo de utilizacao
# arm = safe_session('192.168.1.xxx')

from xarm.wrapper import XArmAPI

def safe_session(ip):

    # 1. Ligar ao robot
    arm = XArmAPI(ip, is_radian=False)

    # 2. Limpar avisos e erros
    arm.clean_warn()
    arm.clean_error()

    # 3. Ativar movimento
    arm.motion_enable(enable=True)

    # 4. Configurar modo e estado
    arm.set_mode(0)
    arm.set_state(0)

    # 5. Definir velocidade maxima das juntas
    code = arm.set_reduced_max_joint_speed(54)
    print("Limite velocidade juntas:", code)

    # 6. Definir velocidade maxima TCP
    code = arm.set_reduced_max_tcp_speed(150)
    print("Limite velocidade TCP:", code)

    # 7. Definir fronteira TCP da bancada
    # Substituir pelos valores reais aprovados
    x_max = 0
    x_min = 0
    y_max = 0
    y_min = 0
    z_max = 0
    z_min = 0

    code = arm.set_reduced_tcp_boundary([x_max, x_min, y_max, y_min, z_max, z_min])
    print("Fronteira TCP:", code)

    # 8. Ativar modo reduzido
    code = arm.set_reduced_mode(True)
    print("Modo reduzido:", code)

    # 9. Devolver objeto arm
    return arm

arm.disconnect()
