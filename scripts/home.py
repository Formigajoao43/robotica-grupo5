from xarm.wrapper import XArmAPI

# Ligar ao robot
arm = XArmAPI('192.168.1.xxx', is_radian=False)

# Configuracao home definida pela equipa
home = [0, 0, 0, 0, 0, 0]

# ATENCAO:
# Substituir os valores de exemplo pelos
# angulos home aprovados pelo professor.

# Limpar avisos e erros
arm.clean_warn()
arm.clean_error()

# Ativar movimento
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)

# Comandar movimento para home
code = arm.set_servo_angle(
    angle=home,
    speed=20,
    wait=True
)

print("Codigo do movimento:", code)

# Ler os angulos finais
code, angulos = arm.get_servo_angle()
print("Angulos finais:", angulos)

arm.disconnect()

