"""Labwork 3 - velocity control of the UFACTORY Lite 6 along a circle, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 3.

Implement the following, replacing every ``raise NotImplementedError``:

    Part A - the task
        circle_reference(t, c, r, alpha_dot)      desired pose and velocity at time t

    Part B - the control
        jacobian(q)                               the 6x6 Jacobian of {6} in {0}
        open_loop_step(q, V_d)                    joint velocities, open loop
        closed_loop_step(q, T_d, V_d, kp)         joint velocities, closed loop
        VelocityNode                              the node that moves the arm

    The constants KP, DT, QDOT_MAX and SIGMA_MIN below are chosen by the group
    (Part B and analysis E.2).

Run the node with ``ros2 run lite6_velocity vel_node`` (in Gazebo add
``--ros-args -p use_sim_time:=true``).

``pose_error``, ``rotation_vector``, ``load_circle``, ``JOINT_NAMES``,
``TRAJECTORY_TOPIC`` and ``main`` below are PROVIDED: already implemented, do
not edit them.

Keep the function and class names, arguments and return types exactly as they
are: the instructor's grader uses them by name.

Conventions - those of Labworks 1 and 2:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``.
    * Joint angles in RADIANS, in the controller's convention, as published on
      /joint_states; joint velocities in rad/s.
    * Lengths in METRES, linear velocities in m/s, angular velocities in rad/s,
      time in seconds.

Allowed imports: ``numpy`` and ``math`` for the maths; ``forward_kinematics``
from your own Labwork 1 and ``inverse_kinematics`` and ``select_solution``
from your own Labwork 2 (already imported below); and, for the node,
``rclpy``, ``sensor_msgs``, ``geometry_msgs``, ``std_msgs``,
``trajectory_msgs`` and ``builtin_interfaces`` (already imported below).
Nothing else - no MoveIt, no kinematics library, no URDF parser, no tf.
"""

import math
import os

import numpy as np

from lite6_kinematics.forward_kinematics import forward_kinematics
from lite6_ik.inverse_kinematics import inverse_kinematics, select_solution

try:
    import rclpy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError:         # the maths also runs without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'
TRAJECTORY_TOPIC = '/lite6_traj_controller/joint_trajectory'

# --- chosen by the group (Part B, analysis E.2): replace each None ------------
KP = 2.0             # 1/s   gain of the closed loop
DT = 0.05          # s     control period
QDOT_MAX = 1.0     # rad/s largest joint velocity the node may command
SIGMA_MIN = 0.05      # smallest singular value of J below which the node stops


# =============================================================================
# PROVIDED - do not edit
# =============================================================================
def rotation_vector(R) -> np.ndarray:
    """Rotation vector phi(R) = theta * u of a 3x3 rotation matrix.

    u is the unit axis of the rotation and theta in [0, pi] its angle, so the
    result has norm theta; the zero vector for the identity. Expressed in the
    frame in which R is expressed.
    """
    R = np.asarray(R, dtype=float)
    v = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(v)                   # sin(theta)
    c = 0.5 * (np.trace(R) - 1.0)           # cos(theta)
    theta = math.atan2(s, c)
    if s > 1e-6 or (s > 0.0 and c > 0.0):
        return (theta / s) * v
    if c > 0.0:                             # R is the identity
        return v
    M = 0.5 * (R + np.eye(3))               # theta = pi: R = 2 u u^T - I
    i = int(np.argmax(np.diag(M)))
    u = M[:, i] / math.sqrt(M[i, i])
    return math.pi * u


def pose_error(T_c, T_d) -> np.ndarray:
    """Error between a current pose T_c and a desired pose T_d, both (4, 4) in {0}.

    Returns the (6,) array [p_d - p_c ; phi(R_d R_c^T)]: the vector from the
    current to the desired position (m), then the rotation vector (rad) of the
    rotation that takes the current orientation to the desired one, expressed
    in {0}.
    """
    T_c = np.asarray(T_c, dtype=float)
    T_d = np.asarray(T_d, dtype=float)
    e = np.zeros(6)
    e[:3] = T_d[:3, 3] - T_c[:3, 3]
    e[3:] = rotation_vector(T_d[:3, :3] @ T_c[:3, :3].T)
    return e


def load_circle(name):
    """(c, r) of circle ``name`` in config/lw3_circles.yaml: centre (3,) in m, radius in m."""
    import yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('lite6_velocity'), 'config', 'lw3_circles.yaml')
    except Exception:       # outside ROS: the source tree
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'config', 'lw3_circles.yaml')
    with open(path) as f:
        circles = yaml.safe_load(f)['circles']
    if name not in circles:
        raise KeyError(f'circle {name!r} is not in {path}: {sorted(circles)}')
    return np.asarray(circles[name]['center'], dtype=float), float(circles[name]['radius'])


# =============================================================================
# Part A - the task
# =============================================================================
def circle_reference(t, c, r, alpha_dot):
    """Desired pose and velocity of {6} at time t on the circle of Part A.

    Args:
        t:         time since the start of the circle (s).
        c:         (3,) centre of the circle in {0} (m).
        r:         radius (m).
        alpha_dot: angular rate of the point on the circle (rad/s).

    Returns:
        (T_d, V_d): T_d the (4, 4) desired pose of {6} in {0}; V_d the (6,)
        desired velocity [v; w] in {0}: v (m/s) the linear velocity of the
        origin of {6}, w (rad/s) its angular velocity.
    """
    # TODO (LW3, Part A)
    #raise NotImplementedError('circle_reference() is not implemented yet')

    c = np.asarray(c, dtype=float).reshape(3)

    alpha = alpha_dot * t

    T_d = np.eye(4, dtype=float)

    # Orientacao constante: roll = pi, pitch = 0, yaw = 0
    T_d[:3, :3] = np.diag([1.0, -1.0, -1.0])

    # Posicao desejada
    T_d[:3, 3] = c + r * np.array([
        math.cos(alpha),
        math.sin(alpha),
        0.0
    ])

    # Velocidade cartesiana desejada [v; omega]
    V_d = np.array([
        -r * alpha_dot * math.sin(alpha),
         r * alpha_dot * math.cos(alpha),
         0.0,
         0.0,
         0.0,
         0.0
    ], dtype=float)

    return T_d, V_d

# =============================================================================
# Part B - the control
# =============================================================================
def jacobian(q) -> np.ndarray:
    """The Jacobian J(q) of {6}, expressed in {0}.

    Args:
        q: six joint angles (rad), order joint1..joint6.

    Returns:
        (6, 6) array J such that, for the joint velocities dq (rad/s),
        [v; w] = J(q) @ dq, where v (rows 0-2) is the linear velocity (m/s) of
        the origin of {6} and w (rows 3-5) its angular velocity (rad/s), both
        expressed in {0}. How J is obtained is the group's choice.
    """
    # TODO (LW3, Part B)
    #raise NotImplementedError('jacobian() is not implemented yet')

    q = np.asarray(q, dtype=float).reshape(6)

    # Transformacoes homogeneas do LW1
    frames = forward_kinematics(q, all_frames=True)

    # Posicao do efetuador final
    p_e = frames[6][:3, 3]

    J = np.zeros((6, 6), dtype=float)

    for i in range(6):

        # Eixo da junta i+1 na convencao MDH do LW1
        T = frames[i + 1]

        z_i = T[:3, 2]
        p_i = T[:3, 3]

        # Parte linear do Jacobiano
        J[:3, i] = np.cross(z_i, p_e - p_i)

        # Parte angular do Jacobiano
        J[3:, i] = z_i

    return J



def open_loop_step(q, V_d) -> np.ndarray:
    """Joint velocities of the open-loop law of Part B.

    Args:
        q:   six joint angles (rad), the current configuration.
        V_d: (6,) desired velocity [v; w] in {0} (m/s, rad/s).

    Returns:
        (6,) joint velocities (rad/s).
    """
    # TODO (LW3, Part B)
    #raise NotImplementedError('open_loop_step() is not implemented yet')

    q = np.asarray(q, dtype=float).reshape(6)
    V_d = np.asarray(V_d, dtype=float).reshape(6)

    J = jacobian(q)

    dq = np.linalg.solve(J, V_d)

    return dq



def closed_loop_step(q, T_d, V_d, kp) -> np.ndarray:
    """Joint velocities of the closed-loop law of Part B.

    Args:
        q:   six joint angles (rad), the current configuration.
        T_d: (4, 4) desired pose of {6} in {0}.
        V_d: (6,) desired velocity [v; w] in {0} (m/s, rad/s).
        kp:  gain (1/s).

    Returns:
        (6,) joint velocities (rad/s). The error is pose_error(), which is
        provided.
    """
    # TODO (LW3, Part B)
    #raise NotImplementedError('closed_loop_step() is not implemented yet')

    q = np.asarray(q, dtype=float).reshape(6)
    T_d = np.asarray(T_d, dtype=float).reshape(4, 4)
    V_d = np.asarray(V_d, dtype=float).reshape(6)

    # Pose atual calculada pelo LW1
    T_c = forward_kinematics(q)

    # Erro de pose calculado pela funcao fornecida
    e = pose_error(T_c, T_d)

    # Jacobiano geometrico
    J = jacobian(q)

    # Velocidade cartesiana com correcao proporcional
    V = V_d + kp * e

    dq = np.linalg.solve(J, V)

    return dq



# =============================================================================
# Part B - the node
# =============================================================================
class VelocityNode(Node):
    """Moves the end-effector along a circle by velocity control.

    ROS parameters (name, default):
        circle      'C1'        a circle of config/lw3_circles.yaml (load_circle())
        center      [0, 0, 0]   centre (m) of a circle of the group's own; used,
        radius      0.0         with radius (m), when radius > 0
        alpha_dot   pi/2        rad/s
        law         'closed'    'open' or 'closed'
        kp          KP          1/s
        dt          DT          s, the control period
        laps        3
        qdot_max    QDOT_MAX    rad/s
        sigma_min   SIGMA_MIN
        start_time  5.0         s, duration of the motion to the start of the circle

    Subscribes   /joint_states         sensor_msgs/JointState (joints picked by name)
    Publishes    TRAJECTORY_TOPIC      trajectory_msgs/JointTrajectory
                 /lite6_vel/reference  geometry_msgs/PoseStamped, frame BASE_FRAME
                 /lite6_vel/status     std_msgs/String, durability TRANSIENT_LOCAL

    Behaviour, with every time taken from the node's clock:
        1. When the first joint angles are received, it selects, with the
           group's Labwork 2 functions, the solution for the start of the
           circle (t = 0) nearest to the current angles, sends it as a
           trajectory of one point reached in start_time, and publishes the
           status 'moving_to_start'. With no executable solution it publishes
           'unreachable' and does nothing more.
        2. Once start_time has elapsed and every joint is within 1e-2 rad of
           that solution, it publishes 'running'; t = 0 is that instant.
        3. Every dt seconds, with q the latest angles received on
           /joint_states: it publishes T_d(t) on /lite6_vel/reference;
           if the smallest singular value of jacobian(q) is below sigma_min it
           publishes 'singular' and stops; it computes the joint velocities
           dq with the law chosen; if any |dq_i| exceeds qdot_max it publishes
           'too_fast' and stops; otherwise it sends q + dt * dq as a trajectory
           of one point reached in dt.
        4. After laps full turns of the circle it publishes 'done' and stops.
    Stopping means sending nothing more: the controller holds the last target.
    """

    def __init__(self):
        super().__init__('vel_node')
        # TODO (LW3, Part B)
        #raise NotImplementedError('VelocityNode is not implemented yet')
         # Parametros ROS 2
        self.declare_parameter('circle', 'C1')
        self.declare_parameter('center', [0.0, 0.0, 0.0])
        self.declare_parameter('radius', 0.0)
        self.declare_parameter('alpha_dot', math.pi / 2.0)
        self.declare_parameter('law', 'closed')
        self.declare_parameter('kp', KP)
        self.declare_parameter('dt', DT)
        self.declare_parameter('laps', 3)
        self.declare_parameter('qdot_max', QDOT_MAX)
        self.declare_parameter('sigma_min', SIGMA_MIN)
        self.declare_parameter('start_time', 5.0)

        self.law = self.get_parameter('law').value
        self.kp = float(self.get_parameter('kp').value)
        self.dt = float(self.get_parameter('dt').value)
        self.laps = int(self.get_parameter('laps').value)
        self.alpha_dot = float(
            self.get_parameter('alpha_dot').value
        )
        self.qdot_max = float(
            self.get_parameter('qdot_max').value
        )
        self.sigma_min = float(
            self.get_parameter('sigma_min').value
        )
        self.start_time = float(
            self.get_parameter('start_time').value
        )

        if self.law not in ('open', 'closed', 'ik'):
            raise ValueError('law deve ser open, closed ou ik')

        if (self.dt <= 0.0 or self.alpha_dot <= 0.0
                or self.laps <= 0 or self.start_time <= 0.0
                or self.qdot_max <= 0.0 or self.sigma_min <= 0.0
                or self.kp < 0.0):
            raise ValueError('Parametros de controlo invalidos')

        # Selecionar a circunferencia
        radius = float(self.get_parameter('radius').value)

        if radius > 0.0:
            self.c = np.asarray(
                self.get_parameter('center').value,
                dtype=float
            ).reshape(3)
            self.r = radius
        else:
            name = self.get_parameter('circle').value
            self.c, self.r = load_circle(name)

        # Estado interno
        self.q = None
        self.q_start = None
        self.state = 'waiting'
        self.move_start_ns = None
        self.run_start_ns = None
        self.controller_ready_ns = None
        self.start_timeout = max(10.0, 2.0 * self.start_time)

        # Publicadores
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, TRAJECTORY_TOPIC, 10
        )

        self.reference_pub = self.create_publisher(
            PoseStamped, '/lite6_vel/reference', 10
        )

        qos_status = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.status_pub = self.create_publisher(
            String, '/lite6_vel/status', qos_status
        )

        # Subscricao dos angulos articulares
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.on_joint_state,
            10
        )

        # Ciclo de controlo
        self.timer = self.create_timer(
            self.dt, self.on_timer
        )

        self.get_logger().info('VelocityNode iniciado')

    def now_ns(self):
        """Tempo atual do relogio ROS, em nanossegundos."""
        return self.get_clock().now().nanoseconds

    def set_status(self, status):
        """Publica e guarda o estado atual."""
        self.state = status

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

        self.get_logger().info(f'Estado: {status}')

    def send_target(self, q_target, duration):
        """Envia uma trajetoria articular com um unico ponto."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [
            float(value) for value in q_target
        ]

        seconds = int(duration)
        nanoseconds = int(round(
            (duration - seconds) * 1e9
        ))

        if nanoseconds >= 1000000000:
            seconds += 1
            nanoseconds -= 1000000000

        point.time_from_start = Duration(
            sec=seconds,
            nanosec=nanoseconds
        )

        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def publish_reference(self, T_d):
        """Publica a pose cartesiana desejada."""
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME

        msg.pose.position.x = float(T_d[0, 3])
        msg.pose.position.y = float(T_d[1, 3])
        msg.pose.position.z = float(T_d[2, 3])

        # Rotacao fixa Rx(pi): quaternion (x,y,z,w)
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0

        self.reference_pub.publish(msg)

    def on_joint_state(self, msg):
        """Atualiza os angulos, identificando as juntas pelo nome."""
        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in JOINT_NAMES):
            return

        self.q = np.array([
            positions[name] for name in JOINT_NAMES
        ], dtype=float)

        if not np.all(np.isfinite(self.q)):
            self.q = None
            return

        # A primeira leitura inicia o movimento para o ponto inicial
        if self.state != 'waiting':
            return

        T_start, _ = circle_reference(
            0.0, self.c, self.r, self.alpha_dot
        )

        solutions = inverse_kinematics(T_start)
        selected = select_solution(solutions, self.q)

        if selected is None:
            self.set_status('unreachable')
            return

        self.q_start = np.asarray(
            selected, dtype=float
        ).reshape(6)

        # Diagnostico: identificar a configuracao inicial escolhida pela IK.
        self.get_logger().info(
            'q atual (rad) = %s; q inicial (rad) = %s'
            % (np.array2string(self.q, precision=4),
               np.array2string(self.q_start, precision=4))
        )

        # Aguardar pelo controlador antes de publicar o primeiro alvo.
        # O envio e efetuado pelo temporizador, nao pelo callback inicial.
        self.set_status('waiting_for_controller')

    def on_timer(self):
        """Executa o controlo com o periodo definido por dt."""
        if self.q is None:
            return

        if self.state == 'waiting_for_controller':
            if self.trajectory_pub.get_subscription_count() == 0:
                self.controller_ready_ns = None
                return

            # Exigir uma ligacao ROS estavel durante 1 segundo.
            now = self.now_ns()
            if self.controller_ready_ns is None:
                self.controller_ready_ns = now
                return
            if (now - self.controller_ready_ns) < 1_000_000_000:
                return

            # Verificar tambem o movimento inicial antes de enviar o alvo.
            # Esta e uma verificacao conservadora da velocidade media; o
            # limite fisico do modo reduzido tem de ser confirmado a parte.
            initial_speed = np.abs(self.q_start - self.q) / self.start_time
            if (not np.all(np.isfinite(initial_speed))
                    or np.any(initial_speed > self.qdot_max)):
                self.get_logger().error(
                    'Movimento inicial excederia qdot_max: '
                    'velocidades medias (rad/s) = %s; limite = %.4f'
                    % (np.array2string(initial_speed, precision=5),
                       self.qdot_max)
                )
                self.set_status('too_fast')
                return

            self.move_start_ns = now
            self.send_target(self.q_start, self.start_time)
            self.set_status('moving_to_start')
            return

        if self.state == 'moving_to_start':
            elapsed = (
                self.now_ns() - self.move_start_ns
            ) * 1e-9
            error_q = np.abs(self.q - self.q_start)

            if (elapsed >= self.start_time
                    and np.all(error_q < 1e-2)):
                self.run_start_ns = self.now_ns()
                self.set_status('running')
            elif elapsed > self.start_timeout:
                # Nunca reenviar automaticamente uma trajetoria ao robo.
                self.get_logger().error(
                    'Inicio falhou: desvio maximo = %.4f rad; '
                    'verificar estado Ready (0), modo e controlador'
                    % float(np.max(error_q))
                )
                self.set_status('start_failed')
                return
            else:
                return

        if self.state != 'running':
            return

        t = (
            self.now_ns() - self.run_start_ns
        ) * 1e-9

        total_time = (
            self.laps * 2.0 * math.pi / self.alpha_dot
        )

        if t >= total_time:
            self.set_status('done')
            return

        # Referencia cartesiana
        T_d, V_d = circle_reference(
            t, self.c, self.r, self.alpha_dot
        )

        self.publish_reference(T_d)

        # Verificar proximidade de singularidades
        J = jacobian(self.q)

        sigma = np.linalg.svd(
            J, compute_uv=False
        )

        sigma_smallest = float(np.min(sigma))

        if (not np.isfinite(sigma_smallest)
                or sigma_smallest < self.sigma_min):
            self.get_logger().error(
                'singular: sigma_min(J)=%.6f; limite=%.6f; t=%.3fs'
                % (sigma_smallest, self.sigma_min, t)
            )
            self.set_status('singular')
            return

        # Calcular velocidades articulares
        
        try:
            if self.law == 'ik':
                # Solucoes IK da pose desejada
                solutions = inverse_kinematics(T_d)

                # Escolher a solucao mais proxima da atual
                selected = select_solution(solutions, self.q)

                if selected is None:
                    self.set_status('unreachable')
                    return

                # Alvo articular obtido diretamente pela IK
                q_target = np.asarray(
                    selected, dtype=float
                ).reshape(6)

                # Velocidade necessaria para atingir o alvo
                dq = (q_target - self.q) / self.dt

            elif self.law == 'open':
                dq = open_loop_step(self.q, V_d)

            else:
                dq = closed_loop_step(
                    self.q, T_d, V_d, self.kp
                )

        except np.linalg.LinAlgError:
            self.set_status('singular')
            return

        # Verificar velocidades maximas
        if (not np.all(np.isfinite(dq))
                or np.any(np.abs(dq) > self.qdot_max)):
            # Diagnostico sem alterar os limites nem enviar comandos.
            if np.all(np.isfinite(dq)):
                joint_idx = int(np.argmax(np.abs(dq)))
                self.get_logger().error(
                    'too_fast: t=%.3fs, junta=%s, |dq|=%.4f rad/s, '
                    'limite=%.4f rad/s, sigma_min(J)=%.6f'
                    % (t, JOINT_NAMES[joint_idx],
                       float(np.abs(dq[joint_idx])), self.qdot_max,
                       sigma_smallest)
                )
            else:
                self.get_logger().error(
                    'too_fast: velocidades de junta nao finitas; '
                    't=%.3fs, sigma_min(J)=%.6f' % (t, sigma_smallest)
                )
            self.get_logger().error(
                'dq (rad/s) = %s; q (rad) = %s'
                % (np.array2string(dq, precision=5),
                   np.array2string(self.q, precision=5))
            )
            self.set_status('too_fast')
            return

        # Integracao apenas nos modos open e closed
        if self.law != 'ik':
            q_target = self.q + self.dt * dq

        # Enviar alvo articular ao controlador
        self.send_target(q_target, self.dt)


def main(args=None):
    """PROVIDED. Entry point of ``ros2 run lite6_velocity vel_node``."""
    if rclpy is None:
        raise RuntimeError('ROS 2 (rclpy) is not available')
    rclpy.init(args=args)
    node = VelocityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
