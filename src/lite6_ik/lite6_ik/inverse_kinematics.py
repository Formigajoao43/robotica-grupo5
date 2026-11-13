"""Labwork 2 - algebraic inverse kinematics of the UFACTORY Lite 6, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 2.

Implement the following, replacing every ``raise NotImplementedError``:

    Part B - the maths (no ROS needed)
        inverse_kinematics(T_d)                    every distinct solution for a pose
        select_solution(solutions, q_current)      the solution that is executed

    Part B - the node
        IkNode                                     reads a desired pose, publishes
                                                   the selected joint angles

Run the node with ``ros2 run lite6_ik ik_node``.

``transform_from_pose``, ``JOINT_LIMITS``, the constants ``TOL_SAME`` and
``TOL_WRIST`` and ``main`` below are PROVIDED: already implemented, do not edit
them.

Keep the function and class names, arguments and return types exactly as they
are: the checker node (``ik_check``), the study program (``ik_study``), the
tests in ``test/`` and the instructor's grader all use them by name.

Conventions - those of Labwork 1:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``. The desired pose T_d is the pose of {6} in {0}.
    * Joint angles in RADIANS, in the controller's convention, as published on
      /joint_states (q = 0 is the controller's zero configuration).
    * Lengths in METRES.

Allowed imports: ``numpy`` and ``math`` for the maths, ``forward_kinematics``
from your own ``lite6_kinematics.forward_kinematics`` (already imported below),
and ``rclpy``, ``sensor_msgs`` and ``geometry_msgs`` for the node (already
imported below). Nothing else - no MoveIt, no kinematics library, no URDF
parser, no tf. ``test/test_inverse_kinematics.py`` checks the imports.
"""

import math

import numpy as np

from lite6_kinematics.forward_kinematics import forward_kinematics

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:         # the maths and its unit tests also run without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'


# =============================================================================
# Part B - the maths
# =============================================================================
def inverse_kinematics(T_d) -> list:
    """Every distinct joint vector that puts the end-effector at T_d.

    The solution is closed-form, from the expressions of Part A: no iterative
    or optimisation method.

    Args:
        T_d: (4, 4) desired pose of {6} in {0}; translation in metres.

    Returns:
        A list of (6,) float arrays, joint angles in radians, order
        joint1..joint6, one array per distinct solution (in any order). The
        empty list if T_d is out of the robot's reach.

    Conventions of Part A:
        * every angle is in (-pi, pi]; the joint ranges JOINT_LIMITS are NOT
          applied here: a solution outside them is still returned;
        * two solutions in which no angle differs by more than TOL_SAME
          (angles compared modulo 2 pi) are the same solution: only one of
          them is returned;
        * if the angle between the axes of joints 4 and 6 is smaller than
          TOL_WRIST, those axes are aligned and every solution returned has
          q4 = 0.
    """
    # TODO (LW2, Part B)
    #raise NotImplementedError('inverse_kinematics() is not implemented yet')

    T_d = np.asarray(T_d, dtype=float)

    d1 = 0.2433
    a2 = 0.2000
    a3 = 0.0870
    d4 = 0.2276
    d6 = 0.0615

    L = math.hypot(a3, d4)
    beta = math.atan2(a3, d4)

    def normalize(angle):
        value = (angle + math.pi) % (2.0 * math.pi) - math.pi
        if value <= -math.pi + 1e-12:
            return math.pi
        return value

    def same_solution(q_a, q_b):
        diff = (
            np.asarray(q_a) - np.asarray(q_b) + math.pi
        ) % (2.0 * math.pi) - math.pi

        return np.all(np.abs(diff) <= TOL_SAME)

    R_d = T_d[:3, :3]
    p_d = T_d[:3, 3]

    # Centro do punho
    p_w = p_d - d6 * R_d[:, 2]

    x_w, y_w, z_w = p_w
    h = z_w - d1

    solutions = []

    # Dois ramos da primeira junta
    q1_base = math.atan2(y_w, x_w)

    for q1 in (q1_base, q1_base + math.pi):

        c1 = math.cos(q1)
        s1 = math.sin(q1)

        rho = x_w * c1 + y_w * s1

        # Lei dos cossenos para a terceira junta
        C = (
            a2 * a2 + L * L - rho * rho - h * h
        ) / (2.0 * a2 * L)

        # Tolerancia na fronteira do alcance
        if C < -1.0 - 1e-10 or C > 1.0 + 1e-10:
            continue

        C = max(-1.0, min(1.0, C))

        elbow_angle = math.acos(C)

        # Dois ramos do cotovelo
        for sign3 in (1.0, -1.0):

            q3 = sign3 * elbow_angle - beta

            phi = q3 + beta

            A = a2 - L * math.cos(phi)
            B = L * math.sin(phi)

            q2 = (
                math.atan2(rho, h)
                - math.atan2(B, A)
            )

            # Rotacao das tres primeiras juntas
            c23 = math.cos(q2 - q3)
            s23 = math.sin(q2 - q3)

            R03 = np.array([
                [c1 * c23, c1 * s23,  s1],
                [s1 * c23, s1 * s23, -c1],
                [-s23,     c23,       0.0]
            ], dtype=float)

            # Orientacao relativa do punho
            R36 = R03.T @ R_d

            r13 = R36[0, 2]
            r21 = R36[1, 0]
            r22 = R36[1, 1]
            r23 = R36[1, 2]
            r31 = R36[2, 0]
            r32 = R36[2, 1]
            r33 = R36[2, 2]

            sin5_abs = math.hypot(r13, r33)

            theta5 = math.atan2(sin5_abs, -r23)

            wrist_solutions = []

            # Singularidade: eixos alinhados
            if theta5 < TOL_WRIST:

                q4 = 0.0
                q5 = 0.0
                q6 = math.atan2(r31, r32)

                wrist_solutions.append((q4, q5, q6))

            # Singularidade: eixos opostos
            elif math.pi - theta5 < TOL_WRIST:

                q4 = 0.0
                q5 = math.pi
                q6 = math.atan2(r31, r32)

                wrist_solutions.append((q4, q5, q6))

            else:

                # Dois ramos regulares do punho
                for sign5 in (1.0, -1.0):

                    s5 = sign5 * sin5_abs

                    q5 = math.atan2(s5, -r23)

                    q4 = math.atan2(
                        -r33 / s5,
                        -r13 / s5
                    )

                    q6 = math.atan2(
                        r22 / s5,
                        -r21 / s5
                    )

                    wrist_solutions.append((q4, q5, q6))

            # Normalizar e eliminar solucoes repetidas
            for q4, q5, q6 in wrist_solutions:

                q = np.array([
                    normalize(q1),
                    normalize(q2),
                    normalize(q3),
                    normalize(q4),
                    normalize(q5),
                    normalize(q6)
                ], dtype=float)

                if not any(
                    same_solution(q, existing)
                    for existing in solutions
                ):
                    solutions.append(q)

    return solutions



def select_solution(solutions, q_current):
    """The solution that is executed, chosen among `solutions`.

    Args:
        solutions: list of (6,) joint vectors (rad), as inverse_kinematics()
                   returns them. Their angles may be any real numbers: this
                   function must not assume they are in (-pi, pi].
        q_current: six joint angles (rad), the current configuration of the arm.

    Returns:
        A (6,) float array, the selected joint vector, or None if no solution
        is executable (no solutions at all, or all discarded). The inputs are
        not modified.

    The rule is the one of Part A: in each solution, every angle q_i is
    replaced by the value q_i + 2 pi k (k an integer) that lies inside the
    range of joint i (JOINT_LIMITS) and is closest to q_current[i]; a solution
    for which no such value exists is discarded. Among the solutions that
    remain, the one at the smallest euclidean distance from q_current is
    selected, and the returned vector is that solution with the replaced angles.
    """
    # TODO (LW2, Part B)
    #raise NotImplementedError('select_solution() is not implemented yet')

    if not solutions:
        return None

    q_current = np.asarray(
        q_current, dtype=float
    ).reshape(6)

    best_solution = None
    best_distance = float('inf')

    for solution in solutions:

        q = np.asarray(
            solution, dtype=float
        ).reshape(6)

        adjusted = np.zeros(6, dtype=float)
        valid = True

        for i in range(6):

            lower = JOINT_LIMITS[i, 0]
            upper = JOINT_LIMITS[i, 1]

            angle = q[i]

            # Valores inteiros de k para os quais
            # lower <= angle + 2*pi*k <= upper
            k_min = math.ceil(
                (lower - angle) / (2.0 * math.pi)
                - 1e-12
            )

            k_max = math.floor(
                (upper - angle) / (2.0 * math.pi)
                + 1e-12
            )

            candidates = []

            for k in range(k_min, k_max + 1):

                candidate = angle + 2.0 * math.pi * k

                if lower - 1e-12 <= candidate <= upper + 1e-12:
                    candidates.append(
                        min(upper, max(lower, candidate))
                    )

            if not candidates:
                valid = False
                break

            adjusted[i] = min(
                candidates,
                key=lambda value: abs(value - q_current[i])
            )

        if not valid:
            continue

        distance = np.linalg.norm(
            adjusted - q_current
        )

        if distance < best_distance:
            best_distance = distance
            best_solution = adjusted.copy()

    return best_solution

# =============================================================================
# Part B - the node
# =============================================================================
class IkNode(Node):
    """Solves the inverse kinematics of every desired pose it receives.

    Subscribes   target_topic         geometry_msgs/PoseStamped  (/lite6_ik/target)
                 joint_states_topic   sensor_msgs/JointState     (/joint_states)
    Publishes    solution_topic       sensor_msgs/JointState     (/lite6_ik/solution)

    The target is the pose of link_eef in link_base (header.frame_id is
    BASE_FRAME). For each target received, the node computes the solutions and
    selects one from the latest joint angles received. If there is a selected
    solution it publishes a JointState:
        name             JOINT_NAMES
        position         the selected solution, in the order of JOINT_NAMES
        header.stamp     the stamp of the TARGET it answers (copy it:
                         ik_check matches the solution to the target by it)
    Otherwise it publishes nothing and logs why:
        unreachable      the pose has no solution at all
        out_of_range     it has solutions, but all of them were discarded

    The topic names are constructor arguments so that the tests can run the
    node on private topics: use them, do not write the names again.
    """

    def __init__(self, target_topic='/lite6_ik/target', joint_states_topic='/joint_states',
                 solution_topic='/lite6_ik/solution'):
        super().__init__('ik_node')
        # TODO (LW2, Part B): create the publisher on solution_topic and the
        # two subscriptions (callbacks: self.on_joint_state, self.on_target).
        #raise NotImplementedError('IkNode.__init__ is not implemented yet')

        self._q_current = None

        self._solution_pub = self.create_publisher(
            JointState,
            solution_topic,
            10
        )

        self._joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.on_joint_state,
            10
        )

        self._target_sub = self.create_subscription(
            PoseStamped,
            target_topic,
            self.on_target,
            10
        )


    def on_joint_state(self, msg):
        """Called for every JointState: keep the latest joint angles."""
        # TODO (LW2, Part B)
        #   Pick the angles of JOINT_NAMES BY NAME from msg.name / msg.position:
        #   the order is not promised and the message may carry other joints.
        #   Store them; do nothing else here (this runs inside the spin, and
        #   joint states arrive hundreds of times per second).
        #raise NotImplementedError('IkNode.on_joint_state is not implemented yet')

        indices = {
            name: i
            for i, name in enumerate(msg.name)
        }

        if not all(
            name in indices
            and indices[name] < len(msg.position)
            for name in JOINT_NAMES
        ):
            return

        self._q_current = np.array([
            msg.position[indices[name]]
            for name in JOINT_NAMES
        ], dtype=float)


    def on_target(self, msg):
        """Called for every desired pose: solve it and publish the solution."""
        # TODO (LW2, Part B)
        #   1. if no joint states have arrived yet, log a warning and return
        #   2. T_d = transform_from_pose(msg)
        #   3. compute the solutions and select one from the latest angles,
        #      with the two functions above
        #   4. if there is a selected solution publish the JointState described
        #      in the class docstring; otherwise log unreachable or
        #      out_of_range (and do not publish)
        #raise NotImplementedError('IkNode.on_target is not implemented yet')

        if self._q_current is None:
            self.get_logger().warn(
                'No joint states received yet'
            )
            return

        if msg.header.frame_id != BASE_FRAME:
            self.get_logger().warn(
                f'Invalid target frame: {msg.header.frame_id}'
            )
            return

        T_d = transform_from_pose(msg)

        solutions = inverse_kinematics(T_d)

        if not solutions:
            self.get_logger().warn('unreachable')
            return

        selected = select_solution(
            solutions,
            self._q_current
        )

        if selected is None:
            self.get_logger().warn('out_of_range')
            return

        result = JointState()

        result.header.stamp = msg.header.stamp
        result.header.frame_id = BASE_FRAME

        result.name = list(JOINT_NAMES)

        result.position = [
            float(angle)
            for angle in selected
        ]

        self._solution_pub.publish(result)



# =============================================================================
# PROVIDED - DO NOT EDIT. Already implemented; nothing below is yours to change.
# =============================================================================
TOL_SAME = 1e-6     # rad - two solutions closer than this in every angle are the same
TOL_WRIST = 1e-6    # rad - angle between the axes of joints 4 and 6 below which they are aligned

# Joint ranges (rad) of the xarm_description model of the Lite 6, one row per
# joint, [lower, upper].
JOINT_LIMITS = np.array([
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.61799, 2.61799],
    [-0.061087, math.pi * 0.99],
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.1642, 2.1642],
    [-math.pi * 0.99, math.pi * 0.99],
])


def transform_from_pose(msg) -> np.ndarray:
    """(4, 4) transform of a geometry_msgs/PoseStamped.

    The quaternion is normalised first, so a slightly off unit quaternion still
    gives a rotation matrix.
    """
    p, o = msg.pose.position, msg.pose.orientation
    n = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
    x, y, z, w = o.x / n, o.y / n, o.z / n, o.w / n
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def main(args=None):
    """Entry point of ``ros2 run lite6_ik ik_node``."""
    if rclpy is None:
        raise SystemExit('ROS 2 is not available here: run this inside the container.')
    rclpy.init(args=args)
    node = None
    status = 0
    try:
        node = IkNode()
        node.get_logger().info('ik_node running: desired poses in, joint angles out')
        rclpy.spin(node)
    except NotImplementedError as e:
        print(f'\n{e}\nImplement it in lite6_ik/inverse_kinematics.py first.')
        status = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
