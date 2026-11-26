"""Labwork 2 - inverse kinematics of the UFACTORY Lite 6 by Newton's method, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 2.

Implement the following, replacing every ``raise NotImplementedError``:

    Part B - the maths (no ROS needed)
        pose_error(T_c, T_d)               the 6-component error between two poses
        jacobian(q, h)                     the numerical 6x6 Jacobian
        inverse_kinematics(T_d, q_start, h)    Newton's method

    Part B - the node
        IkNode                             reads a desired pose, publishes the
                                           joint angles that reach it

Run the node with ``ros2 run lite6_kinematics ik_node``.

``rotation_vector``, ``transform_from_pose``, ``JOINT_LIMITS``, the constants
``H``, ``TOL_POS``, ``TOL_ORI``, ``MAX_ITER``, ``MAX_STEP`` and ``main`` below
are PROVIDED: already implemented, do not edit them.

Keep the function and class names, arguments and return types exactly as they
are: the checker node (``ik_check``), the study program (``ik_study``), the
tests in ``test/`` and the instructor's grader all use them by name.

Conventions - those of Labwork 1:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``. The desired pose T_d is the pose of {6} in {0}.
    * Joint angles in RADIANS, as published on /joint_states.
    * Lengths in METRES.

Allowed imports: ``numpy`` and ``math`` for the maths, ``forward_kinematics``
from your own ``lite6_kinematics.forward_kinematics`` (already imported below),
and ``rclpy``, ``sensor_msgs`` and ``geometry_msgs`` for the node (already
imported below). Nothing else - no MoveIt, no kinematics library, no URDF
parser, no tf. The pose of the end-effector comes only from
``forward_kinematics()``. ``test/test_inverse_kinematics.py`` checks the imports.
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
def pose_error(T_c, T_d) -> np.ndarray:
    """Error between a current pose and a desired pose, equation (1).

    Args:
        T_c: (4, 4) current pose.
        T_d: (4, 4) desired pose.

    Returns:
        (6,) array [p_d - p_c, phi(R_d R_c^T)]: the difference of the positions
        (m), then the rotation vector (rad) of the rotation that takes the
        orientation of T_c to that of T_d, expressed in {0}. Use
        rotation_vector(), which is provided.
    """
    # TODO (LW2, Part B)
    #raise NotImplementedError('pose_error() is not implemented yet')
    T_c = np.asarray(T_c, dtype=float)
    T_d = np.asarray(T_d, dtype=float)

    dp = T_d[:3, 3] - T_c[:3, 3]
    drot = rotation_vector(T_d[:3, :3] @ T_c[:3, :3].T)

    return np.concatenate((dp, drot))


def jacobian(q, h: float = None) -> np.ndarray:
    """Numerical Jacobian of the end-effector pose, equation (2).

    Args:
        q: six joint angles (rad).
        h: increment (rad) added to one joint at a time. Default: H.

    Returns:
        (6, 6) array whose column i is
        pose_error(T(q), T(q + h * delta_i)) / h,
        T being forward_kinematics. Only forward_kinematics() and pose_error()
        are used: the robot does not move.
    """
    if h is None:
        h = H
    # TODO (LW2, Part B)
    #raise NotImplementedError('jacobian() is not implemented yet')
    if h is None:
        h = H

    q = np.asarray(q, dtype=float).reshape(6)
    T_c = forward_kinematics(q)
    J = np.empty((6, 6), dtype=float)

    for i in range(6):
        q_perturbed = q.copy()
        q_perturbed[i] += h

        T_next = forward_kinematics(q_perturbed)
        J[:, i] = pose_error(T_c, T_next) / h

    return J


def inverse_kinematics(T_d, q_start, h: float = None):
    """Joint angles that put the end-effector at T_d: Newton's method, equation (3).

    Args:
        T_d:     (4, 4) desired pose of {6} in {0}.
        q_start: six joint angles (rad), the starting configuration Theta_0.
        h:       increment passed to jacobian(). Default: H.

    Returns:
        (q, status, n_iter):
            q       (6,) array, the final joint angles (rad);
            status  'converged', 'max_iterations', 'singular' or 'out_of_range';
            n_iter  the number of iterations performed: how many times
                    Theta_{k+1} = Theta_k + Delta Theta_k was executed before
                    stopping (0 if q_start already reaches T_d).

    The iteration, the stopping rules, the step limit and the treatment of the
    joint ranges are those of Part A of the handout, with the constants
    TOL_POS, TOL_ORI, MAX_ITER and MAX_STEP and the ranges JOINT_LIMITS.
    """
    if h is None:
        h = H
    # TODO (LW2, Part B)
    #raise NotImplementedError('inverse_kinematics() is not implemented yet')
    
    T_d = np.asarray(T_d, dtype=float)
    q = np.asarray(q_start, dtype=float).reshape(6).copy()

    for i in range(6):
        lower, upper = JOINT_LIMITS[i]

        if not (lower <= q[i] <= upper):
            options = [
                q[i] + 2.0 * math.pi * k
                for k in range(-3, 4)
            ]
            valid = [
                v for v in options
                if lower <= v <= upper
            ]

            if valid:
                q[i] = min(valid, key=lambda v: abs(v - q[i]))

    for n_iter in range(MAX_ITER + 1):
        error = pose_error(forward_kinematics(q), T_d)

        if (
            np.linalg.norm(error[:3]) < TOL_POS
            and np.linalg.norm(error[3:]) < TOL_ORI
        ):
            if (
                np.any(q < JOINT_LIMITS[:, 0])
                or np.any(q > JOINT_LIMITS[:, 1])
            ):
                return q, 'out_of_range', n_iter

            return q, 'converged', n_iter

        if n_iter == MAX_ITER:
            return q, 'max_iterations', n_iter

        J = jacobian(q, h)

        if (
            not np.all(np.isfinite(J))
            or np.linalg.matrix_rank(J) < 6
        ):
            return q, 'singular', n_iter

        try:
            step = np.linalg.solve(J, error)
        except np.linalg.LinAlgError:
            return q, 'singular', n_iter

        if not np.all(np.isfinite(step)):
            return q, 'singular', n_iter

        largest = np.max(np.abs(step))

        if largest > MAX_STEP:
            step *= MAX_STEP / largest

        q += step

    return q, 'max_iterations', MAX_ITER 




# =============================================================================
# Part B - the node
# =============================================================================
class IkNode(Node):
    """Solves the inverse kinematics of every desired pose it receives.

    Subscribes   target_topic         geometry_msgs/PoseStamped  (/lite6_ik/target)
                 joint_states_topic   sensor_msgs/JointState     (/joint_states)
    Publishes    solution_topic       sensor_msgs/JointState     (/lite6_ik/solution)

    The target is the pose of link_eef in link_base (header.frame_id is
    BASE_FRAME). For each target received, the node solves from the latest
    joint angles received, as the starting configuration. If the status is
    'converged' it publishes a JointState:
        name             JOINT_NAMES
        position         the solution, in the order of JOINT_NAMES
        header.stamp     the stamp of the TARGET it answers (copy it:
                         ik_check matches the solution to the target by it)
    Otherwise it publishes nothing and logs the status.

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
            JointState, solution_topic, 10
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
            name: i for i, name in enumerate(msg.name)
        }

        if all(
            name in indices and indices[name] < len(msg.position)
            for name in JOINT_NAMES
        ):
            self._q_current = np.array(
                [msg.position[indices[name]] for name in JOINT_NAMES],
                dtype=float
            )
    

    def on_target(self, msg):
        """Called for every desired pose: solve it and publish the solution."""
        # TODO (LW2, Part B)
        #   1. if no joint states have arrived yet, log a warning and return
        #   2. T_d = transform_from_pose(msg)
        #   3. q, status, n_iter = inverse_kinematics(T_d, <latest angles>)
        #   4. if status == 'converged': publish the JointState described in
        #      the class docstring; otherwise log the status (and do not publish)
        #raise NotImplementedError('IkNode.on_target is not implemented yet')\

        if self._q_current is None:
            self.get_logger().warn(
                'No joint states received yet'
            )
            return

        if msg.header.frame_id != BASE_FRAME:
            self.get_logger().warn(
                f'Invalid target frame: {msg.header.frame_id}; '
                f'expected {BASE_FRAME}'
            )
            return

        T_d = transform_from_pose(msg)

        q, status, n_iter = inverse_kinematics(
            T_d, self._q_current.copy()
        )

        if status != 'converged':
            self.get_logger().warn(
                f'Inverse kinematics: {status} '
                f'after {n_iter} iterations'
            )
            return

        result = JointState()
        result.header.stamp = msg.header.stamp
        result.header.frame_id = BASE_FRAME
        result.name = list(JOINT_NAMES)
        result.position = [float(v) for v in q]

        self._solution_pub.publish(result)
        



# =============================================================================
# PROVIDED - DO NOT EDIT. Already implemented; nothing below is yours to change.
# =============================================================================
H = 1e-6            # rad - increment of each joint in the Jacobian, equation (2)
TOL_POS = 1e-6      # m   - position error below which the pose is reached
TOL_ORI = 1e-6      # rad - orientation error below which the pose is reached
MAX_ITER = 100      #     - maximum number of iterations
MAX_STEP = 0.2      # rad - largest change of one joint in one iteration

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


def rotation_vector(R) -> np.ndarray:
    """Rotation vector phi(R) = theta * u of a 3x3 rotation matrix.

    u is the unit axis of the rotation and theta in [0, pi] its angle, so the
    result has norm theta. The zero vector for the identity.

    Args:
        R: (3, 3) rotation matrix.

    Returns:
        (3,) array, in the frame in which R is expressed.
    """
    R = np.asarray(R, dtype=float)
    v = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(v)                   # sin(theta)
    c = 0.5 * (np.trace(R) - 1.0)           # cos(theta)
    theta = math.atan2(s, c)                # accurate for tiny angles, unlike arccos
    if s > 1e-6 or (s > 0.0 and c > 0.0):
        return (theta / s) * v
    if c > 0.0:                             # R is the identity
        return v
    # theta = pi: sin(theta) vanishes and the axis is read off R = 2 u u^T - I.
    M = 0.5 * (R + np.eye(3))
    i = int(np.argmax(np.diag(M)))
    u = M[:, i] / math.sqrt(M[i, i])
    if np.dot(u, v) < 0.0:
        u = -u
    return math.pi * u


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
    """Entry point of ``ros2 run lite6_kinematics ik_node``."""
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
        print(f'\n{e}\nImplement it in lite6_kinematics/inverse_kinematics.py first.')
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
