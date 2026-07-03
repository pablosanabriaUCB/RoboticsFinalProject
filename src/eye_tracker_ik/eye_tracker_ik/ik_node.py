import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    return roll_x

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.declare_parameter('l1', 0.110)
        self.declare_parameter('l2', 0.130)
        self.declare_parameter('l3', 0.160)
        self.declare_parameter('l4', 0.100)

        self.declare_parameter('joint_limits_lower', [-1.57]*4)
        self.declare_parameter('joint_limits_upper', [1.57]*4)

        self.declare_parameter('ik_max_iterations', 100)
        self.declare_parameter('ik_tolerance', 0.005)
        self.declare_parameter('ik_damping', 0.1)
        self.declare_parameter('ik_step_size', 0.2)
        self.declare_parameter('max_joint_speed', 1.5)
        self.declare_parameter('control_rate', 15.0)

        self.l1 = self.get_parameter('l1').value
        self.l2 = self.get_parameter('l2').value
        self.l3 = self.get_parameter('l3').value
        self.l4 = self.get_parameter('l4').value

        self.joint_limits_lower = np.array(self.get_parameter('joint_limits_lower').value)
        self.joint_limits_upper = np.array(self.get_parameter('joint_limits_upper').value)
        self.max_iterations = self.get_parameter('ik_max_iterations').value
        self.tolerance = self.get_parameter('ik_tolerance').value
        self.damping = self.get_parameter('ik_damping').value
        self.step_size = self.get_parameter('ik_step_size').value
        self.max_joint_speed = self.get_parameter('max_joint_speed').value
        self.control_rate = self.get_parameter('control_rate').value

        self.current_joints = np.array([0.0, 0.5, -1.0, 0.0])
        self.target_joints = self.current_joints.copy()
        self.target_position = None
        self.target_q4 = 0.0
        self.new_target = False
        self.last_target_time = 0.0
        # Math Home para que el hardware quede en (90, 90, 160, 90)
        self.home_position = np.array([0.0, 1.5708, 1.2217, 0.0])

        self.target_sub = self.create_subscription(PointStamped, '/eye_tracker/target_position', self.target_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/arm/joint_commands', 10)
        
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('=== Nodo IK Simulación 4-DOF ACTIVO ===')
        self.get_logger().info('===============================================')
        self.get_logger().info('         TABLA DH - BRAZO ROBOT FINAL 2        ')
        self.get_logger().info('===============================================')
        self.get_logger().info('| i |  Theta |    d (Z)   |    a (X)   | alpha|')
        self.get_logger().info('|---|--------|------------|------------|------|')
        self.get_logger().info(f'| 1 |   q1   | {self.l1:6.3f} m |      0     |  90° |')
        self.get_logger().info(f'| 2 |   q2   |      0     | {self.l2:6.3f} m |   0° |')
        self.get_logger().info(f'| 3 |   q3   |      0     | {self.l3:6.3f} m |   0° |')
        self.get_logger().info(f'| 4 |   q4   |      0     | {self.l4:6.3f} m |   0° |')
        self.get_logger().info('===============================================')

    def forward_kinematics(self, q):
        """FK directa para brazo 4-DOF: Yaw, Pitch, Pitch, Pitch (Wrist)"""
        q1, q2, q3, q4 = q
        
        # En plano X-Z (sin yaw)
        x_plane = self.l2 * np.cos(q2) + self.l3 * np.cos(q2 + q3) + self.l4 * np.cos(q2 + q3 + q4)
        z = self.l1 + self.l2 * np.sin(q2) + self.l3 * np.sin(q2 + q3) + self.l4 * np.sin(q2 + q3 + q4)
        
        # Rotar por q1 (Yaw)
        x = x_plane * np.cos(q1)
        y = x_plane * np.sin(q1)
        
        return np.array([x, y, z])

    def compute_jacobian(self, q, delta=1e-5):
        """Jacobiano 3x4 (Posición 3D respecto a 4 ángulos)"""
        J = np.zeros((3, 4))
        for i in range(4):
            q_plus = q.copy()
            q_minus = q.copy()
            q_plus[i] += delta
            q_minus[i] -= delta
            pos_plus = self.forward_kinematics(q_plus)
            pos_minus = self.forward_kinematics(q_minus)
            J[:, i] = (pos_plus - pos_minus) / (2.0 * delta)
        return J

    def solve_ik(self, target_pos, q_initial):
        q = q_initial.copy()
        best_q = q.copy()
        best_error = float('inf')
        
        for _ in range(self.max_iterations):
            current_pos = self.forward_kinematics(q)
            error = target_pos - current_pos
            error_norm = np.linalg.norm(error)
            
            if error_norm < best_error:
                best_error = error_norm
                best_q = q.copy()
                
            if error_norm < self.tolerance:
                return q, True, error_norm
                
            step_error = error
            if error_norm > 0.05:
                step_error = error * (0.05 / error_norm)
                
            J = self.compute_jacobian(q)
            JJT = J @ J.T
            damped = JJT + (self.damping ** 2) * np.eye(3)
            delta_q = J.T @ np.linalg.solve(damped, step_error)
            
            q = q + self.step_size * delta_q
            
            if error_norm == best_error and error_norm > 0.1:
                q += np.random.uniform(-0.05, 0.05, 4)
                
            q = np.clip(q, self.joint_limits_lower, self.joint_limits_upper)
            
        return best_q, False, best_error

    def target_callback(self, msg):
        # El rostro puede estar a 0.5m o 1m de distancia, pero el brazo solo mide 0.39m!
        # Ahora recibimos el centroide de la bbox directamente en X e Y (0 a 1)
        self.face_x = msg.point.x
        self.face_y = msg.point.y
        self.new_target = True
        self.last_target_time = self.get_clock().now().nanoseconds / 1e9

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Si no hay target o han pasado más de 2 segundos, ir a HOME
        if not hasattr(self, 'face_x') or (current_time - self.last_target_time > 2.0):
            self.target_joints = self.home_position.copy()
            
        elif self.new_target:
            # Control Visual Servoing (Cámara montada EN EL BRAZO)
            error_x = self.face_x - 0.5  # Positivo si la cara está a la derecha
            error_y = self.face_y - 0.5  # Positivo si la cara está abajo
            
            # Ajuste Proporcional
            Kp = 0.05 
            
            # Si el brazo huye de la cara en lugar de seguirla (realimentación positiva),
            # cambia estos valores de 1.0 a -1.0 para invertir el sentido de giro.
            INVERT_PAN = 1.0   # Mantenido en 1.0 a petición del usuario
            INVERT_TILT = -1.0  # Mantenido en 1.0
            
            # Pan (Q1): Gira la base
            self.target_joints[0] += (error_x * Kp) * INVERT_PAN
            
            # Tilt (Q2): Levanta o baja el hombro
            self.target_joints[1] -= (error_y * Kp) * INVERT_TILT
            
            # Limites matemáticos de seguridad
            self.target_joints[0] = np.clip(self.target_joints[0], -1.57, 1.57) 
            self.target_joints[1] = np.clip(self.target_joints[1], 0.0, 1.57)
            
            # Los motores 3 y 4 se quedan en la pose HOME para verse naturales
            self.target_joints[2] = self.home_position[2]
            self.target_joints[3] = self.home_position[3]
            
        self.new_target = False

        # Eliminamos el limitador de velocidad en Python porque el Arduino
        # ya tiene su propia interpolación (INTERP_SPEED) que es mucho más suave.
        self.current_joints = self.target_joints.copy()

        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        js_msg.position = self.current_joints.tolist()
        self.joint_state_pub.publish(js_msg)
        
        # Mapeo Exacto para tu Hardware en Grados
        q_deg = np.degrees(self.current_joints)
        hw_q1 = 90.0 - q_deg[0]   # Invierte izquierda/derecha
        hw_q2 = 180.0 - q_deg[1]  # 90 es arriba
        hw_q3 = 90.0 + q_deg[2]   # 90 es recto
        hw_q4 = 90.0 + q_deg[3]   # 90 es centrado
        
        hw_angles = [hw_q1, hw_q2, hw_q3, hw_q4]
        hw_angles = [float(np.clip(a, 0, 180)) for a in hw_angles]
        
        # El Arduino espera valores en radianes de [-1.57, 1.57] (donde 0 es el centro)
        hw_angles_rad = [(a - 90.0) * math.pi / 180.0 for a in hw_angles]
        
        cmd_msg = Float32MultiArray()
        cmd_msg.data = hw_angles_rad
        self.cmd_pub.publish(cmd_msg)
        
        # Log para depuración
        if hasattr(self, 'face_x'):
            self.get_logger().info(f'Face X:{self.face_x:.2f} Y:{self.face_y:.2f} | Math:[{self.target_joints[0]:.2f}, {self.target_joints[1]:.2f}] | HW(deg):[{hw_angles[0]:.0f}, {hw_angles[1]:.0f}]')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = IKNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
