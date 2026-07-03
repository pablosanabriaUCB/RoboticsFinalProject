#!/usr/bin/env python3
"""
Marcador Interactivo para RViz.
Crea una esfera roja arrastrable que publica PointStamped
en /eye_tracker/target_position.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers import InteractiveMarkerServer


class TargetMarkerNode(Node):
    def __init__(self):
        super().__init__('target_marker')

        self.declare_parameter('initial_x', 0.25)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.15)
        self.declare_parameter('marker_scale', 0.04)

        init_x = self.get_parameter('initial_x').value
        init_y = self.get_parameter('initial_y').value
        init_z = self.get_parameter('initial_z').value
        scale = self.get_parameter('marker_scale').value

        self.target_pub = self.create_publisher(
            PoseStamped, '/eye_tracker/target_pose', 10)

        self.server = InteractiveMarkerServer(self, 'target_marker')

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.name = 'ik_target'
        int_marker.description = 'Arrastra para mover y rotar el target IK'
        int_marker.pose.position.x = init_x
        int_marker.pose.position.y = init_y
        int_marker.pose.position.z = init_z
        int_marker.scale = 0.15

        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = scale
        sphere_marker.scale.y = scale
        sphere_marker.scale.z = scale
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.2
        sphere_marker.color.b = 0.2
        sphere_marker.color.a = 0.9

        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_marker)
        int_marker.controls.append(sphere_control)

        # Controles de posición (Flechas)
        control_x = InteractiveMarkerControl()
        control_x.name = 'move_x'
        control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_x.orientation.w = 1.0
        control_x.orientation.x = 1.0
        control_x.orientation.y = 0.0
        control_x.orientation.z = 0.0
        int_marker.controls.append(control_x)

        control_y = InteractiveMarkerControl()
        control_y.name = 'move_y'
        control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_y.orientation.w = 1.0
        control_y.orientation.x = 0.0
        control_y.orientation.y = 0.0
        control_y.orientation.z = 1.0
        int_marker.controls.append(control_y)

        control_z = InteractiveMarkerControl()
        control_z.name = 'move_z'
        control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_z.orientation.w = 1.0
        control_z.orientation.x = 0.0
        control_z.orientation.y = 1.0
        control_z.orientation.z = 0.0
        int_marker.controls.append(control_z)

        # Control de Rotación (Anillos) - Especialmente para Torsión en X
        rot_x = InteractiveMarkerControl()
        rot_x.name = 'rotate_x'
        rot_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rot_x.orientation.w = 1.0
        rot_x.orientation.x = 1.0
        rot_x.orientation.y = 0.0
        rot_x.orientation.z = 0.0
        int_marker.controls.append(rot_x)

        self.server.insert(int_marker, feedback_callback=self.marker_feedback)
        self.server.applyChanges()

        # Pose inicial neutra (sin rotación)
        self.publish_target(init_x, init_y, init_z, 0.0, 0.0, 0.0, 1.0)
        self.get_logger().info(
            f'=== Marcador Interactivo ACTIVO en ({init_x}, {init_y}, {init_z}) ===')

    def marker_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            p = feedback.pose.position
            q = feedback.pose.orientation
            self.publish_target(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def publish_target(self, px, py, pz, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(px)
        msg.pose.position.y = float(py)
        msg.pose.position.z = float(pz)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TargetMarkerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
