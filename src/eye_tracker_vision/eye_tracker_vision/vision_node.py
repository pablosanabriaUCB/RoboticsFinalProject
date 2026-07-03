#!/usr/bin/env python3
"""
Nodo de Visión - Eye Tracker Arm
Detecta la posición del rostro/ojos usando MediaPipe Face Mesh
y publica la posición 3D estimada como target para el brazo robótico.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import numpy as np
import time


class VisionNode(Node):
    def __init__(self):
        super().__init__('eye_tracker_vision')
        
        # --- Parámetros ROS2 ---
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_url', '')
        self.declare_parameter('show_preview', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('camera_focal_length', 600.0)
        self.declare_parameter('average_face_width', 0.15)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('smoothing_alpha', 0.3)
        
        self.camera_index = self.get_parameter('camera_index').value
        self.camera_url = self.get_parameter('camera_url').value
        self.show_preview = self.get_parameter('show_preview').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.focal_length = self.get_parameter('camera_focal_length').value
        self.face_width_m = self.get_parameter('average_face_width').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        
        # --- Publishers ---
        self.target_pub = self.create_publisher(PointStamped, '/eye_tracker/target_position', 10)
        self.iris_pub = self.create_publisher(Float32MultiArray, '/eye_tracker/iris_data', 10)
        
        # --- MediaPipe Face Mesh ---
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # --- Cámara ---
        source = self.camera_url if self.camera_url else self.camera_index
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Evitar lag de OpenCV
        if not self.cap.isOpened():
            self.get_logger().error(f'ERROR: No se pudo abrir la cámara: {source}')
            raise RuntimeError('No se pudo abrir la cámara')
        
        self.img_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.img_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Cámara abierta: {self.img_w}x{self.img_h}')
        
        # --- Estado del filtro ---
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_z = 0.5
        self.first_detection = True
        
        # --- FPS ---
        self.prev_time = time.time()
        self.fps = 0.0
        
        # --- Timer ---
        # Corremos el timer muy rápido (100Hz) para que cap.read() bloquee y se sincronice
        # con los FPS reales de la cámara. Esto evita que los frames viejos se acumulen (lag de 20s)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('=== Nodo de Visión Eye Tracker ACTIVO ===')
    
    def estimate_face_3d_position(self, face_landmarks):
        """
        Estima la posición 3D del rostro usando el ancho aparente del rostro
        y la posición de los landmarks del iris.
        Usa el método de triángulo semejante para estimar la profundidad Z.
        """
        x_min = min([lm.x for lm in face_landmarks.landmark])
        x_max = max([lm.x for lm in face_landmarks.landmark])
        y_min = min([lm.y for lm in face_landmarks.landmark])
        y_max = max([lm.y for lm in face_landmarks.landmark])
        
        centroid_x = (x_min + x_max) / 2.0
        centroid_y = (y_min + y_max) / 2.0
        
        left_face = face_landmarks.landmark[234]
        right_face = face_landmarks.landmark[454]
        
        face_width_px = abs(right_face.x - left_face.x) * self.img_w
        
        if face_width_px > 10:
            z_estimated = (self.focal_length * self.face_width_m) / face_width_px
        else:
            z_estimated = 0.5
        
        cx = self.img_w / 2.0
        cy = self.img_h / 2.0
        
        # Centrar con el centroide de la bbox
        pixel_x = centroid_x * self.img_w
        pixel_y = centroid_y * self.img_h
        
        # Coordenadas 3D en frame cámara (convención ROS: X adelante, Y izquierda, Z arriba)
        x_3d = z_estimated
        y_3d = -(pixel_x - cx) * z_estimated / self.focal_length
        z_3d = -(pixel_y - cy) * z_estimated / self.focal_length
        
        return x_3d, y_3d, z_3d, centroid_x, centroid_y, x_min, y_min, x_max, y_max
    
    def timer_callback(self):
        success, image = self.cap.read()
        if not success:
            return
        
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.face_mesh.process(image_rgb)
        image_rgb.flags.writeable = True
        
        if results.multi_face_landmarks:
            face_landmarks = results.multi_face_landmarks[0]
            
            x_raw, y_raw, z_raw, centroid_nx, centroid_ny, x_min, y_min, x_max, y_max = self.estimate_face_3d_position(face_landmarks)
            
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame
            msg.point.x = centroid_nx
            msg.point.y = centroid_ny
            msg.point.z = 0.0
            self.target_pub.publish(msg)
            
            iris_msg = Float32MultiArray()
            iris_msg.data = [centroid_nx, centroid_ny, z_raw]
            self.iris_pub.publish(iris_msg)
            
            if self.show_preview:
                display_img = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
                
                self.mp_drawing.draw_landmarks(
                    image=display_img,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_IRISES,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_iris_connections_style()
                )
                
                self.mp_drawing.draw_landmarks(
                    image=display_img,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_tesselation_style()
                )
                
                # Dibujar Bounding Box y su Centroide
                px_min = int(x_min * self.img_w)
                py_min = int(y_min * self.img_h)
                px_max = int(x_max * self.img_w)
                py_max = int(y_max * self.img_h)
                pc_x = int(centroid_nx * self.img_w)
                pc_y = int(centroid_ny * self.img_h)
                
                cv2.rectangle(display_img, (px_min, py_min), (px_max, py_max), (0, 255, 0), 2)
                cv2.circle(display_img, (pc_x, pc_y), 5, (0, 0, 255), -1)
                
                # Dibujar línea desde el centro de la imagen hasta el centroide
                ic_x = int(self.img_w / 2)
                ic_y = int(self.img_h / 2)
                cv2.line(display_img, (ic_x, ic_y), (pc_x, pc_y), (255, 0, 0), 2)
                cv2.circle(display_img, (ic_x, ic_y), 5, (255, 0, 0), -1)
                
                cur_time = time.time()
                self.fps = 1.0 / (cur_time - self.prev_time) if (cur_time - self.prev_time) > 0 else 0
                self.prev_time = cur_time
                
                cv2.putText(display_img, f'FPS: {int(self.fps)}', (20, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_img, f'X:{centroid_nx:.2f} Y:{centroid_ny:.2f} Z:{z_raw:.2f}',
                           (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                cv2.imshow('Eye Tracker Vision', display_img)
                cv2.waitKey(1)
        else:
            if self.show_preview:
                cur_time = time.time()
                self.fps = 1.0 / (cur_time - self.prev_time) if (cur_time - self.prev_time) > 0 else 0
                self.prev_time = cur_time
                cv2.putText(image, f'FPS: {int(self.fps)} - Sin rostro', (20, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow('Eye Tracker Vision', image)
                cv2.waitKey(1)
    
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.face_mesh.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
