#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

class PatrolPlanner:
    def __init__(self):
        rospy.init_node('patrol_planner', anonymous=False)
        rospy.loginfo("Patrol Planner (PCA Edition) inicialitzat. Esperant el mapa...")
        
        self.path_pub = rospy.Publisher('/patrol/waypoints', Path, queue_size=10, latch=True)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        self.path_generated = False
        
        # Paràmetres
        self.stride_meters = 1.0        # Distància entre línies paral·leles
        self.safety_margin_meters = 0.8 # Marge per allunyar-se de les parets
        
    def map_callback(self, msg):
        if self.path_generated:
            return
            
        rospy.loginfo("Mapa rebut! Iniciant anàlisi PCA i Boustrophedon...")
        
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        orig_x = msg.info.origin.position.x
        orig_y = msg.info.origin.position.y
        
        grid = np.array(msg.data).reshape((height, width))
        
        # 1. Crear màscara i aplicar Marge de Seguretat (Erosió)
        free_space = np.uint8(grid == 0)
        margin_pixels = int(self.safety_margin_meters / res)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (margin_pixels*2+1, margin_pixels*2+1))
        safe_free_space = cv2.erode(free_space, kernel, iterations=1)
        
        y_idx, x_idx = np.where(safe_free_space == 1)
        if len(x_idx) == 0:
            rospy.logwarn("L'erosió ha eliminat tot l'espai. Redueix el marge.")
            return

        # ---------------------------------------------------------
        # 2. PCA: Trobar l'orientació de l'habitació
        # ---------------------------------------------------------
        # Agrupem coordenades (X, Y)
        coords = np.column_stack((x_idx, y_idx)).astype(np.float32)
        
        # Calculem el PCA
        mean, eigenvectors, _ = cv2.PCACompute2(coords, np.empty((0)))
        
        # Angle del vector principal (l'eix més llarg de l'habitació)
        angle_rad = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])
        angle_deg = np.degrees(angle_rad)
        
        # Volem que l'eix més llarg quedi vertical (90 graus) per fer línies llargues.
        # Rotem la diferència.
        rotation_angle = angle_deg - 90.0
        center = (mean[0, 0], mean[0, 1])
        
        rospy.loginfo(f"PCA: Habitació inclinada a {angle_deg:.1f}°. Rotant {-rotation_angle:.1f}° internament.")
        
        # Matriu de rotació d'OpenCV
        M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)
        
        # Calcular la nova "bounding box" per no retallar el mapa en rotar-lo
        h, w = safe_free_space.shape
        cos_a = np.abs(M[0, 0])
        sin_a = np.abs(M[0, 1])
        nW = int((h * sin_a) + (w * cos_a))
        nH = int((h * cos_a) + (w * sin_a))
        
        # Ajustar el centre de la translació
        M[0, 2] += (nW / 2) - center[0]
        M[1, 2] += (nH / 2) - center[1]
        
        # Rotem la imatge sencera
        rotated_space = cv2.warpAffine(safe_free_space, M, (nW, nH), flags=cv2.INTER_NEAREST)

        # ---------------------------------------------------------
        # 3. Boustrophedon sobre l'espai rotat (Perfectament alineat)
        # ---------------------------------------------------------
        stride_pixels = int(self.stride_meters / res)
        rotated_waypoints = []
        
        ry_idx, rx_idx = np.where(rotated_space == 1)
        if len(rx_idx) == 0:
            return
            
        min_rx, max_rx = np.min(rx_idx), np.max(rx_idx)
        
        going_up = True
        for x in range(min_rx, max_rx, stride_pixels):
            y_indices = np.where(rotated_space[:, x] == 1)[0]
            
            if len(y_indices) > 0:
                segments = np.split(y_indices, np.where(np.diff(y_indices) > 1)[0] + 1)
                
                if not going_up:
                    segments = reversed(segments)
                    
                for seg in segments:
                    y_start = seg[0]
                    y_end = seg[-1]
                    
                    if y_start == y_end:
                        rotated_waypoints.append((x, y_start))
                        continue
                        
                    if going_up:
                        rotated_waypoints.append((x, y_start))
                        rotated_waypoints.append((x, y_end))
                    else:
                        rotated_waypoints.append((x, y_end))
                        rotated_waypoints.append((x, y_start))
                
                going_up = not going_up

        # ---------------------------------------------------------
        # 4. Desfer la rotació per tornar a les coordenades reals
        # ---------------------------------------------------------
        M_inv = cv2.invertAffineTransform(M)
        original_waypoints = []
        
        for (rx, ry) in rotated_waypoints:
            # Multiplicació matricial manual per desfer la translació/rotació
            px = M_inv[0, 0] * rx + M_inv[0, 1] * ry + M_inv[0, 2]
            py = M_inv[1, 0] * rx + M_inv[1, 1] * ry + M_inv[1, 2]
            original_waypoints.append((px, py))

        # 5. Convertir de píxels a metres globals (ROS)
        self.publish_path(original_waypoints, res, orig_x, orig_y)
        self.path_generated = True

    def publish_path(self, waypoints_px, res, orig_x, orig_y):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for (px, py) in waypoints_px:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = (px * res) + orig_x
            pose.pose.position.y = (py * res) + orig_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        rospy.loginfo(f"Ruta generada amb {len(path_msg.poses)} waypoints! Alineació optimitzada.")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PatrolPlanner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
