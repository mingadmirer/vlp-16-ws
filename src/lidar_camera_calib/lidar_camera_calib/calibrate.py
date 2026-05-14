#!/usr/bin/env python3
"""LiDAR-Camera extrinsic calibration using chessboard.

Algorithm:
  1. Detect chessboard in camera image → camera→board pose (solvePnP)
  2. Project LiDAR points into camera using approximate R_cl → crop board region
  3. Fit plane to cropped LiDAR points via SVD
  4. After first solve, refine crop with updated R_cl, iterate
  5. Output: Rcl (3x3 row-major in ROS convention), Pcl (3D translation)

Usage:
  ros2 run lidar_camera_calib calibrate
  # Hold chessboard visible to both sensors, move to different poses.
  # Press Ctrl+C when enough frames collected (≥15).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import struct


class LidarCameraCalib(Node):
    def __init__(self):
        super().__init__('lidar_camera_calib')

        self.declare_parameter('chessboard_cols', 9)
        self.declare_parameter('chessboard_rows', 6)
        self.declare_parameter('square_size', 0.025)
        self.declare_parameter('min_samples', 15)
        self.declare_parameter('board_margin', 0.02)

        self.cb_cols = self.get_parameter('chessboard_cols').value
        self.cb_rows = self.get_parameter('chessboard_rows').value
        self.square = self.get_parameter('square_size').value
        self.min_samples = self.get_parameter('min_samples').value
        self.margin = self.get_parameter('board_margin').value

        self.pattern = (self.cb_cols, self.cb_rows)
        self.objp = np.zeros((self.cb_cols * self.cb_rows, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cb_cols, 0:self.cb_rows].T.reshape(-1, 2)
        self.objp *= self.square

        self.bridge = CvBridge()
        self.K_raw = None
        self.D = None
        self.K = None   # 3x3
        self.img_w = self.img_h = 0

        # Collected constraints: each frame gives (n_cam, d_cam, n_lidar, d_lidar)
        self.N_cam = []   # board normals in camera frame
        self.d_cam = []
        self.N_lid = []   # board normals in LiDAR frame
        self.d_lid = []

        # Initial guess: assume standard LiDAR→Camera orientation
        # LiDAR: X=forward, Y=left, Z=up
        # Camera (OpenCV): X=right, Y=down, Z=forward
        self.R_cl_init = np.array([
            [ 0.0, -1.0,  0.0],
            [ 0.0,  0.0, -1.0],
            [ 1.0,  0.0,  0.0],
        ])
        self.R_cl = self.R_cl_init.copy()
        self.P_cl = np.zeros(3)

        self.sub_img = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.sub_info = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
        self.sub_pcl = message_filters.Subscriber(self, PointCloud2, '/velodyne_points')

        # Debug counters
        self._cnt_img = 0; self._cnt_info = 0; self._cnt_pcl = 0; self._cnt_cb = 0; self._cnt_detect = 0
        self.create_subscription(Image, '/camera/color/image_raw',
            lambda m: self._inc('_cnt_img'), 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
            lambda m: self._inc('_cnt_info'), 10)
        self.create_subscription(PointCloud2, '/velodyne_points',
            lambda m: self._inc('_cnt_pcl'), 10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            (self.sub_img, self.sub_info, self.sub_pcl), queue_size=5, slop=0.1
        )
        self.ts.registerCallback(self.callback)
        self.create_timer(5.0, self._debug_timer)

        self.get_logger().info('Ready. Hold chessboard visible to both sensors.')
        self.get_logger().info(f'Need ≥{self.min_samples} frames. Press Ctrl+C to compute.')

    def _inc(self, attr):
        setattr(self, attr, getattr(self, attr) + 1)

    def _debug_timer(self):
        self.get_logger().info(
            f'[status] img={self._cnt_img} info={self._cnt_info} pcl={self._cnt_pcl} '
            f'sync_cb={self._cnt_cb} detect={self._cnt_detect} collected={len(self.N_cam)}'
        )

    def callback(self, img_msg, info_msg, pcl_msg):
        self._cnt_cb += 1
        if self.K_raw is None:
            self.K_raw = np.array(info_msg.k).reshape(3, 3)
            self.D = np.array(info_msg.d).ravel()
            self.img_h, self.img_w = info_msg.height, info_msg.width
            self.K = self.K_raw.copy()

        # --- Step 1: detect chessboard, solve camera→board ---
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern, None)
        if not ret:
            return

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        ok, rvec, tvec = cv2.solvePnP(self.objp, corners, self.K, self.D)
        if not ok:
            return
        R_cb, _ = cv2.Rodrigues(rvec)
        t_cb = tvec.ravel()

        n_cam = R_cb[:, 2]                  # board normal in camera frame
        d_cam = float(np.dot(n_cam, t_cb))  # signed distance
        if d_cam < 0:
            n_cam = -n_cam
            d_cam = -d_cam

        # board corners projected to image (for LiDAR cropping)
        corners_2d = corners.reshape(-1, 2)
        hull_2d = cv2.convexHull(corners_2d.astype(np.int32))

        # --- Step 2: crop LiDAR points by distance (no R_cl needed) ---
        # Board is roughly in front of the LiDAR at distance ~d_cam
        pcl_data = pc2.read_points(pcl_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts_lidar = np.array([[p[0], p[1], p[2]] for p in pcl_data])
        if len(pts_lidar) == 0:
            return

        # Select points near the board distance (LiDAR X = forward)
        # d_cam is measured by camera, board is roughly at same physical distance
        dist = np.linalg.norm(pts_lidar, axis=1)
        depth_range = 0.15  # ±15cm tolerance
        on_board = (dist > d_cam - depth_range) & (dist < d_cam + depth_range)

        # Also restrict to a loose forward cone (roughly in front of LiDAR)
        x, y, z = pts_lidar[:, 0], pts_lidar[:, 1], pts_lidar[:, 2]
        forward_cone = x > 0.2  # points in front of LiDAR

        board_pts = pts_lidar[on_board & forward_cone]
        if len(board_pts) < 20:
            return

        # --- Step 3: fit plane to LiDAR board points (SVD) ---
        centroid = board_pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(board_pts - centroid)
        n_lid = Vt[2]           # smallest singular vector = normal
        d_lid = float(np.dot(n_lid, centroid))

        # Ensure normal points toward origin (sensor)
        if d_lid < 0:
            n_lid = -n_lid
            d_lid = -d_lid

        self.N_cam.append(n_cam)
        self.d_cam.append(d_cam)
        self.N_lid.append(n_lid)
        self.d_lid.append(d_lid)

        self._cnt_detect += 1
        n = len(self.N_cam)
        self.get_logger().info(
            f'[{n}] cam_n=[{n_cam[0]:+.3f} {n_cam[1]:+.3f} {n_cam[2]:+.3f}] '
            f'lid_n=[{n_lid[0]:+.3f} {n_lid[1]:+.3f} {n_lid[2]:+.3f}] '
            f'd_cam={d_cam:.3f} d_lid={d_lid:.3f} pts={len(board_pts)}'
        )

    def solve(self):
        """Solve R_cl and P_cl from accumulated plane constraints.

        Constraint: for each frame, the board plane in LiDAR frame
        transformed to camera frame should match the camera-observed plane.
        i.e.  n_lid^T · (R_cl^T · v) + d_lid ≈ n_cam^T · v + d_cam  for any v
        →  R_cl · n_lid = n_cam   (rotation aligns normals)
        →  n_lid^T · P_cl + d_lid ≈ d_cam   (translation aligns distances)
        """

        n = len(self.N_cam)
        if n < self.min_samples:
            self.get_logger().error(f'Need ≥{self.min_samples} frames, got {n}')
            return

        N_cam = np.array(self.N_cam)   # (n, 3)
        N_lid = np.array(self.N_lid)   # (n, 3)

        # Rotation: Kabsch — align LiDAR normals to Camera normals
        H = N_lid.T @ N_cam
        U, _, Vt = np.linalg.svd(H)
        R_cl = Vt.T @ U.T
        if np.linalg.det(R_cl) < 0:
            R_cl[:, 2] *= -1

        # Translation: solve n_lid^T * P_cl ≈ d_cam - d_lid
        d_cam = np.array(self.d_cam)
        d_lid = np.array(self.d_lid)
        A = N_lid          # (n, 3)
        b = d_cam - d_lid
        P_cl = np.linalg.lstsq(A, b, rcond=None)[0]

        # Residual
        residuals = (N_lid @ P_cl) + d_lid - d_cam
        rmse = float(np.sqrt(np.mean(residuals ** 2)))

        print()
        print('=' * 60)
        print('         LiDAR → Camera Extrinsic Calibration')
        print('=' * 60)
        print(f'  Rcl (3x3 row-major, maps LiDAR vector to Camera vector):')
        print(f'    [{R_cl[0,0]:.6f}, {R_cl[0,1]:.6f}, {R_cl[0,2]:.6f}],')
        print(f'    [{R_cl[1,0]:.6f}, {R_cl[1,1]:.6f}, {R_cl[1,2]:.6f}],')
        print(f'    [{R_cl[2,0]:.6f}, {R_cl[2,1]:.6f}, {R_cl[2,2]:.6f}],')
        print(f'  Pcl (translation, LiDAR→Camera, meters):')
        print(f'    [{P_cl[0]:.4f}, {P_cl[1]:.4f}, {P_cl[2]:.4f}]')
        print(f'  Plane distance RMSE: {rmse:.4f} m  ({n} frames)')
        print('=' * 60)
        print()
        print('Copy these into fast_livo config/vlp16_astra.yaml:')
        print(f'  Rcl: [{R_cl[0,0]:.6f}, {R_cl[0,1]:.6f}, {R_cl[0,2]:.6f},')
        print(f'        {R_cl[1,0]:.6f}, {R_cl[1,1]:.6f}, {R_cl[1,2]:.6f},')
        print(f'        {R_cl[2,0]:.6f}, {R_cl[2,1]:.6f}, {R_cl[2,2]:.6f}]')
        print(f'  Pcl: [{P_cl[0]:.4f}, {P_cl[1]:.4f}, {P_cl[2]:.4f}]')


def main():
    rclpy.init()
    node = LidarCameraCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Computing calibration...')
        node.solve()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
