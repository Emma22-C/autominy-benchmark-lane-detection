#!/usr/bin/env python3
# Copyright 2026 J. E. Vidal-Cuevas et al.
# Licensed under the Apache License, Version 2.0 (the "License").
"""
lane_detection_cnn_node.py

Nodo ROS2 (rclpy) que ejecuta el U-Net entrenado sobre la imagen IPM y
publica el MISMO contrato de topics que lane_detection_fcm / _sliding_windows
/ _hough, para que metrics_collector_node lo consuma sin cambios:

  ~/center_deviation     std_msgs/Float32  (px, IPM space)
  ~/angle_deviation      std_msgs/Float32  (deg)
  ~/processing_time_ms   std_msgs/Float32  (ms por frame, IPM+inferencia+postproc)
  ~/detection_status     std_msgs/UInt8    (0=ninguno, 1=L, 2=R, 3=L+R)
  ~/debug_image          sensor_msgs/Image (BGR8, si publish_debug_image=true)

A diferencia de lane_detection_fcm, este nodo NO publica xie_beni_*/fpc_*
(son métricas específicas de FCM) -- metrics_collector_node debe tratar esas
columnas como NaN para la fila 'algo=cnn', tal como se definió cuando se
diseñó el esquema del CSV.

Por qué Python y no C++/LibTorch:
El resto de los nodos son C++ y usan lane_common::Preprocessor directamente.
Aquí se optó por rclpy + PyTorch nativo (en vez de exportar a TorchScript y
enlazar LibTorch en C++) porque el checkpoint ya está en formato nativo
PyTorch (.pt con state_dict) y evita una capa extra de conversión/validación.
El costo es que el paso de IPM se reimplementa en Python (ver
preprocessing.py) en vez de reusar la clase C++ -- ver la nota en ese archivo
sobre cómo verificar que ambas implementaciones coinciden.
"""

import time

import cv2
import cv_bridge
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, UInt8

from lane_detection_cnn.model import load_unet
from lane_detection_cnn.pose_estimation import estimate_pose_and_angle, extract_band_centroids
from lane_detection_cnn.preprocessing import PreprocessingParams, Preprocessor, RoiParams


class LaneDetectionCnnNode(Node):

    def __init__(self):
        super().__init__("lane_detection_cnn")

        self._declare_and_load_parameters()

        pp_params = PreprocessingParams(
            resize_factor=self.get_parameter("preprocessing.resize_factor").value,
            ipm_src_flat=list(self.get_parameter("preprocessing.ipm_src_flat").value),
            ipm_dst_flat=list(self.get_parameter("preprocessing.ipm_dst_flat").value),
            ipm_out_width=self.get_parameter("preprocessing.ipm_out_width").value,
            ipm_out_height=self.get_parameter("preprocessing.ipm_out_height").value,
            roi_left=RoiParams(
                x=self.get_parameter("preprocessing.roi_left_x").value,
                y=self.get_parameter("preprocessing.roi_left_y").value,
                width=self.get_parameter("preprocessing.roi_left_width").value,
                height=self.get_parameter("preprocessing.roi_left_height").value,
            ),
            roi_right=RoiParams(
                x=self.get_parameter("preprocessing.roi_right_x").value,
                y=self.get_parameter("preprocessing.roi_right_y").value,
                width=self.get_parameter("preprocessing.roi_right_width").value,
                height=self.get_parameter("preprocessing.roi_right_height").value,
            ),
        )
        self._preprocessor = Preprocessor(pp_params)

        self._camera_center_offset = self.get_parameter("camera_center_offset").value
        self._n_bands = self.get_parameter("cnn.n_bands").value
        self._threshold = self.get_parameter("cnn.threshold").value
        self._publish_debug_image = self.get_parameter("publish_debug_image").value

        self._device = self._resolve_device(self.get_parameter("cnn.device").value)
        model_path = self._resolve_model_path(self.get_parameter("cnn.model_path").value)
        base_channels = self.get_parameter("cnn.base_channels").value

        self.get_logger().info(f"Cargando checkpoint U-Net desde: {model_path}")
        self._model, val_iou, epoch = load_unet(
            model_path, base_channels=base_channels, device=self._device
        )
        self.get_logger().info(
            f"U-Net cargado. device={self._device} epoch={epoch} val_iou={val_iou}"
        )

        self._bridge = cv_bridge.CvBridge()

        input_topic = self.get_parameter("input_image_topic").value
        self._image_sub = self.create_subscription(
            Image, input_topic, self._image_callback, qos_profile_sensor_data
        )

        self._center_pub = self.create_publisher(Float32, "~/center_deviation", 10)
        self._angle_pub = self.create_publisher(Float32, "~/angle_deviation", 10)
        self._time_pub = self.create_publisher(Float32, "~/processing_time_ms", 10)
        self._status_pub = self.create_publisher(UInt8, "~/detection_status", 10)
        self._debug_pub = None
        if self._publish_debug_image:
            self._debug_pub = self.create_publisher(Image, "~/debug_image", 1)

        self.get_logger().info(
            f"lane_detection_cnn listo. Suscrito a: {input_topic} | "
            f"n_bands={self._n_bands}, threshold={self._threshold}"
        )

    # ---- Setup helpers ----

    def _declare_and_load_parameters(self):
        self.declare_parameter("input_image_topic", "/sensors/camera/color/image_rect_color")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("camera_center_offset", 7)  # mismo default que FCM; el YAML calibrado lo sobreescribe

        self.declare_parameter("preprocessing.resize_factor", 0.5)
        self.declare_parameter("preprocessing.ipm_out_width", 320)
        self.declare_parameter("preprocessing.ipm_out_height", 240)
        self.declare_parameter(
            "preprocessing.ipm_src_flat",
            [56.0, 110.0, 250.0, 110.0, 0.0, 200.0, 320.0, 200.0],
        )
        self.declare_parameter(
            "preprocessing.ipm_dst_flat",
            [56.0, 0.0, 250.0, 0.0, 56.0, 240.0, 250.0, 240.0],
        )
        self.declare_parameter("preprocessing.roi_left_x", 0)
        self.declare_parameter("preprocessing.roi_left_y", 0)
        self.declare_parameter("preprocessing.roi_left_width", 160)
        self.declare_parameter("preprocessing.roi_left_height", 240)
        self.declare_parameter("preprocessing.roi_right_x", 160)
        self.declare_parameter("preprocessing.roi_right_y", 0)
        self.declare_parameter("preprocessing.roi_right_width", 160)
        self.declare_parameter("preprocessing.roi_right_height", 240)

        self.declare_parameter("cnn.model_path", "models/unet_best.pt")
        self.declare_parameter("cnn.base_channels", 16)
        self.declare_parameter("cnn.threshold", 0.5)
        self.declare_parameter("cnn.n_bands", 4)  # igual a fcm.n_clusters por defecto
        self.declare_parameter("cnn.device", "auto")  # auto | cpu | cuda

    def _resolve_device(self, device_param: str) -> str:
        if device_param == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        return device_param

    def _resolve_model_path(self, model_path: str) -> str:
        """Si model_path es relativo, lo resuelve contra el share/ instalado del paquete."""
        import os
        if os.path.isabs(model_path):
            return model_path
        share_dir = get_package_share_directory("lane_detection_cnn")
        return os.path.join(share_dir, model_path)

    # ---- Callback principal ----

    def _image_callback(self, msg: Image):
        t_start = time.monotonic()

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # 1. IPM (equivalente al Preprocessor de lane_common, ver preprocessing.py)
        ipm_gray, ipm_bgr = self._preprocessor.process(bgr)

        # 2. Inferencia U-Net
        tensor = torch.from_numpy(ipm_gray.astype(np.float32) / 255.0)
        tensor = tensor.unsqueeze(0).unsqueeze(0).to(self._device)  # (1,1,H,W)
        with torch.no_grad():
            logits = self._model(tensor)
            prob = torch.sigmoid(logits)
        mask = (prob > self._threshold).squeeze().cpu().numpy().astype(np.uint8)

        # 3. Split ROI + extracción de centroides por bandas
        p = self._preprocessor.params
        mask_left = self._preprocessor.crop_roi(mask, p.roi_left)
        mask_right = self._preprocessor.crop_roi(mask, p.roi_right)
        centroids_left = extract_band_centroids(mask_left, p.roi_left.x, self._n_bands)
        centroids_right = extract_band_centroids(mask_right, p.roi_right.x, self._n_bands)

        # 4. Pose (mismo puerto que LaneDetectionFcmNode::estimatePoseAndAngle)
        pose = estimate_pose_and_angle(
            centroids_left, centroids_right,
            ipm_gray.shape[1], ipm_gray.shape[0],
            self._camera_center_offset,
        )

        dt_ms = (time.monotonic() - t_start) * 1000.0

        # 5. Publicar (mismo contrato que lane_detection_fcm)
        self._center_pub.publish(Float32(data=pose.center_deviation_px))
        self._angle_pub.publish(Float32(data=pose.angle_deg))
        self._time_pub.publish(Float32(data=float(dt_ms)))
        self._status_pub.publish(UInt8(data=pose.status))

        if self._debug_pub is not None:
            self._publish_debug_image_msg(ipm_bgr, mask, pose, dt_ms, msg.header)

    def _publish_debug_image_msg(self, ipm_bgr, mask, pose, dt_ms, header):
        overlay = ipm_bgr.copy()
        # Overlay semitransparente de la máscara
        colored = np.zeros_like(overlay)
        colored[mask > 0] = (0, 200, 0)
        overlay = cv2.addWeighted(overlay, 1.0, colored, 0.4, 0)

        for (x, y) in pose.centroids_left:
            cv2.circle(overlay, (x, y), 4, (0, 255, 255), -1)
        for (x, y) in pose.centroids_right:
            cv2.circle(overlay, (x, y), 4, (255, 0, 255), -1)

        text = f"CNN | angle={pose.angle_deg:.1f} dev={pose.center_deviation_px:.1f} {dt_ms:.1f}ms"
        cv2.putText(
            overlay, text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1,
        )

        debug_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        debug_msg.header = header
        self._debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionCnnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
