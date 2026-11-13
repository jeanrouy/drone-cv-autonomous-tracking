"""Noeud de detection : camera -> YOLOv8n (OpenVINO) -> vision_msgs/Detection2DArray."""
import os
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class LatestFrameGrabber:
    """Lit la camera en continu dans un thread et ne garde que l'image la plus recente."""

    def __init__(self, device, width, height, fps, now_fn):
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        if not self._cap.isOpened():
            raise RuntimeError(f"camera '{device}' introuvable")

        self._now_fn = now_fn
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = None        # horodatage ROS de la capture
        self._t_capture = None    # horloge monotone, pour mesurer la latence
        self._frame_id = 0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            stamp, t_capture = self._now_fn(), time.monotonic()
            with self._lock:
                self._frame, self._stamp, self._t_capture = frame, stamp, t_capture
                self._frame_id += 1

    def latest(self):
        with self._lock:
            return self._frame_id, self._frame, self._stamp, self._t_capture

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.declare_parameter('device', os.environ.get('FM_CAMERA', '/dev/video0'))
        self.declare_parameter('model', 'models/yolov8n_320_openvino_model')
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('publish_image', False)

        device = self.get_parameter('device').value
        model_path = self.get_parameter('model').value
        self.imgsz = self.get_parameter('imgsz').value
        self.conf = self.get_parameter('conf').value
        publish_image = self.get_parameter('publish_image').value

        # Empeche la camera de diviser sa cadence par deux en faible lumiere
        subprocess.run(['v4l2-ctl', '-d', device, '--set-ctrl=exposure_dynamic_framerate=0'], check=False)

        self.get_logger().info(f"Chargement du modele {model_path} (imgsz={self.imgsz})...")
        self.model = YOLO(model_path, task='detect')
        self.model(np.zeros((480, 640, 3), dtype=np.uint8), imgsz=self.imgsz, verbose=False)  # chauffe

        self.grabber = LatestFrameGrabber(device, 640, 480, 30,
                                          lambda: self.get_clock().now().to_msg())
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection2DArray, '/fm/detections', qos_profile_sensor_data)
        self.img_pub = (self.create_publisher(Image, '/fm/image_annotated', qos_profile_sensor_data)
                        if publish_image else None)

        self.last_frame_id = 0
        self.latencies_ms = []
        self.processed = 0
        self.skipped = 0
        self.stats_start = time.monotonic()

        self.timer = self.create_timer(0.01, self.on_timer)          # verifie s'il y a une image neuve
        self.stats_timer = self.create_timer(5.0, self.report_stats)
        self.get_logger().info(f"Detection en cours sur {device}")

    def on_timer(self):
        frame_id, frame, stamp, t_capture = self.grabber.latest()
        if frame is None or frame_id == self.last_frame_id:
            return   # pas d'image neuve depuis la derniere detection
        if self.last_frame_id:
            self.skipped += frame_id - self.last_frame_id - 1
        self.last_frame_id = frame_id

        result = self.model(frame, imgsz=self.imgsz, classes=[0], conf=self.conf, verbose=False)[0]

        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        for (cx, cy, w, h), score in zip(result.boxes.xywh.tolist(), result.boxes.conf.tolist()):
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = score
            det.results.append(hypothesis)
            msg.detections.append(det)
        self.det_pub.publish(msg)

        self.latencies_ms.append((time.monotonic() - t_capture) * 1000.0)
        self.processed += 1

        if self.img_pub is not None:
            image = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
            image.header = msg.header
            self.img_pub.publish(image)

    def report_stats(self):
        elapsed = time.monotonic() - self.stats_start
        if self.latencies_ms:
            lat = np.array(self.latencies_ms)
            self.get_logger().info(
                f"{self.processed / elapsed:.1f} img/s traitees | {self.skipped} sautees | "
                f"latence capture->publication : mediane {np.median(lat):.1f} ms, "
                f"p95 {np.percentile(lat, 95):.1f} ms")
        else:
            self.get_logger().warn("Aucune image traitee depuis 5 s")
        self.latencies_ms.clear()
        self.processed = 0
        self.skipped = 0
        self.stats_start = time.monotonic()

    def destroy_node(self):
        self.grabber.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()