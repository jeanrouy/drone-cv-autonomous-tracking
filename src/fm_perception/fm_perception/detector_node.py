"""Noeud de perception : camera -> YOLOv8n (OpenVINO) + ByteTrack -> selection de la cible.

Publie :
  /fm/detections       vision_msgs/Detection2DArray  toutes les pistes (identifiant du tracker dans det.id)
  /fm/target           fm_interfaces/TargetState     la cible et son etat
  /fm/image_annotated  sensor_msgs/Image             (option publish_image) cible encadree en vert
Service :
  /fm/lock_target      fm_interfaces/LockTarget      designation de la cible par l'operateur
"""
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

from fm_interfaces.msg import TargetState
from fm_interfaces.srv import LockTarget
from fm_perception.target_selector import (LOCKED, LOST, SEARCHING, STATE_NAMES,
                                           TargetSelector, Track)

# Les etats du selecteur doivent etre ceux du message : verification au chargement du module
assert (TargetState.STATE_SEARCHING, TargetState.STATE_LOCKED, TargetState.STATE_LOST) == \
    (SEARCHING, LOCKED, LOST), "etats du selecteur et de TargetState divergents"

STATE_COLORS = {SEARCHING: (200, 200, 200), LOCKED: (0, 200, 0), LOST: (0, 165, 255)}   # BGR


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
        self._stamp = None
        self._t_capture = None
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
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('publish_image', False)
        self.declare_parameter('relock_radius', 0.15)
        self.declare_parameter('lost_grace', 0.35)
        self.declare_parameter('relock_delay', 0.5)
        self.declare_parameter('max_lost_time', 2.0)

        p = lambda name: self.get_parameter(name).value
        device, model_path = p('device'), p('model')
        self.imgsz, self.conf, self.tracker = p('imgsz'), p('conf'), p('tracker')
        publish_image = p('publish_image')

        self.selector = TargetSelector(relock_radius=p('relock_radius'), lost_grace=p('lost_grace'),
                                       relock_delay=p('relock_delay'), max_lost_time=p('max_lost_time'))
        self.logged_events = 0
        self.last_state = SEARCHING

        # Empeche la camera de diviser sa cadence par deux en faible lumiere
        subprocess.run(['v4l2-ctl', '-d', device, '--set-ctrl=exposure_dynamic_framerate=0'], check=False)

        self.get_logger().info(f"Chargement du modele {model_path} (imgsz={self.imgsz})...")
        self.model = YOLO(model_path, task='detect')
        self.model(np.zeros((480, 640, 3), dtype=np.uint8), imgsz=self.imgsz, verbose=False)  # chauffe

        self.grabber = LatestFrameGrabber(device, 640, 480, 30, lambda: self.get_clock().now().to_msg())
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection2DArray, '/fm/detections', qos_profile_sensor_data)
        self.target_pub = self.create_publisher(TargetState, '/fm/target', qos_profile_sensor_data)
        self.img_pub = (self.create_publisher(Image, '/fm/image_annotated', qos_profile_sensor_data)
                        if publish_image else None)
        self.lock_srv = self.create_service(LockTarget, '/fm/lock_target', self.on_lock_request)

        self.last_frame_id = 0
        self.latencies_ms = []
        self.processed = 0
        self.skipped = 0
        self.stats_start = time.monotonic()

        self.timer = self.create_timer(0.01, self.on_timer)
        self.stats_timer = self.create_timer(5.0, self.report_stats)
        self.get_logger().info(f"Perception en cours sur {device} ; "
                               f"en attente d'une designation sur /fm/lock_target")

    # ---------- Service : designation par l'operateur ----------
    def on_lock_request(self, request, response):
        track_id = None if request.track_id < 0 else request.track_id
        self.selector.request_lock(track_id)
        what = "la piste la plus proche du centre" if track_id is None else f"la piste {track_id}"
        response.accepted = True
        response.message = f"demande enregistree : verrouillage sur {what} des qu'elle sera visible"
        self.get_logger().info(response.message)
        return response

    # ---------- Boucle de perception ----------
    def on_timer(self):
        frame_id, frame, stamp, t_capture = self.grabber.latest()
        if frame is None or frame_id == self.last_frame_id:
            return
        if self.last_frame_id:
            self.skipped += frame_id - self.last_frame_id - 1
        self.last_frame_id = frame_id

        result = self.model.track(frame, persist=True, tracker=self.tracker, imgsz=self.imgsz,
                                  classes=[0], conf=self.conf, verbose=False)[0]
        tracks = self.to_tracks(result)
        target = self.selector.update(t_capture, tracks)
        self.last_state = target.state
        self.log_selector_events()

        self.publish_detections(result, stamp)
        self.publish_target(target, stamp)
        self.latencies_ms.append((time.monotonic() - t_capture) * 1000.0)
        self.processed += 1

        if self.img_pub is not None:
            self.publish_annotated(frame, tracks, target, stamp)

    @staticmethod
    def to_tracks(result):
        if result.boxes.id is None:
            return []
        return [Track(tid, u, v, w, h, c) for tid, (u, v, w, h), c in
                zip(result.boxes.id.int().tolist(), result.boxes.xywhn.tolist(), result.boxes.conf.tolist())]

    def log_selector_events(self):
        for _, text in self.selector.events[self.logged_events:]:
            self.get_logger().info(f"[cible] {text}")
        self.logged_events = len(self.selector.events)

    # ---------- Publications ----------
    def publish_detections(self, result, stamp):
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        ids = (result.boxes.id.int().tolist() if result.boxes.id is not None
               else [None] * len(result.boxes))
        for tid, (cx, cy, w, h), score in zip(ids, result.boxes.xywh.tolist(), result.boxes.conf.tolist()):
            det = Detection2D()
            det.header = msg.header
            det.id = '' if tid is None else str(tid)
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

    def publish_target(self, target, stamp):
        msg = TargetState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        msg.state = target.state
        msg.track_id = target.track_id
        msg.u, msg.v = target.u, target.v
        msg.width, msg.height = target.width, target.height
        msg.confidence = target.confidence
        msg.touches_border = target.touches_border
        msg.time_since_seen = target.time_since_seen
        self.target_pub.publish(msg)

    def publish_annotated(self, frame, tracks, target, stamp):
        image = frame.copy()
        img_h, img_w = image.shape[:2]

        def corners(box):
            return ((int((box.u - box.width / 2) * img_w), int((box.v - box.height / 2) * img_h)),
                    (int((box.u + box.width / 2) * img_w), int((box.v + box.height / 2) * img_h)))

        for tr in tracks:
            is_target = target.state == LOCKED and tr.track_id == target.track_id
            color = (0, 200, 0) if is_target else (160, 160, 160)
            p1, p2 = corners(tr)
            cv2.rectangle(image, p1, p2, color, 3 if is_target else 1)
            label = f"CIBLE {tr.track_id}" if is_target else str(tr.track_id)
            cv2.putText(image, label, (p1[0], max(15, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if target.state == LOST:
            p1, p2 = corners(target)
            cv2.rectangle(image, p1, p2, STATE_COLORS[LOST], 1)
        cv2.putText(image, STATE_NAMES[target.state], (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    STATE_COLORS[target.state], 2)

        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        self.img_pub.publish(msg)

    # ---------- Statistiques ----------
    def report_stats(self):
        elapsed = time.monotonic() - self.stats_start
        if self.latencies_ms:
            lat = np.array(self.latencies_ms)
            self.get_logger().info(
                f"{self.processed / elapsed:.1f} img/s | {self.skipped} sautees | "
                f"latence capture->publication : mediane {np.median(lat):.1f} ms, "
                f"p95 {np.percentile(lat, 95):.1f} ms | cible : {STATE_NAMES[self.last_state]}")
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