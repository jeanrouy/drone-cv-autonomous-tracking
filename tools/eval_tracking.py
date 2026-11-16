#!/usr/bin/env python3
"""Evaluation du tracking (ByteTrack) sur une sequence de reference.

Affiche la chronologie des identifiants et la latence du tracking.
Usage :
    python3 tools/eval_tracking.py data/sequences/ref_01.mkv
    python3 tools/eval_tracking.py data/sequences/ref_01.mkv --stride 2 --show
"""
import argparse
import time

import cv2
import numpy as np
from ultralytics import YOLO

WARMUP_FRAMES = 10


def main():
    parser = argparse.ArgumentParser(description="Evaluation du tracking sur une sequence.")
    parser.add_argument('video')
    parser.add_argument('--model', default='models/yolov8n_320_openvino_model')
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--tracker', default='bytetrack.yaml')
    parser.add_argument('--stride', type=int, default=1,
                        help="ne traiter qu'une image sur N (simule des images sautees)")
    parser.add_argument('--show', action='store_true', help="afficher l'image annotee")
    args = parser.parse_args()

    model = YOLO(args.model, task='detect')
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    tracks = {}          # id -> liste des instants (s) ou la piste est presente
    untracked = 0        # images avec detection mais sans piste confirmee
    empty = 0            # images sans aucune detection
    latencies = []
    frame_index, processed = -1, 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_index += 1
        if frame_index % args.stride != 0:
            continue
        t = frame_index / fps

        t0 = time.perf_counter()
        result = model.track(frame, persist=True, tracker=args.tracker, imgsz=args.imgsz,
                             classes=[0], conf=args.conf, verbose=False)[0]
        latency_ms = (time.perf_counter() - t0) * 1000
        processed += 1
        if processed > WARMUP_FRAMES:
            latencies.append(latency_ms)

        if result.boxes.id is not None:
            for track_id in result.boxes.id.int().tolist():
                tracks.setdefault(track_id, []).append(t)
        elif len(result.boxes) > 0:
            untracked += 1
        else:
            empty += 1

        if args.show:
            cv2.imshow('tracking (q pour quitter)', result.plot())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
    step = args.stride / fps
    print(f"\n=== Tracking : {args.video} | 1 image sur {args.stride} "
          f"({fps / args.stride:.0f} img/s simulees) ===")
    print(f"Images traitees : {processed} | sans detection : {empty} | "
          f"detection sans piste confirmee : {untracked}")
    print(f"Identifiants distincts : {len(tracks)}")
    print(" ID | apparition | disparition | presence")
    for track_id, times in sorted(tracks.items(), key=lambda item: item[1][0]):
        span = times[-1] - times[0] + step
        presence = 100.0 * len(times) * step / span
        print(f"{track_id:>3} | {times[0]:8.1f} s | {times[-1]:9.1f} s | "
              f"{len(times) * step:5.1f} s ({presence:.0f} %)")
    lat = np.array(latencies)
    print(f"Latence detection + tracking : mediane {np.median(lat):.1f} ms, "
          f"p95 {np.percentile(lat, 95):.1f} ms")


if __name__ == '__main__':
    main()