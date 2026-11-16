#!/usr/bin/env python3
"""Evaluation hors ligne du selecteur de cible sur une sequence video.

Usage (workspace source avec fm) :
    python3 tools/eval_selector.py data/sequences/ref_01.mkv
    python3 tools/eval_selector.py data/sequences/ref_01.mkv --out data/eval/ref_01_selector.mp4
"""
import argparse
import os

import cv2
from ultralytics import YOLO

from fm_perception.target_selector import LOCKED, LOST, SEARCHING, STATE_NAMES, TargetSelector, Track

STATE_COLORS = {SEARCHING: (200, 200, 200), LOCKED: (0, 200, 0), LOST: (0, 165, 255)}   # BGR


def box_pixels(u, v, width, height, img_w, img_h):
    return (int((u - width / 2) * img_w), int((v - height / 2) * img_h),
            int((u + width / 2) * img_w), int((v + height / 2) * img_h))


def draw(frame, tracks, target, t):
    img_h, img_w = frame.shape[:2]
    for tr in tracks:
        is_target = target.state == LOCKED and tr.track_id == target.track_id
        color = (0, 200, 0) if is_target else (160, 160, 160)
        x1, y1, x2, y2 = box_pixels(tr.u, tr.v, tr.width, tr.height, img_w, img_h)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3 if is_target else 1)
        label = f"CIBLE {tr.track_id}" if is_target else str(tr.track_id)
        cv2.putText(frame, label, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    if target.state == LOST:   # derniere position connue
        x1, y1, x2, y2 = box_pixels(target.u, target.v, target.width, target.height, img_w, img_h)
        cv2.rectangle(frame, (x1, y1), (x2, y2), STATE_COLORS[LOST], 1)
    text = f"{STATE_NAMES[target.state]}  t={t:.1f} s"
    if target.state == LOCKED and target.touches_border:
        text += "  (bord)"
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, STATE_COLORS[target.state], 2)


def main():
    parser = argparse.ArgumentParser(description="Evaluation du selecteur de cible sur une sequence.")
    parser.add_argument('video')
    parser.add_argument('--model', default='models/yolov8n_320_openvino_model')
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--tracker', default='bytetrack.yaml')
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--relock-radius', type=float, default=0.15)
    parser.add_argument('--lost-grace', type=float, default=0.35) ;
    parser.add_argument('--relock-delay', type=float, default=0.5)
    parser.add_argument('--max-lost-time', type=float, default=2.0)
    parser.add_argument('--out', help="video annotee a ecrire (optionnel)")
    args = parser.parse_args()

    model = YOLO(args.model, task='detect')
    selector = TargetSelector(relock_radius=args.relock_radius, lost_grace=args.lost_grace, relock_delay=args.relock_delay,
                              max_lost_time=args.max_lost_time)
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = args.stride / fps

    writer = None
    durations = {SEARCHING: 0.0, LOCKED: 0.0, LOST: 0.0}
    border_time = 0.0
    followed_ids = []
    frame_index = -1

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_index += 1
        if frame_index % args.stride != 0:
            continue
        t = frame_index / fps

        result = model.track(frame, persist=True, tracker=args.tracker, imgsz=args.imgsz,
                             classes=[0], conf=args.conf, verbose=False)[0]
        tracks = []
        if result.boxes.id is not None:
            for tid, (u, v, w, h), c in zip(result.boxes.id.int().tolist(),
                                            result.boxes.xywhn.tolist(), result.boxes.conf.tolist()):
                tracks.append(Track(tid, u, v, w, h, c))

        target = selector.update(t, tracks)
        durations[target.state] += step
        if target.state == LOCKED and target.touches_border:
            border_time += step
        if target.state != SEARCHING and (not followed_ids or followed_ids[-1] != target.track_id):
            followed_ids.append(target.track_id)

        if args.out:
            if writer is None:
                os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
                writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), fps / args.stride,
                                         (frame.shape[1], frame.shape[0]))
            draw(frame, tracks, target, t)
            writer.write(frame)

    if writer is not None:
        writer.release()

    print(f"\n=== Selecteur : {args.video} | 1 image sur {args.stride} ===")
    print(f"Parametres : relock_radius={args.relock_radius} | lost_grace={args.lost_grace} s | relock_delay={args.relock_delay} s"
          f" | max_lost_time={args.max_lost_time} s")
    print("Journal :")
    for t, text in selector.events:
        print(f"  {t:6.2f} s  {text}")
    print(f"Temps : SEARCHING {durations[SEARCHING]:.1f} s | LOCKED {durations[LOCKED]:.1f} s "
          f"(dont {border_time:.1f} s au bord) | LOST {durations[LOST]:.1f} s")
    print(f"Pistes suivies successivement : {' -> '.join(str(i) for i in followed_ids) or 'aucune'}")
    if args.out:
        print(f"Video annotee : {args.out}")


if __name__ == '__main__':
    main()