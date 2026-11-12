#!/usr/bin/env python3
"""Benchmark de YOLO sur une sequence video de reference.

Usage (depuis la racine du depot) :
    python3 tools/bench_yolo.py data/sequences/ref_01.mkv --model models/yolov8n.pt --imgsz 320 \
        --condition idle --csv docs/bench_phase3.csv
"""
import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np
from ultralytics import YOLO

WARMUP_FRAMES = 10


def run_benchmark(video, model_path, imgsz, conf):
    model = YOLO(model_path, task='detect')
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        sys.exit(f"Impossible d'ouvrir la video '{video}'.")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    latencies, pre, inf, post, confidences = [], [], [], [], []
    frames_with_person, n, t_start = 0, 0, None

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t0 = time.perf_counter()
        result = model(frame, imgsz=imgsz, classes=[0], conf=conf, verbose=False)[0]
        latency_ms = (time.perf_counter() - t0) * 1000
        n += 1

        if n == WARMUP_FRAMES:
            t_start = time.perf_counter()   # le debit est mesure apres la chauffe
        if n <= WARMUP_FRAMES:
            continue

        latencies.append(latency_ms)
        pre.append(result.speed['preprocess'])
        inf.append(result.speed['inference'])
        post.append(result.speed['postprocess'])
        if len(result.boxes) > 0:
            frames_with_person += 1
            confidences.append(float(result.boxes.conf.max()))
        if n % 300 == 0:
            print(f"  {n}/{total} images...", flush=True)

    elapsed = time.perf_counter() - t_start
    measured = len(latencies)
    return {
        'frames': measured,
        'median_ms': float(np.median(latencies)),
        'p95_ms': float(np.percentile(latencies, 95)),
        'max_ms': float(np.max(latencies)),
        'pre_ms': float(np.median(pre)),
        'inference_ms': float(np.median(inf)),
        'post_ms': float(np.median(post)),
        'fps': measured / elapsed,
        'detection_rate': 100.0 * frames_with_person / measured,
        'mean_conf': float(np.mean(confidences)) if confidences else 0.0,
    }


def main():
    parser = argparse.ArgumentParser(description="Benchmark de YOLO sur une sequence video.")
    parser.add_argument('video', help="sequence de reference (ex. data/sequences/ref_01.mkv)")
    parser.add_argument('--model', required=True, help="modele .pt ou dossier OpenVINO")
    parser.add_argument('--imgsz', type=int, required=True)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--condition', default='idle', help="condition de charge (idle, gazebo...)")
    parser.add_argument('--csv', help="fichier CSV auquel ajouter une ligne")
    args = parser.parse_args()

    backend = 'openvino' if 'openvino' in args.model else 'pytorch'
    print(f"=== {backend} {args.imgsz} | condition : {args.condition} ===")
    r = run_benchmark(args.video, args.model, args.imgsz, args.conf)

    print(f"Images mesurees    : {r['frames']}")
    print(f"Latence detecteur  : mediane {r['median_ms']:.1f} ms | p95 {r['p95_ms']:.1f} ms"
          f" | max {r['max_ms']:.1f} ms")
    print(f"  dont pre/inf/post : {r['pre_ms']:.1f} / {r['inference_ms']:.1f} / {r['post_ms']:.1f} ms")
    print(f"Debit global       : {r['fps']:.1f} FPS (decodage video compris)")
    print(f"Detection          : {r['detection_rate']:.1f} % des images | confiance moyenne {r['mean_conf']:.2f}")

    if args.csv:
        row = {'condition': args.condition, 'backend': backend, 'imgsz': args.imgsz}
        row.update({k: round(v, 2) if isinstance(v, float) else v for k, v in r.items()})
        new_file = not os.path.exists(args.csv)
        with open(args.csv, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        print(f"Ligne ajoutee a {args.csv}")


if __name__ == '__main__':
    main()