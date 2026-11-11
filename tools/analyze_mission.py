#!/usr/bin/env python3
"""Analyse d'un vol de mission offboard_demo a partir d'un rosbag.

Usage (workspace source avec fm) :
    python3 tools/analyze_mission.py bags/mission_01
    python3 tools/analyze_mission.py bags/mission_01 --csv docs/campaign_phase2.csv
"""
import argparse
import csv
import math
import os
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from px4_msgs.msg import VehicleStatus
except ImportError as error:
    sys.exit(f"Import impossible ({error}). As-tu source le workspace (commande fm) ?")

POSITION_TOPIC = '/fmu/out/vehicle_local_position_v1'
STATUS_TOPIC = '/fmu/out/vehicle_status_v4'
SETPOINT_TOPIC = '/fmu/in/trajectory_setpoint'
REACHED_TOLERANCE = 0.3   # m, meme valeur que dans offboard_demo
MIN_TARGET_JUMP = 0.5     # m, un saut de consigne plus grand signale un nouveau point
MIN_LEG_LENGTH = 1.0      # m, en dessous ce n'est pas un cote du carre
EXPECTED_LEGS = 4


# ---------- Lecture du bag ----------
def read_bag(path):
    """Renvoie trois listes de tuples : positions, statuts, consignes."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    msg_types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    positions, statuses, setpoints = [], [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])
        t = t_ns * 1e-9   # nanosecondes -> secondes
        if topic == POSITION_TOPIC:
            positions.append((t, msg.x, msg.y, msg.z, math.hypot(msg.vx, msg.vy)))
        elif topic == STATUS_TOPIC:
            statuses.append((t, msg.arming_state, msg.nav_state))
        elif topic == SETPOINT_TOPIC:
            setpoints.append((t, msg.position[0], msg.position[1], msg.position[2]))
    return positions, statuses, setpoints


# ---------- Outils ----------
def arming_window(statuses):
    """Instants d'armement et de desarmement (None si absents)."""
    armed = VehicleStatus.ARMING_STATE_ARMED
    t_arm = next((t for t, state , _ in statuses if state == armed), None)
    if t_arm is None:
        return None, None
    t_disarm = next((t for t, state , _ in statuses if t > t_arm and state != armed), None)
    return t_arm, t_disarm


def position_at(positions, t):
    """Dernier echantillon de position avant l'instant t."""
    before = [p for p in positions if p[0] <= t]
    return before[-1] if before else positions[0]


def find_targets(setpoints):
    """Points cibles successifs : (instant d'apparition, x, y, z)."""
    targets = []
    for t, x, y, z in setpoints:
        if not targets or math.dist((x, y, z), targets[-1][1:]) > MIN_TARGET_JUMP:
            targets.append((t, x, y, z))
    return targets


# ---------- Analyse ----------
def analyse_leg(positions, previous, target, t_end):
    """Indicateurs d'un cote du carre, du point 'previous' au point 'target'."""
    _, x0, y0, _ = previous
    t1, x1, y1, z1 = target
    length = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / length, (y1 - y0) / length   # direction du deplacement

    window = [p for p in positions if t1 <= p[0] < t_end]
    if not window:
        return None

    along = [(px - x1) * ux + (py - y1) * uy for _, px, py, _, _ in window]
    cross = [abs((px - x0) * uy - (py - y0) * ux) for _, px, py, _, _ in window]
    reached = next((t for t, px, py, pz, _ in window
                    if math.dist((px, py, pz), (x1, y1, z1)) < REACHED_TOLERANCE), None)
    return {
        'duration': reached - t1 if reached is not None else None,
        'peak_speed': max(p[4] for p in window),
        'overshoot': max(0.0, max(along)),
        'cross_track': max(cross),
        'alt_dev': max(abs(p[3] - z1) for p in window),
    }


def analyse(path):
    positions, statuses, setpoints = read_bag(path)
    if not positions or not statuses or not setpoints:
        sys.exit("Bag incomplet : il faut les topics de position, de statut et de consignes.")

    t_arm, t_disarm = arming_window(statuses)
    if t_arm is None:
        sys.exit("Le drone n'a jamais ete arme pendant l'enregistrement.")
    home = position_at(positions, t_arm)
    end = position_at(positions, t_disarm) if t_disarm is not None else positions[-1]

    targets = find_targets(setpoints)
    land = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
    t_land = next((t for t, _, nav in statuses if t > t_arm and nav == land), None)
    takeoff_target = targets[0][1:]
    takeoff_reached = next((t for t, px, py, pz, _ in positions
                            if t >= t_arm and math.dist((px, py, pz), takeoff_target) < REACHED_TOLERANCE),
                           None)

    legs = []
    for i in range(1, len(targets)):
        prev, target = targets[i - 1], targets[i]
        if math.hypot(target[1] - prev[1], target[2] - prev[2]) < MIN_LEG_LENGTH:
            continue
        if i + 1 < len(targets):
            t_end = targets[i + 1][0]
        else:
            t_end = t_land if t_land is not None else setpoints[-1][0]
        leg = analyse_leg(positions, prev, target, t_end)
        if leg is not None:
            legs.append(leg)

    completed = (t_disarm is not None and len(legs) == EXPECTED_LEGS
                 and all(leg['duration'] is not None for leg in legs))
    return {
        'bag': os.path.basename(os.path.normpath(path)),
        'completed': completed,
        'mission_s': t_disarm - t_arm if t_disarm is not None else None,
        'takeoff_s': takeoff_reached - t_arm if takeoff_reached is not None else None,
        'legs': legs,
        'landing_offset_m': math.hypot(end[1] - home[1], end[2] - home[2]),
    }


# ---------- Sorties ----------
def fmt(value, unit, digits=2):
    return f"{value:.{digits}f} {unit}" if value is not None else "n/a"


def print_report(r):
    print(f"\n=== {r['bag']} ===")
    print(f"Mission complete       : {'OUI' if r['completed'] else 'NON'}")
    print(f"Arme -> desarme        : {fmt(r['mission_s'], 's', 1)}")
    print(f"Decollage (point 1)    : {fmt(r['takeoff_s'], 's', 1)}")
    print("Cote |  duree  |  v max   | depassement | ecart lat. | ecart alt.")
    for i, leg in enumerate(r['legs'], start=1):
        print(f"  {i}  | {fmt(leg['duration'], 's', 1):>7} | {fmt(leg['peak_speed'], 'm/s', 1):>8} | "
              f"{fmt(leg['overshoot'], 'm'):>11} | {fmt(leg['cross_track'], 'm'):>10} | "
              f"{fmt(leg['alt_dev'], 'm'):>10}")
    print(f"Ecart a l'atterrissage : {fmt(r['landing_offset_m'], 'm')}")


def append_csv(r, csv_path):
    def worst(key):
        values = [leg[key] for leg in r['legs'] if leg[key] is not None]
        return round(max(values), 2) if values else ''

    def rounded(value, digits):
        return round(value, digits) if value is not None else ''

    row = {
        'bag': r['bag'],
        'completed': r['completed'],
        'mission_s': rounded(r['mission_s'], 1),
        'takeoff_s': rounded(r['takeoff_s'], 1),
        'max_overshoot_m': worst('overshoot'),
        'max_cross_track_m': worst('cross_track'),
        'max_alt_dev_m': worst('alt_dev'),
        'max_speed_ms': worst('peak_speed'),
        'landing_offset_m': rounded(r['landing_offset_m'], 2),
    }
    new_file = not os.path.exists(csv_path)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(row))
        if new_file:
            writer.writeheader()
        writer.writerow(row)
    print(f"Ligne ajoutee a {csv_path}")


def main():
    parser = argparse.ArgumentParser(description="Analyse d'un vol de mission offboard_demo.")
    parser.add_argument('bag', help="dossier du rosbag (ex. bags/mission_01)")
    parser.add_argument('--csv', help="fichier CSV de campagne auquel ajouter une ligne")
    args = parser.parse_args()

    result = analyse(args.bag)
    print_report(result)
    if args.csv:
        append_csv(result, args.csv)


if __name__ == '__main__':
    main()