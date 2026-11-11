#!/usr/bin/env python3
"""Analyse de l'essai E2 (perte du flux Offboard) : rosbag + log ULog de PX4.

Usage (workspace source avec fm) :
    python3 tools/analyze_failsafe.py bags/e2_01 --ulog <fichier.ulg>
"""
import argparse
import math
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
HEARTBEAT_TOPIC = '/fmu/in/offboard_control_mode'

COM_OF_LOSS_T = 1.0                      # s, valeur du parametre PX4 pendant l'essai
MAX_EXIT_DELAY = COM_OF_LOSS_T + 0.5     # critere E2-a
MAX_ALT_LOSS = 0.5                       # m, critere E2-b
ALT_WINDOW = 5.0                         # s apres la coupure, pour E2-b

OFFBOARD = VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMED = VehicleStatus.ARMING_STATE_ARMED
NAV_NAMES = {
    VehicleStatus.NAVIGATION_STATE_POSCTL: 'Position',
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: 'Hold',
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL: 'Return',
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: 'Land',
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: 'Offboard',
}


def nav_name(nav):
    return NAV_NAMES.get(int(nav), f"code {int(nav)}")


def fmt(value, unit, digits=2):
    if value is None:
        return "n/a"
    if math.isnan(value):
        return "incoherent"
    return f"{value:.{digits}f} {unit}"


def verdict(ok):
    return 'PASS' if ok else 'FAIL'


# ---------- Source 1 : rosbag ----------
def read_topics(path, topics):
    """Renvoie {topic: [(instant en s, message), ...]}."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    msg_types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    data = {topic: [] for topic in topics}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        data[topic].append((t_ns * 1e-9, deserialize_message(raw, msg_types[topic])))
    return data


def analyse_bag(path):
    data = read_topics(path, [POSITION_TOPIC, STATUS_TOPIC, HEARTBEAT_TOPIC])
    positions, statuses, heartbeats = data[POSITION_TOPIC], data[STATUS_TOPIC], data[HEARTBEAT_TOPIC]
    if not positions or not statuses or not heartbeats:
        sys.exit("Bag incomplet : il faut la position, le statut et le heartbeat Offboard.")

    t_cut = heartbeats[-1][0]   # dernier heartbeat enregistre = instant de la coupure
    t_arm = next((t for t, m in statuses if m.arming_state == ARMED), None)
    exit_ = next(((t, m.nav_state) for t, m in statuses
                  if t > t_cut and m.nav_state != OFFBOARD and m.arming_state == ARMED), None)
    t_disarm = next((t for t, m in statuses if t > t_cut and m.arming_state != ARMED), None)

    def position_at(t):
        before = [m for tp, m in positions if tp <= t]
        return before[-1] if before else positions[0][1]

    home = position_at(t_arm) if t_arm is not None else positions[0][1]

    def altitude(m):
        return home.z - m.z   # NED : z vers le bas

    alt_cut = altitude(position_at(t_cut))
    after_cut = [(t, m) for t, m in positions if t >= t_cut]
    window = [m for t, m in after_cut if t <= t_cut + ALT_WINDOW]
    end = position_at(t_disarm) if t_disarm is not None else positions[-1][1]

    return {
        'exit_delay': exit_[0] - t_cut if exit_ else None,
        'new_mode': nav_name(exit_[1]) if exit_ else 'aucun',
        'alt_cut': alt_cut,
        'alt_loss': max(0.0, alt_cut - min(altitude(m) for m in window)),
        'alt_max': max(altitude(m) for _, m in after_cut),
        'disarmed': t_disarm is not None,
        'time_to_disarm': t_disarm - t_cut if t_disarm is not None else None,
        'landing_offset': math.hypot(end.x - home.x, end.y - home.y),
    }


# ---------- Source 2 : log ULog de PX4 ----------
def analyse_ulog(path):
    try:
        from pyulog import ULog
    except ImportError:
        print("pyulog absent : pip3 install --user pyulog")
        return None
    ulog = ULog(path, ['vehicle_status', 'failsafe_flags', 'offboard_control_mode'])

    def get(name):
        try:
            return ulog.get_dataset(name).data
        except (IndexError, KeyError, ValueError):
            print(f"  (topic '{name}' absent du log)")
            return None

    status, flags, heartbeat = get('vehicle_status'), get('failsafe_flags'), get('offboard_control_mode')
    if status is None:
        return None

    ts, nav = status['timestamp'], status['nav_state']
    t_exit = next((ts[i] for i in range(1, len(nav))
                   if nav[i - 1] == OFFBOARD and nav[i] != OFFBOARD), None)
    result = {'t_exit': t_exit, 't_lost': None, 't_last_hb': None}

    if flags is not None and t_exit is not None:
        lost, fts = flags['offboard_control_signal_lost'], flags['timestamp']
        rises = [fts[i] for i in range(1, len(lost))
                 if not lost[i - 1] and lost[i] and fts[i] <= t_exit]
        result['t_lost'] = rises[-1] if rises else None

    if heartbeat is not None and result['t_lost'] is not None:
        before = [t for t in heartbeat['timestamp'] if t <= result['t_lost']]
        result['t_last_hb'] = before[-1] if before else None
    return result


def delta_s(later, earlier):
    """Ecart en secondes entre deux horodatages PX4 (microsecondes)."""
    if later is None or earlier is None:
        return None
    d = (int(later) - int(earlier)) * 1e-6
    return d if 0 <= d < 10 else float('nan')


# ---------- Rapport ----------
def print_report(bag, ulog):
    print("\n=== Essai E2 : perte du flux Offboard ===")
    print("Source 1 : rosbag (instants de reception cote ROS 2)")
    print(f"  Coupure -> sortie d'Offboard : {fmt(bag['exit_delay'], 's')}"
          f"   (nouveau mode : {bag['new_mode']})")
    print(f"  Altitude a la coupure : {bag['alt_cut']:.2f} m | perte max sur {ALT_WINDOW:.0f} s :"
          f" {bag['alt_loss']:.2f} m | altitude max ensuite : {bag['alt_max']:.2f} m")
    print(f"  Desarme : {'oui' if bag['disarmed'] else 'non'}"
          f" ({fmt(bag['time_to_disarm'], 's', 1)} apres la coupure)"
          f" | ecart au point de depart : {bag['landing_offset']:.2f} m")

    if ulog is not None:
        detection = delta_s(ulog['t_lost'], ulog['t_last_hb'])
        reaction = delta_s(ulog['t_exit'], ulog['t_lost'])
        print("Source 2 : log ULog (horloge interne de PX4)")
        print(f"  Dernier heartbeat -> perte detectee : {fmt(detection, 's')}"
              f"   (attendu ~ COM_OF_LOSS_T = {COM_OF_LOSS_T} s)")
        print(f"  Perte detectee -> changement de mode : {fmt(reaction, 's')}")

    print("Criteres :")
    exit_ok = bag['exit_delay'] is not None and bag['exit_delay'] <= MAX_EXIT_DELAY
    print(f"  E2-a sortie d'Offboard <= {MAX_EXIT_DELAY} s : {verdict(exit_ok)}")
    print(f"  E2-b perte d'altitude < {MAX_ALT_LOSS} m   : {verdict(bag['alt_loss'] < MAX_ALT_LOSS)}")
    print(f"  E2-c atterri et desarme sans operateur : {verdict(bag['disarmed'])}")


def main():
    parser = argparse.ArgumentParser(description="Analyse de l'essai E2 (perte du flux Offboard).")
    parser.add_argument('bag', help="dossier du rosbag (ex. bags/e2_01)")
    parser.add_argument('--ulog', help="log de vol PX4 (.ulg) de l'essai")
    args = parser.parse_args()

    bag = analyse_bag(args.bag)
    ulog = analyse_ulog(args.ulog) if args.ulog else None
    print_report(bag, ulog)


if __name__ == '__main__':
    main()