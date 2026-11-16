"""Selection et verrouillage d'une cible parmi les pistes du tracker.

Module sans dependance ROS : testable hors ligne et par des tests unitaires.
Les etats reprennent les constantes du message fm_interfaces/TargetState.
"""
import math
from dataclasses import dataclass, replace

SEARCHING, LOCKED, LOST = 0, 1, 2
STATE_NAMES = {SEARCHING: 'SEARCHING', LOCKED: 'LOCKED', LOST: 'LOST'}


@dataclass(frozen=True)
class Track:
    """Une piste du tracker, coordonnees normalisees entre 0 et 1."""
    track_id: int
    u: float
    v: float
    width: float
    height: float
    confidence: float


@dataclass
class Target:
    """Cible courante : memes champs que le message TargetState (hors en-tete)."""
    state: int = SEARCHING
    track_id: int = -1
    u: float = 0.0
    v: float = 0.0
    width: float = 0.0
    height: float = 0.0
    confidence: float = 0.0
    touches_border: bool = False
    time_since_seen: float = 0.0


def distance(u1, v1, u2, v2):
    return math.hypot(u1 - u2, v1 - v2)


def touches_border(track, margin):
    """Vrai si la boite touche un bord de l'image (sa hauteur ne reflete plus la distance)."""
    return (track.u - track.width / 2 < margin or track.u + track.width / 2 > 1 - margin
            or track.v - track.height / 2 < margin or track.v + track.height / 2 > 1 - margin)


class TargetSelector:
    def __init__(self, relock_radius=0.15, relock_delay=0.3, max_lost_time=2.0, border_margin=0.02):
        self.relock_radius = relock_radius    # distance max (normalisee) pour reprendre une autre piste
        self.relock_delay = relock_delay      # s : d'abord laisser le tracker retrouver le meme id
        self.max_lost_time = max_lost_time    # s : au-dela, nouvelle recherche
        self.border_margin = border_margin
        self._target = Target()
        self._last_seen = None                # instant (s) de la derniere observation de la cible
        self.events = []                      # journal des transitions : (instant, texte)

    def update(self, t, tracks):
        """Met a jour la selection avec les pistes de l'image prise a l'instant t (en s)."""
        tracks = list(tracks)

        # 1. Aucune cible : premier verrouillage sur la piste la plus proche du centre
        if self._target.state == SEARCHING:
            if tracks:
                first = min(tracks, key=lambda tr: distance(tr.u, tr.v, 0.5, 0.5))
                self._set_locked(t, first)
                self._log(t, f"verrouillage initial sur la piste {first.track_id} (plus proche du centre)")
            return replace(self._target)

        # 2. La piste verrouillee est presente : on la garde, quelles que soient les autres
        locked = next((tr for tr in tracks if tr.track_id == self._target.track_id), None)
        if locked is not None:
            if self._target.state == LOST:
                self._log(t, f"piste {locked.track_id} retrouvee apres {t - self._last_seen:.2f} s")
            self._set_locked(t, locked)
            return replace(self._target)

        # 3. La piste verrouillee est absente de cette image
        if self._target.state == LOCKED:
            self._target.state = LOST
            self._log(t, f"piste {self._target.track_id} perdue")
        self._target.time_since_seen = t - self._last_seen

        candidates = []
        if self._target.time_since_seen >= self.relock_delay:
            candidates = [tr for tr in tracks
                          if distance(tr.u, tr.v, self._target.u, self._target.v) <= self.relock_radius]
        if candidates:
            new = min(candidates, key=lambda tr: distance(tr.u, tr.v, self._target.u, self._target.v))
            self._log(t, f"reverrouillage sur la piste {new.track_id} pres de la derniere position "
                         f"(apres {self._target.time_since_seen:.2f} s)")
            self._set_locked(t, new)
        elif self._target.time_since_seen > self.max_lost_time:
            self._log(t, f"cible perdue depuis plus de {self.max_lost_time:.1f} s : nouvelle recherche")
            self._target = Target()
        return replace(self._target)

    def _set_locked(self, t, track):
        self._target = Target(state=LOCKED, track_id=track.track_id, u=track.u, v=track.v,
                              width=track.width, height=track.height, confidence=track.confidence,
                              touches_border=touches_border(track, self.border_margin),
                              time_since_seen=0.0)
        self._last_seen = t

    def _log(self, t, text):
        self.events.append((t, text))