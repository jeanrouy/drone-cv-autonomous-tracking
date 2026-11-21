"""Selection et verrouillage d'une cible parmi les pistes du tracker.

Module sans dependance ROS : testable hors ligne et par des tests unitaires.
Les etats reprennent les constantes du message fm_interfaces/TargetState.

Regles (v4) :
- verrouillage UNIQUEMENT sur demande de l'operateur (request_lock) : jamais de verrouillage
  ni de reacquisition automatiques ;
- une fois verrouille, on garde l'identifiant, quelles que soient les autres pistes ;
- anti-rebond : une absence plus courte que lost_grace ne fait pas passer en LOST ;
- R1 : une piste qui chevauchait la cible au debut de son absence ne peut pas la remplacer ;
- R2 : une piste ne remplace la cible que si sa hauteur est comparable ;
- R3 : cible perdue en touchant un bord lateral : pas de reprise automatique ;
- R4 : saut brusque de taille et de position de la piste verrouillee : identite rompue ;
- perte au-dela de max_lost_time : retour en SEARCHING, en attente d'une nouvelle designation.
"""
import math
from dataclasses import dataclass, replace

SEARCHING, LOCKED, LOST = 0, 1, 2
STATE_NAMES = {SEARCHING: 'SEARCHING', LOCKED: 'LOCKED', LOST: 'LOST'}
CENTER = 'center'   # demande de verrouillage sur la piste la plus proche du centre


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


# ---------- Geometrie des boites (tout objet ayant u, v, width, height) ----------

def distance(u1, v1, u2, v2):
    return math.hypot(u1 - u2, v1 - v2)


def top(box):
    """Ordonnee du haut de la boite (la tete pour une personne debout)."""
    return box.v - box.height / 2


def size_ratio(h1, h2):
    """Rapport >= 1 entre deux hauteurs (1 = meme taille)."""
    small, large = sorted((h1, h2))
    return large / small if small > 0 else math.inf


def overlaps(a, b):
    """Vrai si les boites a et b se chevauchent."""
    return abs(a.u - b.u) * 2 < a.width + b.width and abs(a.v - b.v) * 2 < a.height + b.height


def touches_lateral_border(box, margin):
    return box.u - box.width / 2 < margin or box.u + box.width / 2 > 1 - margin


def touches_border(box, margin):
    return (touches_lateral_border(box, margin)
            or top(box) < margin or box.v + box.height / 2 > 1 - margin)


# ---------- Selecteur ----------

class TargetSelector:
    def __init__(self, relock_radius=0.15, lost_grace=0.35, relock_delay=0.5, max_lost_time=2.0,
                 max_height_ratio=1.4, jump_height_ratio=1.5, jump_top=0.1, border_margin=0.02):
        self.relock_radius = relock_radius          # distance max (normalisee) pour reprendre une piste
        self.lost_grace = lost_grace                # s : absence toleree avant LOST (anti-rebond)
        self.relock_delay = relock_delay            # s : d'abord laisser le tracker retrouver le meme id
        self.max_lost_time = max_lost_time          # s : au-dela, retour en SEARCHING
        self.max_height_ratio = max_height_ratio    # R2 : ecart de taille tolere pour une reprise
        self.jump_height_ratio = jump_height_ratio  # R4 : changement de taille juge brusque
        self.jump_top = jump_top                    # R4 : deplacement du haut de boite juge brusque
        self.border_margin = border_margin
        self.events = []                            # journal des transitions : (instant, texte)
        self._reset()

    def request_lock(self, track_id=None):
        """Demande de l'operateur : verrouiller la piste track_id, ou la plus centrale si None.

        La demande reste en attente jusqu'a ce qu'une piste correspondante soit visible.
        """
        self._lock_request = CENTER if track_id is None else track_id

    def update(self, t, tracks):
        """Met a jour la selection avec les pistes de l'image prise a l'instant t (en s)."""
        tracks = list(tracks)

        # 0. Designation par l'operateur : prioritaire, dans tous les etats
        if self._lock_request is not None:
            chosen = self._requested_track(tracks)
            if chosen is not None:
                how = ("plus proche du centre" if self._lock_request == CENTER
                       else "designee par son identifiant")
                self._lock_request = None
                self._set_locked(t, chosen, tracks)
                self._log(t, f"verrouillage sur demande de l'operateur : piste {chosen.track_id} ({how})")
                return replace(self._target)

        # 1. Aucune cible : on attend une designation (jamais de verrouillage automatique)
        if self._target.state == SEARCHING:
            return replace(self._target)

        # 2. La piste verrouillee est-elle presente, et designe-t-elle toujours la meme personne ?
        locked = next((tr for tr in tracks if tr.track_id == self._target.track_id), None)
        if locked is not None and locked.track_id in self._excluded:
            locked = None   # identite deja declaree rompue pendant cet episode
        identity_broken = False
        if locked is not None:
            if self._identity_jump(locked):                               # R4
                identity_broken = True
                self._excluded.add(locked.track_id)
                self._log(t, f"saut de taille et de position de la piste {locked.track_id} : "
                             f"identite consideree comme rompue")
            elif self._target.state == LOST and self._exited_side:
                pass   # R3 : sortie par le bord, le meme id peut designer une autre personne
            else:
                if self._target.state == LOST:
                    self._log(t, f"piste {locked.track_id} retrouvee apres {t - self._last_seen:.2f} s")
                self._set_locked(t, locked, tracks)
                return replace(self._target)

        # 3. La cible n'est pas (ou plus) observee dans cette image
        self._target.time_since_seen = t - self._last_seen
        if self._target.time_since_seen <= self.lost_grace or identity_broken:
            # R1 : au debut de l'absence, ce qui chevauche la cible est probablement ce qui la masque
            self._excluded |= self._near_ids | {tr.track_id for tr in tracks
                                                if tr.track_id != self._target.track_id
                                                and overlaps(tr, self._target)}

        if self._target.state == LOCKED:
            if self._target.time_since_seen < self.lost_grace and not identity_broken:
                return replace(self._target)   # absence breve : on reste LOCKED sur la derniere position
            self._target.state = LOST
            self._exited_side = touches_lateral_border(self._target, self.border_margin)   # R3
            reason = " (sortie par un bord lateral : pas de reprise automatique)" if self._exited_side else ""
            self._log(t, f"piste {self._target.track_id} perdue{reason}")

        self.diagnostics = [(tr.track_id,
                             distance(tr.u, tr.v, self._target.u, self._target.v),
                             size_ratio(tr.height, self._target.height),
                             tr.track_id in self._excluded) for tr in tracks]

        candidates = []
        if not self._exited_side and self._target.time_since_seen >= self.relock_delay:
            candidates = [tr for tr in tracks
                          if tr.track_id not in self._excluded                                     # R1
                          and distance(tr.u, tr.v, self._target.u, self._target.v) <= self.relock_radius
                          and size_ratio(tr.height, self._target.height) <= self.max_height_ratio]  # R2
        if candidates:
            new = min(candidates, key=lambda tr: distance(tr.u, tr.v, self._target.u, self._target.v))
            self._log(t, f"reverrouillage sur la piste {new.track_id} pres de la derniere position "
                         f"(apres {self._target.time_since_seen:.2f} s)")
            self._set_locked(t, new, tracks)
        elif self._target.time_since_seen > self.max_lost_time:
            self._log(t, f"cible perdue depuis plus de {self.max_lost_time:.1f} s : "
                         f"en attente d'une designation par l'operateur")
            self._reset()
        return replace(self._target)

    def _requested_track(self, tracks):
        if not tracks:
            return None
        if self._lock_request == CENTER:
            return min(tracks, key=lambda tr: distance(tr.u, tr.v, 0.5, 0.5))
        return next((tr for tr in tracks if tr.track_id == self._lock_request), None)

    def _identity_jump(self, track):
        """R4 : taille ET haut de boite changent brusquement (une occultation du bas ne suffit pas)."""
        return (size_ratio(track.height, self._target.height) > self.jump_height_ratio
                and abs(top(track) - top(self._target)) > self.jump_top)

    def _set_locked(self, t, track, tracks):
        self._target = Target(state=LOCKED, track_id=track.track_id, u=track.u, v=track.v,
                              width=track.width, height=track.height, confidence=track.confidence,
                              touches_border=touches_border(track, self.border_margin),
                              time_since_seen=0.0)
        self._last_seen = t
        self._near_ids = {tr.track_id for tr in tracks
                          if tr.track_id != track.track_id and overlaps(tr, track)}
        self._excluded = set()
        self._exited_side = False

    def _reset(self):
        self._target = Target()
        self._last_seen = None
        self._near_ids = set()       # pistes qui chevauchaient la cible a sa derniere observation
        self._excluded = set()       # pistes interdites de reprise pendant l'episode de perte en cours
        self._exited_side = False    # cible perdue en touchant un bord lateral
        self._lock_request = None    # demande de l'operateur en attente : None, CENTER ou un identifiant
        self.diagnostics = []        # en LOST : (id, distance, rapport de taille, exclue) par piste

    def _log(self, t, text):
        self.events.append((t, text))