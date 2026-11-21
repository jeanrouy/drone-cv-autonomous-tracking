import pytest

from fm_perception.target_selector import LOCKED, LOST, SEARCHING, TargetSelector, Track


def track(track_id, u, v, w=0.1, h=0.4, conf=0.9):
    return Track(track_id, u, v, w, h, conf)


def armed_selector(**params):
    """Selecteur pour lequel l'operateur a demande un verrouillage sur la piste la plus centrale."""
    selector = TargetSelector(**params)
    selector.request_lock()
    return selector


# ---------- Designation par l'operateur (v4) ----------

def test_aucun_verrouillage_sans_demande_de_l_operateur():
    selector = TargetSelector()
    assert selector.update(0.0, [track(1, 0.5, 0.5)]).state == SEARCHING


def test_la_demande_reste_en_attente_jusqu_a_ce_qu_une_piste_apparaisse():
    selector = armed_selector()
    assert selector.update(0.0, []).state == SEARCHING
    assert selector.update(0.1, [track(1, 0.5, 0.5)]).state == LOCKED


def test_verrouille_la_piste_designee_par_son_identifiant():
    selector = TargetSelector()
    selector.request_lock(2)
    target = selector.update(0.0, [track(1, 0.5, 0.5), track(2, 0.2, 0.5)])
    assert (target.state, target.track_id) == (LOCKED, 2)


def test_premier_verrouillage_sur_la_piste_la_plus_proche_du_centre():
    selector = armed_selector()
    target = selector.update(0.0, [track(1, 0.2, 0.5), track(2, 0.55, 0.5)])
    assert (target.state, target.track_id) == (LOCKED, 2)


# ---------- Suivi et reprise (v1 a v3) ----------

def test_ne_change_pas_de_cible_si_une_piste_plus_centrale_apparait():
    selector = armed_selector()
    selector.update(0.0, [track(1, 0.3, 0.5)])
    target = selector.update(0.1, [track(1, 0.3, 0.5), track(2, 0.5, 0.5)])
    assert target.track_id == 1


def test_une_absence_breve_ne_declenche_pas_lost():
    selector = armed_selector(lost_grace=0.35)
    selector.update(0.0, [track(1, 0.5, 0.5)])
    target = selector.update(0.2, [])
    assert (target.state, target.track_id) == (LOCKED, 1)
    assert target.time_since_seen == pytest.approx(0.2)


def test_retrouve_la_meme_piste_apres_une_perte():
    selector = armed_selector(lost_grace=0.35)
    selector.update(0.0, [track(1, 0.5, 0.5)])
    assert selector.update(0.5, []).state == LOST
    target = selector.update(0.6, [track(1, 0.52, 0.5)])
    assert (target.state, target.track_id) == (LOCKED, 1)


def test_reverrouille_une_piste_voisine_seulement_apres_le_delai():
    selector = armed_selector(relock_radius=0.15, lost_grace=0.2, relock_delay=0.4)
    selector.update(0.0, [track(1, 0.5, 0.5)])
    assert selector.update(0.3, [track(7, 0.62, 0.5)]).state == LOST   # perdue, mais trop tot
    target = selector.update(0.5, [track(7, 0.62, 0.5)])
    assert (target.state, target.track_id) == (LOCKED, 7)


def test_apres_une_perte_definitive_attend_l_operateur():
    selector = armed_selector(relock_radius=0.15, max_lost_time=2.0)
    selector.update(0.0, [track(1, 0.2, 0.5)])
    assert selector.update(1.0, [track(9, 0.8, 0.5)]).state == LOST
    assert selector.update(2.5, [track(9, 0.8, 0.5)]).state == SEARCHING
    assert selector.update(3.0, [track(9, 0.5, 0.5)]).state == SEARCHING   # pas de reacquisition


def test_signale_une_boite_au_bord_de_l_image():
    selector = armed_selector(border_margin=0.02)
    target = selector.update(0.0, [track(1, 0.97, 0.5, w=0.08)])
    assert target.touches_border


# ---------- Cas d'echec de l'essai E8 (v3) ----------

def test_r1_n_accroche_pas_la_piste_qui_masquait_la_cible():
    # pub_01, 8.4 s : la piste 6 passe devant la cible puis reste a cote
    selector = armed_selector(lost_grace=0.35, relock_delay=0.5)
    selector.update(0.0, [track(1, 0.50, 0.5), track(6, 0.80, 0.5)])   # 1 est la plus centrale
    selector.update(0.1, [track(1, 0.50, 0.5), track(6, 0.58, 0.5)])   # 6 chevauche la cible
    selector.update(0.3, [track(6, 0.52, 0.5)])                          # 6 masque la cible
    target = selector.update(0.8, [track(6, 0.46, 0.5)])                 # meme taille, tout pres
    assert target.state == LOST


def test_r2_ne_reprend_pas_une_piste_de_taille_tres_differente():
    # pub_01, 8.4 s : premier plan contre arriere-plan
    selector = armed_selector(lost_grace=0.2, relock_delay=0.4)
    selector.update(0.0, [track(1, 0.5, 0.5, h=0.3)])
    assert selector.update(0.5, [track(7, 0.62, 0.5, h=0.6)]).state == LOST
    target = selector.update(0.6, [track(8, 0.62, 0.5, h=0.33)])
    assert (target.state, target.track_id) == (LOCKED, 8)


def test_r3_pas_de_reprise_apres_une_sortie_par_le_bord():
    # pub_01, 10.5 s : sortie a gauche, une autre personne entre a gauche
    selector = armed_selector(lost_grace=0.2, relock_delay=0.4, max_lost_time=2.0)
    selector.update(0.0, [track(1, 0.04, 0.5)])                          # touche le bord gauche
    assert selector.update(0.5, []).state == LOST
    assert selector.update(0.8, [track(1, 0.05, 0.5)]).state == LOST     # meme id : refuse
    assert selector.update(1.0, [track(9, 0.06, 0.5)]).state == LOST     # autre id : refuse
    assert selector.update(2.5, [track(9, 0.06, 0.5)]).state == SEARCHING


def test_r4_un_saut_de_taille_et_de_position_rompt_l_identite():
    # echange d'identifiants par le tracker : meme id, autre personne
    selector = armed_selector()
    selector.update(0.0, [track(5, 0.5, 0.5, h=0.3)])            # haut de la boite a 0.35
    target = selector.update(0.1, [track(5, 0.5, 0.5, h=0.6)])   # haut a 0.20 : la tete a saute
    assert target.state == LOST


def test_r4_une_occultation_partielle_ne_rompt_pas_l_identite():
    # ref_01, 47 s : le bas de la silhouette est cache, la tete ne bouge pas
    selector = armed_selector()
    selector.update(0.0, [track(1, 0.5, 0.5, h=0.3)])              # haut a 0.35
    target = selector.update(0.1, [track(1, 0.5, 0.425, h=0.15)])  # haut toujours a 0.35
    assert (target.state, target.track_id) == (LOCKED, 1)