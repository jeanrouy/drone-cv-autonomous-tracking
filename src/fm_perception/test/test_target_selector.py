from fm_perception.target_selector import LOCKED, LOST, SEARCHING, TargetSelector, Track


def track(track_id, u, v, w=0.1, h=0.4, conf=0.9):
    return Track(track_id, u, v, w, h, conf)


def test_premier_verrouillage_sur_la_piste_la_plus_proche_du_centre():
    selector = TargetSelector()
    target = selector.update(0.0, [track(1, 0.2, 0.5), track(2, 0.55, 0.5)])
    assert target.state == LOCKED
    assert target.track_id == 2


def test_ne_change_pas_de_cible_si_une_piste_plus_centrale_apparait():
    selector = TargetSelector()
    selector.update(0.0, [track(1, 0.3, 0.5)])
    target = selector.update(0.1, [track(1, 0.3, 0.5), track(2, 0.5, 0.5)])
    assert target.track_id == 1


def test_retrouve_la_meme_piste_apres_une_courte_perte():
    selector = TargetSelector()
    selector.update(0.0, [track(1, 0.5, 0.5)])
    assert selector.update(0.1, []).state == LOST
    target = selector.update(0.2, [track(1, 0.52, 0.5)])
    assert (target.state, target.track_id) == (LOCKED, 1)


def test_reverrouille_une_piste_voisine_seulement_apres_le_delai():
    selector = TargetSelector(relock_radius=0.15, relock_delay=0.3)
    selector.update(0.0, [track(1, 0.5, 0.5)])
    assert selector.update(0.1, [track(7, 0.55, 0.5)]).state == LOST   # trop tot
    target = selector.update(0.4, [track(7, 0.55, 0.5)])
    assert (target.state, target.track_id) == (LOCKED, 7)


def test_ignore_une_piste_eloignee_puis_relance_la_recherche():
    selector = TargetSelector(relock_radius=0.15, max_lost_time=2.0)
    selector.update(0.0, [track(1, 0.2, 0.5)])
    assert selector.update(1.0, [track(9, 0.8, 0.5)]).state == LOST
    assert selector.update(2.5, [track(9, 0.8, 0.5)]).state == SEARCHING


def test_signale_une_boite_au_bord_de_l_image():
    selector = TargetSelector(border_margin=0.02)
    target = selector.update(0.0, [track(1, 0.97, 0.5, w=0.08)])
    assert target.touches_border