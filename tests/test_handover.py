from handover import Handover, MANUAL, HOLDING, DEGRADED, FAILSAFE, REANCHOR


def run(h, steps):
    """steps: list of (time, mode, quality). Returns all actions in order."""
    out = []
    for t, mode, q in steps:
        out += h.update(float(t), mode, q)
    return out


def test_manual_flight_vision_lost_never_lands():
    h = Handover()
    assert run(h, [(t, "STABILIZE", 0) for t in range(10)]) == []
    assert h.state == MANUAL


def test_engage_reanchors_once():
    h = Handover()
    acts = run(h, [(0, "STABILIZE", 80), (1, "LOITER", 80), (2, "LOITER", 80), (3, "LOITER", 80)])
    assert acts == [REANCHOR]
    assert h.state == HOLDING


def test_disengage_does_nothing():
    h = Handover()
    run(h, [(0, "LOITER", 80), (1, "LOITER", 80)])
    assert h.update(2.0, "STABILIZE", 80) == []
    assert h.state == MANUAL


def test_low_quality_is_degraded_not_failsafe():
    h = Handover()
    acts = run(h, [(0, "LOITER", 80)] + [(t, "LOITER", 20) for t in range(1, 10)])
    assert acts == [REANCHOR]
    assert h.state == DEGRADED


def test_lost_in_hold_fails_safe_once():
    h = Handover(failsafe_after_s=3.0)
    steps = [(0, "LOITER", 80)] + [(1 + i * 0.1, "LOITER", 0) for i in range(31)]
    steps += [(4.1 + i * 0.1, "LAND", 0) for i in range(30)]   # FC accepted LAND
    acts = run(h, steps)
    assert acts == [REANCHOR, ("SET_MODE", "LAND")]
    assert h.state == MANUAL


def test_failsafe_resends_if_ignored_then_stops():
    h = Handover(resend_s=1.0, max_sends=5)
    steps = [(0, "LOITER", 80)] + [(1 + i * 0.1, "LOITER", 0) for i in range(200)]
    acts = run(h, steps)
    assert acts.count(("SET_MODE", "LAND")) == 5


def test_short_loss_recovers_without_failsafe():
    h = Handover()
    acts = run(h, [(0, "LOITER", 80), (1, "LOITER", 0), (2, "LOITER", 0),
                   (3.5, "LOITER", 80), (5, "LOITER", 0), (7, "LOITER", 80)])
    assert acts == [REANCHOR]
    assert h.state == HOLDING


def test_pilot_takeover_during_failsafe():
    h = Handover()
    run(h, [(0, "LOITER", 80), (1, "LOITER", 0), (5, "LOITER", 0)])
    assert h.state == FAILSAFE
    assert run(h, [(6, "ALT_HOLD", 0), (7, "ALT_HOLD", 0), (20, "ALT_HOLD", 0)]) == []
    assert h.state == MANUAL


def test_failsafe_none_never_sets_mode():
    h = Handover(failsafe_mode="NONE")
    acts = run(h, [(0, "LOITER", 80)] + [(t, "LOITER", 0) for t in range(1, 20)])
    assert acts == [REANCHOR]


def test_reengage_reanchors_again():
    h = Handover()
    acts = run(h, [(0, "LOITER", 80), (1, "STABILIZE", 80), (2, "POSHOLD", 80)])
    assert acts == [REANCHOR, REANCHOR]