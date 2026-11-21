from handover import AutoReanchor, Handover, MANUAL, HOLDING, DEGRADED, FAILSAFE, REANCHOR


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


def test_stale_mode_never_sends_set_mode_until_fresh():
    h = Handover(failsafe_after_s=3.0)
    h.update(0.0, "LOITER", 80)
    # vision lost from 1.0 s to 5.9 s while the autopilot heartbeat is stale: no SET_MODE
    for i in range(10, 60):
        assert h.update(i * 0.1, "LOITER", 0, mode_fresh=False) == []
    assert h.state == FAILSAFE
    # heartbeat fresh again: the first SET_MODE is sent now
    assert h.update(6.0, "LOITER", 0, mode_fresh=True) == [("SET_MODE", "LAND")]


def test_gave_up_once_after_max_sends_ignored():
    h = Handover(resend_s=1.0, max_sends=5)
    steps = [(0, "LOITER", 80)] + [(1 + i * 0.1, "LOITER", 0) for i in range(200)]
    acts = run(h, steps)
    assert acts.count(("SET_MODE", "LAND")) == 5
    assert acts.count(("GAVE_UP",)) == 1


def test_auto_reanchor_fires_after_delay_and_retries():
    a = AutoReanchor(2.0)
    assert [a.update(t, False, True) for t in (0.0, 1.0, 1.9)] == [False, False, False]
    assert a.update(2.0, False, True) is True            # 2 s lost, far from home's scale: save a new home
    assert a.update(3.0, False, True) is False           # timer restarted
    assert a.update(4.0, False, True) is True            # still lost: try again


def test_auto_reanchor_never_acts_while_engaged():
    a = AutoReanchor(2.0)
    assert not any(a.update(t, True, True) for t in range(0, 30))


def test_auto_reanchor_resets_when_vision_returns():
    a = AutoReanchor(2.0)
    a.update(0.0, False, True)
    a.update(1.5, False, False)                          # vision came back: timer cleared
    assert a.update(2.5, False, True) is False           # a new 2 s starts here
    assert a.update(4.5, False, True) is True


def test_auto_reanchor_off_when_zero():
    a = AutoReanchor(0)
    assert not any(a.update(t, False, True) for t in range(0, 30))


if __name__ == "__main__":
    # tests.run_all and deploy/install.sh run every file as a script. Without this block they would run ZERO tests
    # here (pytest finds the test_ functions by itself, a plain `python -m` does not).
    names = sorted(n for n, f in list(globals().items()) if n.startswith("test_") and callable(f))
    for n in names:
        globals()[n]()
        print("  ok", n)
    print(f"\nALL HANDOVER TESTS PASSED ({len(names)} tests)")
