import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.planner import recommend, get_crop, known_crops, day_detail, nokku_for, biodynamic_for


def test_nakshatra_classification_real_dates():
    # From the pulled panchangam cache + traditional classification
    assert nokku_for("2026-09-03") == "keezh"      # Krittika
    assert nokku_for("2026-08-18") == "sama"       # Swati (verified vs drikpanchang)
    assert biodynamic_for("2026-09-03") == "root"  # biodynamic root day


def test_groundnut_dual_recommendation_is_sep3():
    rec = recommend("groundnut", after="2026-08-10", horizon_days=40)
    assert rec.recommended_date == "2026-09-03"    # only date favourable in BOTH systems
    assert rec.both_agree is True
    assert rec.nokku == "keezh" and rec.biodynamic == "root"
    assert rec.nakshatra == "Krittika"


def test_recommended_date_is_not_an_avoid_day():
    from farmos.planner import is_avoid
    rec = recommend("groundnut", after="2026-08-10")
    assert not is_avoid(rec.recommended_date)


def test_aug13_keezh_but_rejected_as_avoid_and_not_biodynamic_root():
    # Aug 13 is Keezh (Ashlesha) but a கரி நாள் avoid day AND not a biodynamic root day
    d = day_detail("2026-08-13")
    assert d.nokku == "keezh"
    assert d.is_avoid is True
    assert d.biodynamic != "root"


def test_to_seed_plan_carries_crop_geometry():
    rec = recommend("groundnut", after="2026-08-10")
    sp = rec.to_seed_plan(plot_w_m=1.2, plot_l_m=2.4)
    assert sp.crop == "groundnut"
    assert sp.row_gap_m == 0.30 and sp.seed_gap_m == 0.15
    assert sp.recommended_date == "2026-09-03"


def test_unknown_crop_raises():
    try:
        get_crop("banana")
        assert False, "expected KeyError"
    except KeyError:
        pass
    assert "groundnut" in known_crops()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print(f"ok  {name}")
    print("all planner tests passed")
