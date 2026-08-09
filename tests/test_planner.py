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


def test_survey_returns_all_buckets():
    from farmos.planner import survey
    s = survey("groundnut", after="2026-08-10", horizon_days=40)
    assert s["needs"] == {"nokku": "keezh", "biodynamic": "root"}
    assert "2026-09-03" in [r["date"] for r in s["recommended_both_systems"]]
    assert "2026-08-13" in s["avoid_days_kari_naal"]                 # Keezh but avoid -> excluded
    # single-system alternatives exist and are disjoint from the both-systems list
    rec_dates = {r["date"] for r in s["recommended_both_systems"]}
    p_only = {r["date"] for r in s["panchangam_only"]}
    b_only = {r["date"] for r in s["biodynamic_only"]}
    assert rec_dates.isdisjoint(p_only) and rec_dates.isdisjoint(b_only)
    assert "2026-08-15" in b_only   # Aug 15 is a biodynamic root day but not Keezh Nokku


def test_price_summary_mock():
    from farmos.planner import price_summary
    p = price_summary("groundnut")
    assert p["mock"] is True                       # loudly flagged as mock
    assert p["current"]["month"] == "2026-08" and p["current"]["price"] > 0
    assert p["history_span"][0] == "2023-01"       # 3+ years of monthly history
    assert len(p["recent_12_months"]) == 12
    assert p["yoy_change_pct"] is not None
    for crop in ("groundnut", "corn", "sesame"):
        assert price_summary(crop)["current"]["price"] > 0


def test_llm_tool_loop_with_stub():
    from farmos.planner.llm import converse, StubLLMClient
    trace = []
    text, msgs = converse(StubLLMClient(crop="groundnut", after="2026-08-10"),
                          "When should I sow groundnut?", trace=trace)
    assert "2026-09-03" in text
    assert any(t["tool"] == "survey_sowing_window" for t in trace)
    assert trace[0]["result"]["recommended_both_systems"][0]["date"] == "2026-09-03"


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
