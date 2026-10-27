import yaml

from sa_tasks import run_map

RECORD = {
    "step": "full", "result": "ok", "duration_s": 70.0,
    "timeline_s": {"start": 0.0, "iris_closed": 20.5, "m4_stopped": 60.1},
    "pick": {"nut_coarse_mm": [365.0, 155.0, 16.0], "nut_mm": [359.0, 155.5, 16.0],
             "landed_tcp_mm": [358.5, 154.5, 12.9]},
    "place": {"stud": {"position_mm": [431.0, -153.5, 32.0], "observed_mm": [435.0, -149.0, 32.0],
                       "close_mm": [433.0, -148.5, 32.0]},
              "contact_tcp_mm": [431.2, -153.5, 54.7],
              "screw": {"end_tcp_mm": [430.0, -152.5, 54.0]}},
}


def test_summary_row_has_positions_sides_and_target_errors():
    row = run_map.summarise("20261027-101500_full", RECORD)
    assert row["status"] == "unrated"
    assert (row["nut_side"], row["stud_side"]) == ("left", "right")
    assert [row["landed_dx"], row["landed_dy"]] == [-0.5, -1.0]
    assert [row["end_dx"], row["end_dy"]] == [-1.0, 1.0]
    assert row["thread_dx"] is None  # no thread search in this record
    assert row["t_iris_closed_s"] == 20.5 and row["t_m4_stopped_s"] == 60.1


def test_status_from_the_rating_else_from_the_result():
    assert run_map.status({**RECORD, "outcome": {"screwed": "no"}}) == "failed"
    assert run_map.status({**RECORD, "outcome": {"screwed": "partly"}}) == "partly"
    assert run_map.status({**RECORD, "result": "failed: no stud"}) == "aborted"


def test_load_filters_by_date_and_step(tmp_path):
    for name, step in (("20261026-130000_full", "full"), ("20261027-100000_full", "full"),
                       ("20261027-100500_pick", "pick")):
        (tmp_path / f"{name}.yaml").write_text(yaml.safe_dump({**RECORD, "step": step}))
    (tmp_path / "tcp_offset.yaml").write_text("- {}\n")
    rows = run_map.load(tmp_path, "20261027", ["full"])
    assert [row["record"] for row in rows] == ["20261027-100000_full"]
    run_map.write_csv(rows, tmp_path / "s.csv")
    run_map.plot(rows, {"nut": "blue", "stud": "red"}, tmp_path / "m.png")
    assert (tmp_path / "m.png").stat().st_size > 0


def test_step_times_and_table():
    assert run_map.step_times({"start": 0.0, "a": 2.5, "b": 4.0}) == {"start": 0.0, "a": 2.5,
                                                                     "b": 1.5}
    row = {"record": "20261027-101500_full", "timeline": {"start": 0.0, "a": 2.5}}
    table = run_map.times_table([row])
    assert "101500" in table and "2.5" in table and table.splitlines()[-1].endswith("2.5")
