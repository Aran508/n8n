import math
from pathlib import Path

import pytest
import yaml

from forge import PlanError, Planner, load_cell
from forge.budget import feed_force, torque_table, worst_by_tool
from forge.kinematics import Unreachable, forward, inverse

RECIPES = Path(__file__).resolve().parent.parent / "recipes"
cell = load_cell()
C = cell.links.coupler


def recipe(name):
    return yaml.safe_load((RECIPES / f"{name}.yaml").read_text())


def plan(r):
    return Planner(cell, r.get("arms")).plan(r)


@pytest.mark.parametrize("arm", list(cell.arms))
@pytest.mark.parametrize("tool", ["gripper_A", "spindle", "pen"])
def test_ik_round_trip_at_fixture(arm, tool):
    target = cell.location("fixture")
    l3 = C + cell.tools[tool].length
    joints = inverse(cell, cell.arms[arm], target, -90, 0, l3)
    assert math.dist(forward(cell, cell.arms[arm], joints, l3), target) < 0.1  # joints rounded to 0.01 deg


def test_tool_points_down():
    j = inverse(cell, cell.arms["arm1"], cell.location("fixture"), -90, 0, C + 38)
    assert abs(j[1] + j[2] + j[3] + 90) < 1e-6


def test_far_target_is_rejected_with_reason():
    with pytest.raises(Unreachable, match="max is 152 mm"):
        inverse(cell, cell.arms["arm3"], (105, 105, 0))


@pytest.mark.parametrize("tool", list(cell.tools))
def test_every_tool_slot_reachable_by_both_neighbours(tool):
    spec = cell.tools[tool]
    z = cell.racks[spec.rack]["z"]
    for arm in cell.racks[spec.rack]["arms"]:
        inverse(cell, cell.arms[arm], (*spec.slot, z), -90, 0, C)  # bare coupler docking
        inverse(cell, cell.arms[arm], (*spec.slot, z + 5), -90, 0, C + spec.length)  # lifting it out


def test_servo_budget_with_every_tool():
    assert all(row["ok"] for row in torque_table(cell))
    assert all(r["ok"] for r in worst_by_tool(cell))
    assert feed_force(cell)["max_feed_N"] > 2


@pytest.mark.parametrize("name", ["week1_single_arm", "week2_four_arms", "doweled_bracket"])
def test_build_recipes_plan_cleanly(name):
    tl = plan(recipe(name))
    assert tl.duration > 0 and tl.segments


def test_rack_capacity_enforced():
    with pytest.raises(PlanError, match="zone conflict in rack_L"):
        plan(recipe("week2_zone_conflict"))


def test_ai_first_draft_is_caught():
    with pytest.raises(PlanError, match="arm2 cannot reach rack_L"):
        plan(recipe("ai_first_draft"))


def test_bracket_ends_in_finished_tray_with_pins():
    events = plan(recipe("doweled_bracket")).events
    detaches = [e.data for e in events if e.type == "detach"]
    assert detaches[-1]["item"] == "plate" and detaches[-1]["pos"] == list(cell.location("finished"))
    assert {d["item"] for d in detaches if d.get("into") == "plate"} == {"pin1", "pin2"}
    cuts = [e.data for e in events if e.type == "cut"]
    assert [c["bit"] for c in cuts] == ["ream_2_0", "ream_2_0", "csink_90", "csink_90"]


def test_all_tools_back_in_racks_after_bracket():
    p = Planner(cell)
    p.plan(recipe("doweled_bracket"))
    assert set(p.tool_at.values()) == {"rack"} and set(p.bit_at.values()) == {"block"}


def test_wrong_tool_gives_a_fix():
    r = {"items": {"plate": {"at": "kit.plate"}}, "steps": [{"pick": {"arm": "arm3", "item": "plate"}}]}
    with pytest.raises(PlanError, match="add tool_change: {arm: arm3, tool: gripper_A}"):
        plan(r)


def test_tool_cannot_be_on_two_arms():
    r = {"steps": [{"tool_change": {"arm": "arm1", "tool": "pen"}}, {"tool_change": {"arm": "arm3", "tool": "pen"}}]}
    with pytest.raises(PlanError, match="pen is on arm1"):
        plan(r)


def test_machining_needs_a_bit():
    r = {"steps": [{"tool_change": {"arm": "arm4", "tool": "spindle"}},
                   {"machine": {"arm": "arm4", "at": "fixture.hole1"}}]}
    with pytest.raises(PlanError, match="no bit"):
        plan(r)


def test_tool_length_moves_the_tcp():
    p = Planner(cell, ["arm4"])
    p.plan({"steps": [{"tool_change": {"arm": "arm4", "tool": "spindle"}}, {"bit_change": {"arm": "arm4", "bit": "ream_2_0"}}]})
    st = p.state["arm4"]
    flange_z = forward(cell, cell.arms["arm4"], st.joints, C)[2]
    assert abs(flange_z - st.tip[2] - (36 + 16)) < 0.1


@pytest.mark.parametrize("name", ["micro_drone", "desk_bot_head", "led_bar", "doweled_bracket", "week2_four_arms"])
def test_auto_schedule_is_faster_and_collision_free(name):
    r = recipe(name)
    auto = Planner(cell, r.get("arms"))
    tl = auto.plan(r)
    seq = plan({**r, "schedule": "sequential"})
    assert tl.duration < 0.9 * seq.duration
    assert auto._first_conflict(0, tl.duration, None) is None


@pytest.mark.parametrize("name", ["micro_drone", "desk_bot_head", "led_bar"])
def test_projects_finish_in_the_tray(name):
    events = plan(recipe(name)).events
    last = [e.data for e in events if e.type == "detach"][-1]
    assert last["pos"] == list(cell.location("finished"))


def test_nearby_work_keeps_recipe_order():
    steps = {e.data["label"]: e for e in plan(recipe("micro_drone")).events if e.type == "step"}
    board = steps["Flight controller onto the centre standoffs"]
    screws = steps["Two M2 screws hold the board"]
    assert screws.t >= board.data["end"]


def test_soldering_needs_the_iron_and_screws_need_a_hex_bit():
    with pytest.raises(PlanError, match="needs a iron to solder"):
        plan({"steps": [{"solder": {"arm": "arm4", "at": ["led.j1a"]}}]})
    r = {"steps": [{"tool_change": {"arm": "arm4", "tool": "spindle"}}, {"bit_change": {"arm": "arm4", "bit": "ream_2_0"}},
                   {"drive": {"arm": "arm4", "at": ["drone.screw1"]}}]}
    with pytest.raises(PlanError, match="hex bit"):
        plan(r)


def test_eight_solder_joints_on_the_led_bar():
    assert sum(1 for e in plan(recipe("led_bar")).events if e.type == "solder") == 8
