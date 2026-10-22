import math
from pathlib import Path

import pytest
import yaml

from forge import PlanError, Planner, load_cell
from forge.budget import torque_table
from forge.kinematics import Unreachable, forward, inverse

RECIPES = Path(__file__).resolve().parent.parent / "recipes"
cell = load_cell()


def recipe(name):
    return yaml.safe_load((RECIPES / f"{name}.yaml").read_text())


@pytest.mark.parametrize("arm", list(cell.arms))
@pytest.mark.parametrize("loc", ["fixture", "fixture.slot", "fixture.top", "fixture.edge"])
def test_ik_round_trip_on_plate(arm, loc):
    target = cell.location(loc)
    joints = inverse(cell, cell.arms[arm], target)
    assert math.dist(forward(cell, cell.arms[arm], joints), target) < 0.1  # joints are rounded to 0.01 deg


def test_tool_points_down():
    j = inverse(cell, cell.arms["arm1"], cell.location("fixture"))
    assert abs(j[1] + j[2] + j[3] + 90) < 1e-6


def test_far_target_is_rejected_with_reason():
    with pytest.raises(Unreachable, match="max is 152 mm"):
        inverse(cell, cell.arms["arm3"], cell.location("output"))


def test_servo_angles_stay_in_range():
    j = inverse(cell, cell.arms["arm2"], cell.location("fixture"))
    assert all(0 <= s <= 180 for s in cell.joints_to_servos(cell.arms["arm2"], j))


def test_torque_budget_within_half_stall():
    assert all(row["ok"] for row in torque_table(cell))


@pytest.mark.parametrize("name", ["week1_single_arm", "week2_four_arms", "led_keychain"])
def test_build_recipes_plan_cleanly(name):
    r = recipe(name)
    tl = Planner(cell, r.get("arms")).plan(r)
    assert tl.duration > 0 and tl.segments


def test_zone_capacity_enforced():
    with pytest.raises(PlanError, match="zone conflict"):
        r = recipe("week2_zone_conflict")
        Planner(cell, r.get("arms")).plan(r)


def test_ai_first_draft_is_caught():
    r = recipe("ai_first_draft")
    with pytest.raises(PlanError, match="arm3 cannot reach"):
        Planner(cell, r.get("arms")).plan(r)


def test_keychain_ends_in_output_tray():
    r = recipe("led_keychain")
    tl = Planner(cell, r.get("arms")).plan(r)
    last = [e for e in tl.events if e.type == "detach"][-1]
    assert last.data["item"] == "housing" and last.data["pos"] == list(cell.location("output"))


def test_cannot_pick_with_full_hands():
    r = {"items": {"led": {"at": "led_tray"}, "cap": {"at": "cap_tray"}},
         "steps": [{"pick": {"arm": "arm4", "item": "led"}}, {"pick": {"arm": "arm4", "item": "cap"}}]}
    with pytest.raises(PlanError, match="already holding"):
        Planner(cell).plan(r)
