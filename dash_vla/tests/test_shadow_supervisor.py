"""The H0 shadow records a skill call and returns the original result."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dash_vla.harness.shadow import ShadowSafetySupervisor


class _Result:
    def __init__(self, data: dict) -> None:
        self.data = data


class _Toolkit:
    def __init__(self) -> None:
        self.calls = []

    def execute_tool(self, name, arguments):
        self.calls.append((name, dict(arguments)))
        return _Result(
            {
                "terminated": False,
                "success": True,
                "check_success": True,
                "cost": {"checkcontact": 1},
                "state": {
                    "robot0_eef_pos": [-0.20, 0.0, 1.17],
                    "robot0_gripper_qpos": [0.02, -0.02],
                },
            }
        )


def test_shadow_preserves_the_action_and_drops_ground_truth(tmp_path: Path) -> None:
    toolkit = _Toolkit()
    original = toolkit.execute_tool
    shadow = ShadowSafetySupervisor(tmp_path / "shadow.jsonl")
    shadow.attach(toolkit)

    seen = toolkit.execute_tool("view_env_state", {"step": 0})
    arguments = {"xyz": [0.20, 0.10, 1.20], "gripper": -1.0}
    result = toolkit.execute_tool("move_to", arguments)

    assert toolkit.calls[1] == ("move_to", arguments)
    assert seen.data["check_success"] is True
    assert result.data["cost"]["checkcontact"] == 1
    assert result is not seen
    assert toolkit.execute_tool is not original

    lines = (tmp_path / "shadow.jsonl").read_text().splitlines()
    record = json.loads(lines[1])
    assert record["tool"] == "move_to"
    assert record["action_changed"] is False
    assert record["gripper_opening"] == 0.04
    blob = "\n".join(lines)
    assert "check_success" not in blob
    assert "checkcontact" not in blob
    assert "single_move_horizontal_over_0_30m" in record["notes"]
    assert "open_command_while_grip_closed" in record["notes"]
    assert record["decision"] == "RECONSIDER"
    assert record["advice"] == ["split_move_under_0_30m", "hold_gripper_until_explicit_release"]
    assert record["recovery_executed"] is False
    assert record["post"]["action_changed"] is False
    assert record["hook"] == "tool_boundary"
    assert record["intra_chunk_monitoring"] is False
    assert "SAFE_ABORT" not in blob
    assert "SWITCH_SKILL" not in blob


def test_shadow_reraises_tool_errors(tmp_path: Path) -> None:
    class Broken:
        def execute_tool(self, name, arguments):
            raise RuntimeError("servo failed")

    toolkit = Broken()
    ShadowSafetySupervisor(tmp_path / "shadow.jsonl").attach(toolkit)
    try:
        toolkit.execute_tool("release", {})
    except RuntimeError as exc:
        assert str(exc) == "servo failed"
    else:
        raise AssertionError("the original tool error was swallowed")
    record = json.loads((tmp_path / "shadow.jsonl").read_text())
    assert record["error"] == "RuntimeError: servo failed"
    assert record["action_changed"] is False
    assert record["decision"] == "ALLOW"


def test_sam3_segment_is_perception_and_is_not_rewritten(tmp_path: Path) -> None:
    toolkit = _Toolkit()
    ShadowSafetySupervisor(tmp_path / "shadow.jsonl").attach(toolkit)
    arguments = {"prompt": "the sake bottle", "step": 0}
    toolkit.execute_tool("segment", arguments)
    assert toolkit.calls == [("segment", arguments)]
    record = json.loads((tmp_path / "shadow.jsonl").read_text())
    assert record["role"] == "sam3_perception"
    assert record["decision"] == "ALLOW"
    assert record["reasons"] == ["sam3_perception"]
    assert record["action_changed"] is False


def test_motion_without_an_eef_observation_is_restage_and_still_runs(tmp_path: Path) -> None:
    toolkit = _Toolkit()
    ShadowSafetySupervisor(tmp_path / "shadow.jsonl").attach(toolkit)
    arguments = {"xyz": [0.0, 0.0, 1.0], "gripper": 1.0}
    toolkit.execute_tool("move_to", arguments)
    assert toolkit.calls == [("move_to", arguments)]
    record = json.loads((tmp_path / "shadow.jsonl").read_text())
    assert record["decision"] == "RESTAGE"
    assert record["reasons"] == ["missing_eef_observation"]


def test_a_short_hold_move_is_allowed(tmp_path: Path) -> None:
    toolkit = _Toolkit()
    shadow = ShadowSafetySupervisor(tmp_path / "shadow.jsonl")
    shadow.attach(toolkit)
    toolkit.execute_tool("view_env_state", {"step": 0})
    arguments = {"xyz": [-0.22, 0.02, 1.17], "gripper": 1.0}
    toolkit.execute_tool("move_to", arguments)
    assert toolkit.calls[1] == ("move_to", arguments)
    record = json.loads((tmp_path / "shadow.jsonl").read_text().splitlines()[1])
    assert record["decision"] == "ALLOW"
    assert record["notes"] == []
    assert record["advice"] == []


def test_chunk_boundary_logs_without_editing_the_pending_action(tmp_path: Path) -> None:
    shadow = ShadowSafetySupervisor(tmp_path / "shadow.jsonl")
    pending = [[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]]
    before = [row[:] for row in pending]
    judgement = shadow.observe_chunk_boundary(
        observation={"eef_pos": [-0.20, 0.0, 1.17], "gripper_opening": 0.04, "check_success": True},
        proposed_skill={"name": "move_to", "arguments": {"xyz": [0.20, 0.10, 1.20], "gripper": -1.0}},
        pending_action=pending,
    )
    assert pending == before
    assert judgement["phase"] == "during"
    assert judgement["hook"] == "chunk_boundary"
    assert judgement["decision"] == "RECONSIDER"
    assert judgement["action_changed"] is False
    record = json.loads((tmp_path / "shadow.jsonl").read_text())
    assert "check_success" not in record
    assert record["intra_chunk_monitoring"] is False


def test_mechanism_gate_flags_only_the_bad_call(tmp_path: Path) -> None:
    from dash_vla.harness.gates import mechanism_gate, recovery_gate

    result = mechanism_gate(tmp_path / "shadow.jsonl")
    assert result["passed"] is True
    assert result["bad_decision"] == "RECONSIDER"
    assert result["normal_decision"] == "ALLOW"
    assert "single_move_horizontal_over_0_30m" in result["bad_reasons"]
    assert "open_command_while_grip_closed" in result["bad_reasons"]
    recovery = recovery_gate()
    assert recovery["passed"] is False
    assert recovery["opens_g6"] is False
    assert recovery["recovery_executed"] is False
