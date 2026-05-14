"""
Unit tests for rover_autonomy.py — no camera or GPU required.
Run with: python -m pytest test_rover_autonomy.py -v
"""

import pytest
from rover_autonomy import Detection, DecisionEngine, DANGER_ZONE_LEFT, DANGER_ZONE_RIGHT

FRAME_W = 1280
FRAME_H = 720

@pytest.fixture
def engine():
    return DecisionEngine(FRAME_W, FRAME_H)


# ─── Detection helper ─────────────────────────────────────────────────────────

def make_det(label, cx_frac, cy_frac=0.5, size_frac=0.15, conf=0.8):
    """Create a detection centred at (cx_frac, cy_frac) of the frame."""
    w = int(FRAME_W * size_frac)
    h = int(FRAME_H * size_frac)
    cx = int(FRAME_W * cx_frac)
    cy = int(FRAME_H * cy_frac)
    return Detection(label, conf, cx - w//2, cy - h//2, cx + w//2, cy + h//2)


# ─── Detection property tests ─────────────────────────────────────────────────

class TestDetection:
    def test_centroid(self):
        d = Detection("person", 0.9, 100, 200, 300, 400)
        assert d.cx == 200
        assert d.cy == 300

    def test_area(self):
        d = Detection("person", 0.9, 0, 0, 100, 50)
        assert d.area == 5000

    def test_small_detection(self):
        d = Detection("bottle", 0.6, 640, 360, 641, 361)
        assert d.area == 1


# ─── Decision engine tests ────────────────────────────────────────────────────

class TestDecisionEngine:

    def test_clear_path_gives_forward(self, engine):
        # Seed history so smoothing resolves
        for _ in range(5):
            decision = engine.decide([])
        assert decision.action == "FORWARD"

    def test_obstacle_left_of_centre_steers_right(self, engine):
        # Large obstacle in danger zone, left half → steer right
        det = make_det("person", cx_frac=0.35, size_frac=0.25)
        for _ in range(5):
            decision = engine.decide([det])
        assert decision.action in ("STEER_RIGHT", "STOP")

    def test_obstacle_right_of_centre_steers_left(self, engine):
        det = make_det("person", cx_frac=0.65, size_frac=0.25)
        for _ in range(5):
            decision = engine.decide([det])
        assert decision.action in ("STEER_LEFT", "STOP")

    def test_very_large_obstacle_stops(self, engine):
        # Obstacle filling >20% of frame → STOP regardless of position
        det = make_det("person", cx_frac=0.5, size_frac=0.50)
        for _ in range(5):
            decision = engine.decide([det])
        assert decision.action == "STOP"

    def test_obstacle_outside_danger_zone_ignored(self, engine):
        # Obstacle at far left edge — outside DANGER_ZONE_LEFT
        det = make_det("person", cx_frac=0.05, size_frac=0.25)
        for _ in range(5):
            decision = engine.decide([det])
        assert decision.action == "FORWARD"

    def test_non_obstacle_class_ignored(self, engine):
        # "airplane" is not in OBSTACLE_CLASSES
        det = make_det("airplane", cx_frac=0.5, size_frac=0.30)
        for _ in range(5):
            decision = engine.decide([det])
        assert decision.action == "FORWARD"

    def test_tiny_detection_ignored(self, engine):
        # Obstacle below DANGER_ZONE_MIN_AREA threshold
        det = make_det("person", cx_frac=0.5, size_frac=0.01)
        for _ in range(5):
            decision = engine.decide([det])
        assert decision.action == "FORWARD"

    def test_smoothing_prevents_flicker(self, engine):
        # Alternating detections should not flicker between actions
        det = make_det("person", cx_frac=0.5, size_frac=0.30)
        actions = []
        for i in range(10):
            dets = [det] if i % 2 == 0 else []
            actions.append(engine.decide(dets).action)
        # Should not alternate every single frame
        changes = sum(1 for a, b in zip(actions, actions[1:]) if a != b)
        assert changes < 8  # some stability expected

    def test_multiple_obstacles_picks_largest(self, engine):
        small = make_det("person", cx_frac=0.40, size_frac=0.10)
        large = make_det("person", cx_frac=0.60, size_frac=0.30)
        for _ in range(5):
            decision = engine.decide([small, large])
        # Large one is on the right half → should steer left (or stop)
        assert decision.action in ("STEER_LEFT", "STOP")

    def test_decision_has_reason(self, engine):
        det = make_det("chair", cx_frac=0.5, size_frac=0.25)
        for _ in range(5):
            decision = engine.decide([det])
        if decision.action != "FORWARD":
            assert len(decision.reason) > 0

    def test_fresh_engine_handles_empty(self, engine):
        """No crash on first frame with no history."""
        decision = engine.decide([])
        assert decision.action in ("FORWARD", "STOP", "STEER_LEFT", "STEER_RIGHT")


# ─── Danger zone boundary tests ───────────────────────────────────────────────

class TestDangerZone:

    def test_danger_zone_boundaries_are_valid(self):
        assert 0 < DANGER_ZONE_LEFT < 0.5
        assert 0.5 < DANGER_ZONE_RIGHT < 1.0
        assert DANGER_ZONE_LEFT < DANGER_ZONE_RIGHT

    def test_centre_is_in_danger_zone(self, engine):
        d = make_det("person", cx_frac=0.5, size_frac=0.20)
        assert engine._in_danger_zone(d)

    def test_far_left_not_in_danger_zone(self, engine):
        d = make_det("person", cx_frac=0.05, size_frac=0.05)
        assert not engine._in_danger_zone(d)

    def test_far_right_not_in_danger_zone(self, engine):
        d = make_det("person", cx_frac=0.95, size_frac=0.05)
        assert not engine._in_danger_zone(d)
