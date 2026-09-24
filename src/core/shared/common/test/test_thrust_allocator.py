import math

import pytest

from common.models import USVModel
from common.thrust_allocator import FixedThrustAllocator


def test_fixed_allocator_uses_project_force_sign_and_reports_rpm():
    model = USVModel(max_force=180.0, D=0.395)
    allocator = FixedThrustAllocator(model)

    allocation = allocator.allocate(surge_force=20.0, yaw_moment=0.0,
                                    command_limit=1.0)

    assert allocation.left_force == pytest.approx(10.0)
    assert allocation.right_force == pytest.approx(10.0)
    assert allocation.command.left == pytest.approx(10.0 / 180.0)
    assert allocation.command.right == pytest.approx(10.0 / 180.0)
    assert allocation.left_rpm > 0.0
    assert allocation.right_rpm == pytest.approx(allocation.left_rpm)
    assert allocator.rpm_to_force(allocation.left_rpm) == pytest.approx(10.0)


def test_fixed_allocator_positive_yaw_increases_right_force():
    model = USVModel(max_force=180.0, D=0.395)
    allocation = FixedThrustAllocator(model).allocate(0.0, 1.0,
                                                       command_limit=1.0)

    assert allocation.left_force < 0.0
    assert allocation.right_force > 0.0
    assert allocation.right_force - allocation.left_force == pytest.approx(1.0 / model.D)
    assert allocation.command.right > allocation.command.left


def test_fixed_allocator_rpm_limits_are_converted_back_to_applied_force():
    model = USVModel(max_force=180.0, D=0.395)
    probe = FixedThrustAllocator(model)
    limit_rpm = probe.force_to_rpm(2.0)
    allocator = FixedThrustAllocator(
        model, {"left_rpm_min": -limit_rpm, "left_rpm_max": limit_rpm,
                "right_rpm_min": -limit_rpm, "right_rpm_max": limit_rpm})

    allocation = allocator.allocate(20.0, 0.0, command_limit=1.0)

    assert allocation.left_rpm == pytest.approx(limit_rpm)
    assert allocation.right_rpm == pytest.approx(limit_rpm)
    assert allocation.left_force == pytest.approx(2.0)
    assert allocation.right_force == pytest.approx(2.0)
    assert allocation.command.left == pytest.approx(2.0 / 180.0)
    assert allocation.command.right == pytest.approx(2.0 / 180.0)
    assert math.isfinite(allocation.applied_surge_force)
