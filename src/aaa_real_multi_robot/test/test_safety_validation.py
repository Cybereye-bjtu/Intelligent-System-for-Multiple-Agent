from geometry_msgs.msg import TransformStamped

from aaa_real_multi_robot.alignment_manager import transform_to_se2
from aaa_real_multi_robot.lease_gate import LeaseGate


def test_negative_limit_is_fail_closed():
    assert LeaseGate._bounded(0.5, -1.0) == 0.0
    assert LeaseGate._bounded(float("nan"), 1.0) == 0.0


def test_alignment_rejects_non_normalized_quaternion():
    message = TransformStamped()
    message.transform.rotation.w = 0.5
    try:
        transform_to_se2(message)
    except ValueError as error:
        assert "normalized" in str(error)
    else:
        raise AssertionError("non-normalized quaternion was accepted")
