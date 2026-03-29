import numpy as np
from scipy.spatial.transform import Rotation as R


_EPS = 1e-8


def _safe_normalize(v: np.ndarray, eps: float = _EPS) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError(f"Cannot normalize near-zero vector (norm={n}).")
    return v / n


def is_closed(hand_points: np.ndarray, threshold: float = 0.15):
    return np.linalg.norm(hand_points[8] - hand_points[4]) < threshold


def hand_to_rot(hand_points: np.ndarray) -> R:
    """Compute rotation from hand landmark points (right hand assumed).

    Uses index finger direction and thumb-to-index axis to form orthonormal frame.
    """
    _forward = hand_points[4] - hand_points[1]  # thumb to wrist
    _forward /= np.linalg.norm(_forward)

    _side_skew = hand_points[5] - hand_points[1]  # index to wrist
    _side = _forward - _side_skew * np.dot(_forward, _side_skew)
    _side /= np.linalg.norm(_side)

    _up = np.cross(_side, _forward)
    _up /= np.linalg.norm(_up)

    try:
        return R.from_matrix(np.vstack([_up, -_forward, _side]).T)
    except ValueError:
        return R.from_matrix(np.eye(3))


def hand_to_pose(hand_points: np.ndarray) -> tuple[R, np.ndarray]:
    return hand_to_rot(hand_points), np.mean(hand_points, axis=0)


def clean_pose(
    tip1: np.ndarray, tip2: np.ndarray, base: np.ndarray, gripper_displacement: float
):
    """
    e.g.
    index finger tip
    thumb finger tip
    base of hand

    gripper displacement along the plane define by the fingers (so it's apples to apples)
    """

    # left/right axis between tips
    left = _safe_normalize(tip1 - tip2)

    # forward: from base toward tips, projected into plane orthogonal to left
    fwd = tip1 - base
    fwd = fwd - left * np.dot(left, fwd)
    fwd = _safe_normalize(fwd)

    # up: perpendicular to the plane spanned by fwd & left
    up = np.cross(fwd, left)
    up = _safe_normalize(up)

    # re-orthogonalize left to remove numeric drift (Gram-Schmidt-ish)
    left = np.cross(
        up, fwd
    )  # already orthogonal to both; unit if up & fwd are unit+orthogonal

    rot = np.column_stack([fwd, left, up])  # columns are basis vectors

    # enforce right-handed frame (should already be right handed due to the order of the arguments in np.cross)
    if np.linalg.det(rot) < 0:
        rot[:, 2] *= -1  # flip up

    tip_center = (tip1 + tip2) / 2.0
    ee_position = tip_center - gripper_displacement * fwd
    return R.from_matrix(rot), ee_position


def hand_fingers_to_pose(
    hand_points: np.ndarray,
    gripper_displacement: float = 0.13,
) -> tuple[R, np.ndarray]:
    """
    gripper tip points
    gripper base point

    human hand tip points
    human hand base point
    """
    hand_base = hand_points[0]
    hand_thumb = hand_points[4]
    hand_index = hand_points[8]
    return clean_pose(hand_thumb, hand_index, hand_base, gripper_displacement)
