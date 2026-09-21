"""Forward and inverse kinematics for the 3-DOF Sketch Terminator arm.

This is the single source of truth for the arm's geometry. Every node that
converts between joint angles and Cartesian marker positions goes through this
module, so the link lengths and the safety limits only ever live in one place.

Geometry
--------
The arm is a base rotation followed by two revolute joints in a vertical plane::

    joint3 (alpha) -- base yaw, rotates the whole arm about Z
    joint1 (beta)  -- shoulder, lifts the first link
    joint2 (gama)  -- elbow, between the two links

The naming is inherited from the original hand-derived equations and is kept
because the URDF, the controllers and the GUI all use it:

    alpha (a) -- base angle,     published as ``joint3``
    beta  (b) -- shoulder angle, published as ``joint1``
    gama  (g) -- elbow angle,    published as ``joint2``

Angles are radians, lengths are metres. The marker tip touches the paper at
``z = 0`` and is fully lifted at ``z = H_BASE``.
"""

import math

# Link lengths, measured from the CAD assembly (metres).
L1 = 0.2       # shoulder joint -> elbow joint
L2 = 0.2531    # elbow joint -> marker tip
H_BASE = 0.09  # height of the shoulder joint above the paper

# Safety limits. These are deliberately tighter than the mechanical range: the
# arm has no self-collision checking, so the envelope is kept conservative.
Z_MAX = 0.09           # never command the marker above the shoulder pivot
Z_LIFTED = 0.08        # height a clamped-too-high z is pulled back down to
BASE_ANGLE_MAX = 0.7 * math.pi   # +/- base yaw, keeps the arm off its own cabling
MIN_RADIUS = 0.05      # closest the tip may come to the base axis; inside this
                       # radius the base yaw becomes ill-conditioned and the
                       # arm whips around trying to track small XY changes


def wrap_to_pi(angle):
    """Fold an angle into (-pi, pi]."""
    return math.remainder(angle, 2 * math.pi)


def _clamp(value, low, high):
    return max(low, min(high, value))


class Kinematics:
    """Converts between joint angles and marker tip positions.

    ``offset_x`` / ``offset_y`` shift the whole workspace in the base frame.
    They exist so a mounting offset measured on the real rig can be dialled in
    without touching the equations; both default to zero.
    """

    def __init__(self, offset_x=0.0, offset_y=0.0):
        self.L1 = L1
        self.L2 = L2
        self.H_base = H_BASE
        self.offset_x = offset_x
        self.offset_y = offset_y

    def get_dk(self, beta_msg, gama_msg, alpha_msg):
        """Forward kinematics: joint angles -> marker tip ``[x, y, z]`` in metres.

        Takes the angles exactly as they appear on ``/joint_states``
        (``joint1``, ``joint2``, ``joint3``) and undoes the offsets that
        :meth:`get_ik` applies, so ``get_dk(*get_ik(p)) == p`` inside the
        reachable workspace.
        """
        # Undo the message-frame offsets to recover the geometric angles.
        alpha = alpha_msg + math.pi / 2
        b = math.pi / 2 - beta_msg
        g1 = math.pi - gama_msg

        # Law of cosines over the L1/L2 triangle: distance from the shoulder
        # joint straight to the marker tip.
        r1 = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(g1))

        # Angle between L1 and r1, again by the law of cosines. The clamp
        # absorbs the float error that makes |cos| creep just past 1.0 when the
        # arm is fully stretched or fully folded.
        cos_b2 = (self.L1**2 + r1**2 - self.L2**2) / (2 * self.L1 * r1)
        b2 = math.acos(_clamp(cos_b2, -1.0, 1.0))

        # Angle of r1 away from vertical, then split into radius and height.
        b1 = math.pi - b - b2
        r = r1 * math.sin(b1)
        h1 = r1 * math.cos(b1)

        # Project the planar radius out along the base yaw.
        return [
            r * math.cos(alpha) + self.offset_x,
            r * math.sin(alpha) + self.offset_y,
            self.H_base - h1,
        ]

    def get_ik(self, x, y, z):
        """Inverse kinematics: marker tip ``[x, y, z]`` -> ``[beta, gama, alpha]``.

        Returns the angles in the order the controllers expect them
        (``joint1``, ``joint2``, ``joint3``), already wrapped to (-pi, pi].

        Targets outside the safe envelope are clamped rather than rejected, so
        a caller streaming points at 25 Hz never has to handle an exception
        mid-stroke. A target outside the arm's *reach* still yields the closest
        fully-stretched or fully-folded pose, because the law-of-cosines
        arguments are clamped.
        """
        x_adj = x - self.offset_x
        y_adj = y - self.offset_y

        z = _clamp(z, 0.0, Z_MAX)
        if z >= Z_MAX:
            z = Z_LIFTED
        h1 = self.H_base - z

        # atan2 (not atan) so the full circle of base angles resolves correctly.
        a = _clamp(math.atan2(y_adj, x_adj), -BASE_ANGLE_MAX, BASE_ANGLE_MAX)

        # Push the target out of the singular column above the base axis.
        r = max(math.sqrt(x_adj**2 + y_adj**2), MIN_RADIUS)

        b1 = math.atan2(r, h1)
        r1 = math.sqrt(r**2 + h1**2)

        # Same triangle as in get_dk, solved the other way round.
        cos_b2 = (self.L1**2 + r1**2 - self.L2**2) / (2 * r1 * self.L1)
        b2 = math.acos(_clamp(cos_b2, -1.0, 1.0))

        cos_g1 = (self.L1**2 + self.L2**2 - r1**2) / (2 * self.L1 * self.L2)
        g1 = math.acos(_clamp(cos_g1, -1.0, 1.0))

        b = math.pi - b1 - b2
        g = math.pi - g1

        # Shift into the joint frames used on the wire.
        return [
            wrap_to_pi(math.pi / 2 - b),  # joint1 (beta)
            wrap_to_pi(g),                # joint2 (gama)
            wrap_to_pi(a - math.pi / 2),  # joint3 (alpha)
        ]


def _self_check():
    """Round-trip IK against DK over the reachable workspace."""
    kin = Kinematics()
    for x in (0.10, 0.20, 0.30, 0.40):
        for y in (-0.20, -0.05, 0.0, 0.15, 0.30):
            for z in (0.0, 0.03, 0.08):
                r = math.hypot(x, y)
                # Skip targets the arm physically cannot reach; those clamp to
                # the nearest pose instead of round-tripping.
                if not MIN_RADIUS <= r or math.hypot(r, H_BASE - z) > L1 + L2:
                    continue
                back = kin.get_dk(*kin.get_ik(x, y, z))
                for want, got, axis in zip((x, y, z), back, "xyz"):
                    assert abs(want - got) < 1e-6, f"{axis}: {want} -> {got}"

    # Out-of-envelope targets must clamp, not raise.
    assert kin.get_ik(0.0, 0.0, 5.0)
    assert kin.get_ik(9.0, 9.0, -1.0)
    assert abs(wrap_to_pi(3 * math.pi)) - math.pi < 1e-9
    print("kinematics self-check passed")


if __name__ == "__main__":
    _self_check()
