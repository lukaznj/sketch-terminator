#!/usr/bin/env python3

import math

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class Kinematics:
    def __init__(self):
        # Link lengths in meters
        self.L1 = 0.2
        self.L2 = 0.2531
        self.H_base = 0.09

        # Offsets (can be set if needed)
        self.offset_x = 0.0
        self.offset_y = 0.0

    def get_dk(self, beta_msg, gama_msg, alpha_msg):
        """
        Direct kinematics (DK).
        Takes joint angles [beta, gama, alpha] (in radians) and returns [x, y, z] end effector position in meters.
        """
        alpha = alpha_msg + math.pi / 2
        
        # Geometrical angles from joint commands
        b = math.pi / 2 - beta_msg
        g1 = math.pi - gama_msg

        # 3D planar radius
        r1 = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(g1))

        # Angle b2
        cos_b2 = (self.L1**2 + r1**2 - self.L2**2) / (2 * self.L1 * r1)
        cos_b2 = max(-1.0, min(1.0, cos_b2))
        b2 = math.acos(cos_b2)

        # Angle b1
        b1 = math.pi - b - b2

        # Radial distance and height offset
        r = r1 * math.sin(b1)
        h1 = r1 * math.cos(b1)

        # Coordinates
        mz = self.H_base - h1
        mx = r * math.cos(alpha) + self.offset_x
        my = r * math.sin(alpha) + self.offset_y

        return [mx, my, mz]

    def get_ik(self, x, y, z):
        """
        Inverse kinematics (IK) with safety limits.
        Takes end effector position [x, y, z] (in meters) and returns [beta, gama, alpha] joint angles (in radians).
        """
        # Apply offsets
        x_adj = x - self.offset_x
        y_adj = y - self.offset_y

        # Safety limit for Z (in both meters and centimeters/millimeters)
        if z >= 9.0:
            z = 8.0
        elif z >= 0.09:
            z = 0.08

        h1 = self.H_base - z
        
        # Calculate base angle
        a = math.atan2(y_adj, x_adj)

        # Safety limits for base angle
        if a < math.pi * -0.7:
            a = math.pi * -0.7
        if a > math.pi * 0.7:
            a = math.pi * 0.7
        
        r = math.sqrt(x_adj**2 + y_adj**2)
        b1 = math.atan2(r, h1)
        r1 = math.sqrt(r**2 + h1**2)

        # Triangle equations for L1, L2, r1
        # cos_b2 = (L1^2 + r1^2 - L2^2) / (2 * r1 * L1)
        cos_b2_val = (self.L1**2 + r1**2 - self.L2**2) / (2 * r1 * self.L1)
        cos_b2_val = max(-1.0, min(1.0, cos_b2_val))
        b2 = math.acos(cos_b2_val)

        # cos_g1 = (L1^2 + L2^2 - r1^2) / (2 * L1 * L2)
        cos_g1_val = (self.L1**2 + self.L2**2 - r1**2) / (2 * self.L1 * self.L2)
        cos_g1_val = max(-1.0, min(1.0, cos_g1_val))
        g1 = math.acos(cos_g1_val)

        b = math.pi - b1 - b2
        g = math.pi - g1

        # Transform to ROS messages joint ranges
        a_msg = a - math.pi/2
        b_msg = math.pi/2 - b
        g_msg = g

        return [wrap_to_pi(b_msg), wrap_to_pi(g_msg), wrap_to_pi(a_msg)]
