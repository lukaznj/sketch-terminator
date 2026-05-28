#!/usr/bin/env python3

import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from .kinematics import Kinematics

class TrajectoryGenerator:
    def __init__(self, v_max=1.0, a_max=2.0, dt_min=0.1, sample_dt=0.05):
        """
        v_max: max joint velocity (rad/s)
        a_max: max joint acceleration (rad/s^2)
        dt_min: minimum duration for any segment (s)
        sample_dt: interpolation sampling time (s), if None no interpolation is done
        """
        self.v_max = v_max
        self.a_max = a_max
        self.dt_min = dt_min
        self.sample_dt = sample_dt
        self.kinematics = Kinematics()

    def generate_trajectory(self, cartesian_path, current_joints=None, default_z=0.0):
        """
        Converts a Cartesian path [[x0, y0], [x1, y1], ...] (in meters) into a JointTrajectory message.
        If default_z is provided, uses that as the constant height for all points.
        If path points are 3D [x, y, z], uses their z values.
        current_joints: optional [b, g, a] starting joint positions to prepend to the trajectory.
        """
        # Parse path waypoints to 3D in meters
        waypoints_3d = []
        for p in cartesian_path:
            if isinstance(p, dict):
                x = p.get('x', 0.0)
                y = p.get('y', 0.0)
                z = p.get('z', default_z)
                waypoints_3d.append([x, y, z])
            elif len(p) >= 3:
                waypoints_3d.append([p[0], p[1], p[2]])
            else:
                waypoints_3d.append([p[0], p[1], default_z])

        # Convert waypoints to joint angles [beta, gama, alpha]
        joint_points = []
        for wp in waypoints_3d:
            joints = self.kinematics.get_ik(wp[0], wp[1], wp[2])
            joint_points.append(joints)

        # Prepend current joint angles if available
        if current_joints is not None:
            joint_points = [current_joints] + joint_points

        if not joint_points:
            return None

        # Determine segment times
        times_from_start = [0.0]
        cumulative_time = 0.0

        for i in range(1, len(joint_points)):
            q_prev = joint_points[i-1]
            q_curr = joint_points[i]
            
            # Find max duration over all joints based on v_max and a_max
            dt_segment = self.dt_min
            for j in range(3):
                dq = abs(q_curr[j] - q_prev[j])
                t_vel = dq / self.v_max
                # Under constant acceleration to mid-point and deceleration to stop, t_acc = 2 * sqrt(dq / a_max)
                # But here we use a conservative bound: t_acc = sqrt(2 * dq / a_max) or simply 2 * sqrt(dq / a_max)
                t_acc = math.sqrt(2.0 * dq / self.a_max) if self.a_max > 0 else 0.0
                dt_segment = max(dt_segment, t_vel, t_acc)
            
            cumulative_time += dt_segment
            times_from_start.append(cumulative_time)

        # Interpolate points if sample_dt is set
        final_joint_points = []
        final_times = []

        if self.sample_dt is not None and self.sample_dt > 0:
            total_duration = times_from_start[-1]
            t = 0.0
            segment_idx = 0
            
            while t <= total_duration:
                # Find current segment
                while segment_idx < len(times_from_start) - 1 and t > times_from_start[segment_idx + 1]:
                    segment_idx += 1
                
                t0 = times_from_start[segment_idx]
                t1 = times_from_start[segment_idx + 1]
                q0 = joint_points[segment_idx]
                q1 = joint_points[segment_idx + 1]
                
                # Linear interpolation
                if t1 > t0:
                    factor = (t - t0) / (t1 - t0)
                else:
                    factor = 1.0
                
                q_interp = []
                for j in range(3):
                    val = q0[j] + factor * (q1[j] - q0[j])
                    q_interp.append(val)
                
                final_joint_points.append(q_interp)
                final_times.append(t)
                t += self.sample_dt
            
            # Ensure the last point is exactly reached
            if final_times[-1] < total_duration:
                final_joint_points.append(joint_points[-1])
                final_times.append(total_duration)
        else:
            final_joint_points = joint_points
            final_times = times_from_start

        # Create JointTrajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        for i, q in enumerate(final_joint_points):
            point = JointTrajectoryPoint()
            point.positions = [float(q[0]), float(q[1]), float(q[2])]
            
            # Convert time to ROS Duration msg
            t_sec = final_times[i]
            sec = int(t_sec)
            nanosec = int((t_sec - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            # Estimate velocities (optional, but good for smooth motion)
            if i > 0 and i < len(final_joint_points) - 1:
                dt_prev = final_times[i] - final_times[i-1]
                dt_next = final_times[i+1] - final_times[i]
                point.velocities = []
                for j in range(3):
                    v = (final_joint_points[i+1][j] - final_joint_points[i-1][j]) / (dt_prev + dt_next)
                    # Limit to max velocity
                    v = max(-self.v_max, min(self.v_max, v))
                    point.velocities.append(float(v))
            else:
                point.velocities = [0.0, 0.0, 0.0]

            trajectory.points.append(point)

        return trajectory
