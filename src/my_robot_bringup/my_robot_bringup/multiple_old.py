"""
How does this run,
We will run test node, which will take the raw motor cmds and publish them so that the robot moves with randomness,
We need to already know how the robot runs to generate the plan, to create the local goals, etc. But to validate we
want to be able to run, so similar to what we are already doing we create CSV files with odom data, could do the same with obstacle data?


Lets say we are odom_rad (x,y), we need to know yaw, and locations of revalent obstacles,

April 29: Currently, this takes odom_csv, and creates the local goals and obstacles to generate ray traces

"""
import matplotlib.patches as patches
import numpy as np
import rosbag2_py
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped, Twist
import tf2_geometry_msgs  # <-- this registers the PointStamped type with TF2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import matplotlib.pyplot as plt
import csv
import os
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker

import sys
from sensor_msgs.msg import LaserScan
# from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
import pandas as pd
import yaml
import time
import random

BASE_SEED = 12345
def euler_from_quaternion(x, y, z, w):
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

class Segment():

    def __init__(self, map_points, node, RADIUS, OFFSET, start_index=None, end_index=None, seg_seed =42):

        self.map_points = map_points
        self.node = node
        self.RADIUS = RADIUS
        self.OFFSET = OFFSET
        self.start_index = start_index
        self.end_index = end_index
        self.seg_seed = seg_seed

    def init(self):
        print("ininting") 
        print(f"len of map points {len(self.map_points)}")
        start_xy = self.map_points[0] if self.map_points else (0.0, 0.0)
        self.local_goal_manager_ = Local_Goal_Manager(start_xy)
        # self.local_goal_manager_ = Local_Goal_Manager(self.map_points)
        self.obstacle_manager_ = Obstacle_Manager(
            self.OFFSET, self.RADIUS, self.local_goal_manager_,seed=self.seg_seed, clean_mode=True)
        self.global_path = self.create_path()
        self.local_goal_manager_.global_path = self.global_path

        print(f"len of global path {len(self.global_path.poses)}")
        self.create_local_goals()
        self.create_obstacles()
        self.obstacle_manager_.prune_overlapping_with_path(
            self.global_path,
            clearance_extra=0.02,  # ~2cm buffer
            radius_mult=1.1        # require center ≥ 1.1*radius away from path
        )

    def create_path(self):
        return self.node.create_path_from_points(self.map_points)

    def create_local_goals(self):
        self.local_goal_manager_.generate_local_goals_claude(self.global_path)
        print(
            f"created the local_goals : {len(self.local_goal_manager_.data)}")

    def create_obstacles(self):
        self.obstacle_manager_.create_all_obstacle()
        print(
            f"created the obstacles {len(self.obstacle_manager_.obstacle_array)}")

    def get_obstacles(self, current_pos):
        return self.obstacle_manager_.get_active_obstacles_claude(self.global_path, current_pos)


class Obstacle():
    def __init__(self, center_x=None, center_y=None, radius=None):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius


class Obstacle_Manager():
    def __init__(self, OFFSET, RADIUS, local_goal_manager=None,
                 seed=42,
                 # realism knobs:
                 offset_range=(0.85, 1.30),   # width multiplier per segment
                 radius_range=(0.90, 1.15),   # radius multiplier per obstacle
                 jitter_std=0.05,             # XY noise (m)
                 along_std=0.04,              # slide along segment (m)
                 drop_prob=0.08,              # drop obstacle (gaps)
                 single_side_prob=0.12,       # only one side sometimes
                 pinch_prob=0.06,             # extra narrowing event
                 pinch_scale=(0.45, 0.75),    # pinch multiplier for width
                 clutter_prob=0.04,           # add free obstacle near corridor
                 clutter_rad=1.2, 
                 clean_mode = True):            # radius (m) from mid for clutter:
        self.obstacle_array = []
        self.local_goal_manager_ = local_goal_manager
        self.OFFSET = OFFSET
        self.RADIUS = RADIUS
        self.prev_dir_x = 0
        self.prev_dir_y = 0

        self._rng = random.Random(seed)
        # self._offset_range = offset_range
        # self._radius_range = radius_range
        # self._jitter_std = jitter_std
        # self._along_std = along_std
        # self._drop_prob = drop_prob
        # self._single_side_prob = single_side_prob
        # self._pinch_prob = pinch_prob
        # self._pinch_scale = pinch_scale
        # self._clutter_prob = clutter_prob
        # self._clutter_rad = clutter_rad

        # values for assymetric obstacle creation
        self.skew_s_max = 0.50     # max fractional skew 0..1
        self.skew_gain  = 4.0      # curvature → skew growth (bigger = more skew)
        self.widen_out  = 0.7      # outside widening relative to inside shrinking
        self.min_inside = 0.40     # floor for inside (as fraction of OFFSET)
        self.max_outmul = 1.80     # cap for outside (as multiple of OFFSET)

        # No noise added to obstacle
        self.clean_mode = clean_mode
        if self.clean_mode:
            # deterministic geometry
            self._offset_range   = (1.0, 1.0)   # no width scaling
            self._radius_range   = (1.0, 1.0)   # exact base radius
            self._jitter_std     = 0.0
            self._along_std      = 0.0
            self._drop_prob      = 0.0
            self._single_side_prob = 0.0
            self._pinch_prob     = 0.0
            self._clutter_prob   = 0.0

              # also disable curvature skew in clean mode
            self.skew_s_max = 0.3
            self.widen_out  = 0.5
        else:
            # keep the user-specified values (your current code)
            self._offset_range = offset_range
            self._radius_range = radius_range
            self._jitter_std   = jitter_std
            self._along_std    = along_std
            self._drop_prob    = drop_prob
            self._single_side_prob = single_side_prob
            self._pinch_prob   = pinch_prob
            self._clutter_prob = clutter_prob



    def _signed_curvature(self, i):
        """Signed curvature at local-goal index i (left turn +, right turn -)."""
        data = self.local_goal_manager_.data
        n = len(data)
        if n < 3:
            return 0.0
        # clamp neighbors
        i0 = max(0, i-1)
        i2 = min(n-1, i+1)
        p0 = data[i0].pose.position
        p1 = data[i].pose.position
        p2 = data[i2].pose.position
        # headings
        h0 = math.atan2(p1.y - p0.y, p1.x - p0.x)
        h1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
        # unwrap turn
        d = (h1 - h0 + math.pi) % (2*math.pi) - math.pi
        # use segment length as arc length
        L = math.hypot(p2.x - p1.x, p2.y - p1.y) + 1e-6
        return d / L
    def get_active_obstacles(self):
        current_local_goal_count = self.local_goal_manager_.get_local_goal_counter()
        total_goals = len(self.local_goal_manager_.data)
        print(f"local goal count {current_local_goal_count}")
        valid_border_min = max(0, current_local_goal_count - 4)
        valid_border_max = min(total_goals, current_local_goal_count + 20)
        active_list = self.obstacle_array[valid_border_min: valid_border_max]
        return active_list

    def get_active_obstacles_claude(self, global_path=None, current_index=None):
        """Select obstacles based on distance to the current robot position."""
        # Get current robot position
        robot_pos = self.local_goal_manager_.current_odom
        print(f"active obstacles reference to where it is {robot_pos}")
        # Calculate distance to each obstacle
        obstacles_with_distance = []
        for obs in self.obstacle_array:
            dist = math.sqrt((obs.center_x - robot_pos[0])**2 +
                             (obs.center_y - robot_pos[1])**2)
            obstacles_with_distance.append((obs, dist))

        # Sort by distance and take closest N obstacles
        obstacles_with_distance.sort(key=lambda x: x[1])
        active_obstacles = [obs for obs,
                            dist in obstacles_with_distance[:20] if dist < 5.0]

        if global_path is not None:
            end_index = min(current_index + 200, len(global_path.poses))
            path_points = []
            for i in range(current_index, end_index):
                pose = global_path.poses[i]
                path_points.append(
                    (pose.pose.position.x, pose.pose.position.y))

            valid_obs = []
            THRESHOLD = 1.5 * self.RADIUS
            for obs in active_obstacles:
                min_dist = self.calculate_min_distance_to_path(
                    obs.center_x, obs.center_y, path_points)
                if min_dist > THRESHOLD:
                    valid_obs.append(obs)
                    continue
                else:
                    continue

            return valid_obs
        else:
            return active_obstacles

    def prune_overlapping_with_path(self, global_path, clearance_extra=0.02, radius_mult=1.1):
        """
        Remove obstacles whose center is too close to the path polyline.
        For each obstacle, require:
            min_dist(center, path) >= max(radius * radius_mult, self.RADIUS * radius_mult) + clearance_extra
        """
        # build path points
        path_points = [(ps.pose.position.x, ps.pose.position.y) for ps in global_path.poses]

        kept = []
        dropped = 0
        for ob in self.obstacle_array:
            # dynamic clearance: scale with this obstacle's actual radius + small extra margin
            min_clear = max(ob.radius * radius_mult, self.RADIUS * radius_mult) + clearance_extra
            d = self.calculate_min_distance_to_path(ob.center_x, ob.center_y, path_points)
            if d >= min_clear:
                kept.append(ob)
            else:
                dropped += 1
        self.obstacle_array = kept
        print(f"[Obstacle prune] kept {len(kept)}, dropped {dropped} (too close to path)")
    def calculate_min_distance_to_path(self, x, y, path_points):
            """Calculate minimum distance from point (x,y) to the path."""
            min_dist = float('inf')

            for i in range(len(path_points) - 1):
                # Get points for this path segment
                x1, y1 = path_points[i]
                x2, y2 = path_points[i + 1]

                # Calculate distance to this segment
                dist = self.point_to_segment_distance(x, y, x1, y1, x2, y2)

                # Update minimum distance
                min_dist = min(min_dist, dist)

            return min_dist

    def point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        """Calculate distance from point to line segment."""
        # Line segment vector
        dx = x2 - x1
        dy = y2 - y1

        # If segment is just a point
        if dx == 0 and dy == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)

        # Calculate projection parameter
        t = ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)

        # Constrain t to segment bounds
        t = max(0, min(1, t))

        # Calculate closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # Return distance to closest point
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

    def create_all_obstacle(self):
        data = self.local_goal_manager_.data
        for i in range(len(data)-1):
            self.obstacle_creation(data[i], data[i+1], i=i)
        print("All obstacles created")
    # def create_all_obstacle(self):
    #
    #     for i, goal in enumerate(self.local_goal_manager_.data):
    #         current = self.local_goal_manager_.data[i]
    #         if i + 1 >= len(self.local_goal_manager_.data):
    #             return
    #         next = self.local_goal_manager_.data[i+1]
    #         self.obstacle_creation(current, next)
    #     print("All obstacles created")
    #     return
    def obstacle_creation(self, current_local_goal, next_local_goal, i=None):
        """
        Curvature-aware corridor: 
        - Inside wall is pulled in, outside wall is pushed out depending on curvature.
        - Adds optional jitter/variation for realism.
        - Enforces a minimum clearance from the path (so obstacles don't overlap with it).
        """

        # --- positions along this path segment ---
        x1 = current_local_goal.pose.position.x
        y1 = current_local_goal.pose.position.y
        x2 = next_local_goal.pose.position.x
        y2 = next_local_goal.pose.position.y

        # compute tangent (dx,dy) and perpendicular (px,py)
        dx, dy = x2 - x1, y2 - y1
        L = math.hypot(dx, dy)                 # segment length
        if L == 0.0:
            return                             # degenerate case (no movement)
        dx, dy = dx / L, dy / L                # unit tangent along path
        px, py = -dy, dx                       # unit normal (left side)
        mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0  # segment midpoint

        # --- curvature at this segment ---
        if i is None:
            # fallback: find index of current_local_goal in the list
            i = max(1, min(len(self.local_goal_manager_.data) - 2,
                           self.local_goal_manager_.data.index(current_local_goal)))
        kappa = self._signed_curvature(i)      # signed curvature (+ left, - right)

        # --- skew factor determines asymmetry between inside/outside walls ---
        # grows smoothly with |curvature| up to self.skew_s_max
        skew = self.skew_s_max * (1.0 - math.exp(-self.skew_gain * abs(kappa)))

        # --- nominal offsets for walls ---
        BASE = self.OFFSET                     # baseline corridor half-width
        # inside wall shrinks with skew, but not below min_inside*BASE
        off_in  = max(self.min_inside * BASE, BASE * (1.0 - skew))
        # outside wall widens with skew, capped at max_outmul*BASE
        off_out = min(self.max_outmul * BASE, BASE * (1.0 + self.widen_out * skew))

        # choose which side is "inside" based on curvature sign
        inside_s  = +1 if kappa > 0.0 else -1
        outside_s = -inside_s

        # --- helper: Gaussian noise generator for jitter ---
        def normal(std):
            # Box-Muller transform to sample from N(0,std^2)
            u1 = max(1e-9, self._rng.random())
            u2 = self._rng.random()
            return math.sqrt(-2.0 * math.log(u1)) * math.cos(2 * math.pi * u2) * std

        # --- radii for the two obstacles (varied slightly) ---
        r_lo, r_hi = self._radius_range
        rad_in  = self.RADIUS * (r_lo + (r_hi - r_lo) * self._rng.random())
        rad_out = self.RADIUS * (r_lo + (r_hi - r_lo) * self._rng.random())

        # --- obstacle centers (apply offset, plus jitter & along-track shift) ---
        cx_in  = mx + inside_s  * off_in  * px + normal(self._jitter_std) + normal(self._along_std) * dx
        cy_in  = my + inside_s  * off_in  * py + normal(self._jitter_std) + normal(self._along_std) * dy
        cx_out = mx + outside_s * off_out * px + normal(self._jitter_std) + normal(self._along_std) * dx
        cy_out = my + outside_s * off_out * py + normal(self._jitter_std) + normal(self._along_std) * dy

        # ------------------------------------------------------------------
        # ENFORCE MINIMUM CLEARANCE FROM THE PATH
        # (so obstacle centers don’t get too close to the path polyline)
        path_points = [(ps.pose.position.x, ps.pose.position.y)
                       for ps in self.local_goal_manager_.global_path.poses]

        def enforce_clearance(cx, cy, radius, mx, my, px, py):
            # Clearance rule: at least ~1.1–1.2× radius plus 3cm buffer
            min_clear = max(radius * 1.10, self.RADIUS * 1.20) + 0.03
            d = self.calculate_min_distance_to_path(cx, cy, path_points)
            need = min_clear - d
            if need > 0.0:
                # if too close, push outward along ±normal
                sign = math.copysign(1.0, (px * (cx - mx) + py * (cy - my)))
                cx += sign * need * px
                cy += sign * need * py
            return cx, cy

        # enforce clearance for both inside and outside
        cx_in,  cy_in  = enforce_clearance(cx_in,  cy_in,  rad_in,  mx, my, px, py)
        cx_out, cy_out = enforce_clearance(cx_out, cy_out, rad_out, mx, my, px, py)
        # ------------------------------------------------------------------

        # --- probabilistic dropouts (to create corridor gaps) ---
        if self._rng.random() >= self._drop_prob * 0.5:
            self.obstacle_array.append(Obstacle(cx_in,  cy_in,  rad_in))
        if self._rng.random() >= self._drop_prob:
            self.obstacle_array.append(Obstacle(cx_out, cy_out, rad_out))

        # --- occasional clutter near corridor (random extra obstacle) ---
        if self._rng.random() < self._clutter_prob:
            ang = 2 * math.pi * self._rng.random()
            rad = 0.2 + (self._clutter_rad - 0.2) * self._rng.random()
            self.obstacle_array.append(
                Obstacle(mx + rad * math.cos(ang),
                         my + rad * math.sin(ang),
                         self.RADIUS * (r_lo + (r_hi - r_lo) * self._rng.random()))
            )
    # def obstacle_creation(self, current_local_goal, next_local_goal):
    #     # mid-point
    #     x1 = current_local_goal.pose.position.x
    #     y1 = current_local_goal.pose.position.y
    #     x2 = next_local_goal.pose.position.x
    #     y2 = next_local_goal.pose.position.y
    #
    #     mx = (x1 + x2) / 2.0
    #     my = (y1 + y2) / 2.0
    #
    #     # direction (unit tangent)
    #     dx = x2 - x1
    #     dy = y2 - y1
    #     L = math.sqrt(dx*dx + dy*dy)
    #     if L == 0.0:
    #         return
    #     dx /= L
    #     dy /= L
    #
    #     # unit perpendicular
    #     px = -dy
    #     py = dx
    #
    #     # --- width modulation ---
    #     lo, hi = self._offset_range
    #     width_scale = lo + (hi - lo) * self._rng.random()
    #     off = self.OFFSET * width_scale
    #
    #     # occasional pinch (extra narrowing)
    #     if self._rng.random() < self._pinch_prob:
    #         p_lo, p_hi = self._pinch_scale
    #         off *= p_lo + (p_hi - p_lo) * self._rng.random()
    #
    #     # choose sides (both or single)
    #     keep_both = (self._rng.random() > self._single_side_prob)
    #     sides = [+1, -1] if keep_both else [self._rng.choice([+1, -1])]
    #
    #     def sample_radius():
    #         r_lo, r_hi = self._radius_range
    #         return self.RADIUS * (r_lo + (r_hi - r_lo) * self._rng.random())
    #
    #     def normal(std):
    #         # Box–Muller for a quick normal from uniform RNG
    #         u1 = max(1e-9, self._rng.random())
    #         u2 = self._rng.random()
    #         z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2*math.pi*u2)
    #         return z * std
    #
    #     # create 1–2 wall obstacles
    #     for s in sides:
    #         # base center on the perpendicular
    #         cx = mx + s * off * px
    #         cy = my + s * off * py
    #
    #         # jitter (XY) and along-path slide
    #         cx += normal(self._jitter_std) + normal(self._along_std) * dx
    #         cy += normal(self._jitter_std) + normal(self._along_std) * dy
    #
    #         r = sample_radius()
    #
    #         # random drop to create gaps
    #         if self._rng.random() < self._drop_prob:
    #             continue
    #
    #         ob = Obstacle(center_x=cx, center_y=cy, radius=r)
    #         self.obstacle_array.append(ob)
    #
    #     # occasional clutter near (not exactly on) the corridor
    #     if self._rng.random() < self._clutter_prob:
    #         ang = 2 * math.pi * self._rng.random()
    #         rad = 0.2 + (self._clutter_rad - 0.2) * self._rng.random()
    #         cx = mx + rad * math.cos(ang)
    #         cy = my + rad * math.sin(ang)
    #         r  = sample_radius()
    #         self.obstacle_array.append(Obstacle(center_x=cx, center_y=cy, radius=r))
    def obstacle_creation_sym(self, current_local_goal, next_local_goal):
        # creates obstacles symetric with no noise
        mid_x = (current_local_goal.pose.position.x +
                 next_local_goal.pose.position.x) / 2
        mid_y = (current_local_goal.pose.position.y +
                 next_local_goal.pose.position.y) / 2
        dir_x = next_local_goal.pose.position.x - current_local_goal.pose.position.x
        dir_y = next_local_goal.pose.position.y - current_local_goal.pose.position.y
        # Normalize
        length = math.sqrt(dir_x * dir_x + dir_y * dir_y)

        if (length > 0):
            dir_x /= length
            dir_y /= length

        perp_x = -dir_y
        perp_y = dir_x

        offset_x = perp_x * self.OFFSET
        offset_y = perp_y * self.OFFSET

        ob1 = Obstacle()
        ob1.center_x = mid_x + offset_x
        ob1.center_y = mid_y + offset_y
        ob1.radius = self.RADIUS

        ob2 = Obstacle()
        ob2.center_x = mid_x - offset_x
        ob2.center_y = mid_y - offset_y
        ob2.radius = self.RADIUS

        self.obstacle_array.append(ob1)
        self.obstacle_array.append(ob2)


class Local_Goal_Manager():
    def __init__(self, current_odom, global_path=None):
        self.current_odom = (None, None)
        self.data = []
        self.global_path = global_path
        self.current_lg_counter = 0
        self.current_odom = current_odom

    def generate_local_goals_claude(self, global_path):
        """
        Modified to create more local goals in areas with high curvature
        """
        if self.global_path is None:
            print("Cannot generate local goals: No global path available")
            self.global_path = global_path
            return

        accumulated_distance = 0.0
        base_threshold = 0.2
        lo, hi = 0.15, 0.25
        goal_counter = 0
        rand_spacing = random.uniform(lo, hi)
        print(f"len of global_path : {len(self.global_path.poses)}")
        for i in range(len(self.global_path.poses)-1):
            current_pose = self.global_path.poses[i]
            next_pose = self.global_path.poses[i+1]

            # Calculate direction change if not at beginning
            curvature_factor = 1.0
            if i > 0:
                prev_pose = self.global_path.poses[i-1]
                prev_dir_x = current_pose.pose.position.x - prev_pose.pose.position.x
                prev_dir_y = current_pose.pose.position.y - prev_pose.pose.position.y
                curr_dir_x = next_pose.pose.position.x - current_pose.pose.position.x
                curr_dir_y = next_pose.pose.position.y - current_pose.pose.position.y

                # Normalize vectors
                prev_len = math.sqrt(prev_dir_x**2 + prev_dir_y**2)
                curr_len = math.sqrt(curr_dir_x**2 + curr_dir_y**2)

                if prev_len > 0 and curr_len > 0:
                    prev_dir_x /= prev_len
                    prev_dir_y /= prev_len
                    curr_dir_x /= curr_len
                    curr_dir_y /= curr_len

                    # Dot product gives cosine of angle between vectors
                    dot_product = prev_dir_x * curr_dir_x + prev_dir_y * curr_dir_y
                    angle = math.acos(max(-1.0, min(1.0, dot_product)))

                    # Adjust threshold based on curvature (smaller threshold = more points)
                    curvature_factor = max(0.3, 1.0 - angle/math.pi)

            # Calculate distance with adaptive threshold
            segment_distance = self.distance_between_poses(
                current_pose, next_pose)
            adaptive_threshold = base_threshold * curvature_factor
            # print(f"accumulated_distance is {accumulated_distance}")
            accumulated_distance += segment_distance
            if accumulated_distance >= adaptive_threshold:
                self.data.append(current_pose)
                accumulated_distance = 0
                # goal_counter+=1
                # if goal_counter % 5 == 0:
                #     rand_spacing = random.uniform(lo, hi)
        print(f"created local goals: {len(self.data)}")

    def generate_local_goals(self, global_path):
        """
        Generate local goals along the global path based on the current robot position
        """
        if self.global_path is None:
            print("Cannot generate local goals: No global path available")
            self.global_path = global_path
            return

        # Find the closest point on the path to the current robot position

        # Start from the closest point and generate local goals along the path

        accumulated_distance = 0.0
        threshold_distance = .08

        for i in range(len(self.global_path.poses)-1):
            current_pose = self.global_path.poses[i]
            next_pose = self.global_path.poses[i+1]

            # Calculate distance between consecutive points
            segment_distance = self.distance_between_poses(
                current_pose, next_pose)
            print(segment_distance)
            accumulated_distance += segment_distance
            # Add points at regular intervals
            if accumulated_distance >= threshold_distance:
                print("Add a local goal")
                self.data.append(current_pose)
                accumulated_distance = 0

        print(f"Generated {len(self.data)} local goals")
        self.current_lg = (self.data[self.current_lg_counter].pose.position.x,
                           self.data[self.current_lg_counter].pose.position.y)

    def distance_between_points(self, point1, point2):
        """
        Calculate Euclidean distance between two points (x1, y1) and (x2, y2)
        """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def distance_between_poses(self, pose1, pose2):
        """
        Calculate distance between two PoseStamped messages
        """
        return self.distance_between_points(
            (pose1.pose.position.x, pose1.pose.position.y),
            (pose2.pose.position.x, pose2.pose.position.y)
        )

    def update_claude(self, current_odom=None):
        if self.global_path is None or current_odom[0] is None:
            return

        self.current_odom = current_odom  # Make sure to update this field
        self.current_lg = (self.data[self.current_lg_counter].pose.position.x,
                           self.data[self.current_lg_counter].pose.position.y)
        dist_to_goal = self.distance_between_points(
            self.current_lg, current_odom)

        # Look ahead more dynamically
        look_ahead = 3  # Check several goals ahead
        best_index = self.current_lg_counter
        best_dist = dist_to_goal

        # Check if any of the next few local goals are closer
        for i in range(1, look_ahead + 1):
            next_index = self.current_lg_counter + i
            if next_index < len(self.data):
                next_lg = (
                    self.data[next_index].pose.position.x,
                    self.data[next_index].pose.position.y,
                )
                next_dist = self.distance_between_points(next_lg, current_odom)

                if next_dist < best_dist:
                    best_dist = next_dist
                    best_index = next_index

        # Update to the best local goal found
        if best_index != self.current_lg_counter:
            self.current_lg_counter = best_index
            self.current_lg = (
                self.data[self.current_lg_counter].pose.position.x,
                self.data[self.current_lg_counter].pose.position.y,
            )
        elif dist_to_goal < 0.05:  # If we're close to the current goal
            self.current_lg_counter += 1
            if self.current_lg_counter < len(self.data):
                self.current_lg = (
                    self.data[self.current_lg_counter].pose.position.x,
                    self.data[self.current_lg_counter].pose.position.y,
                )

        return self.current_lg_counter

    def update(self, current_odom=None):
        # Comparing poses so might not work
        if self.global_path is None or self.current_lg is None or self.current_odom[0] is None:
            return
        self.current_odom = current_odom  # updating every call
        dist_to_goal = self.distance_between_points(
            self.current_lg, current_odom)

        if dist_to_goal < .05:
            self.current_lg_counter += 1
            if self.current_lg_counter < len(self.data):
                if self.data[self.current_lg_counter].pose.position.x:
                    self.current_lg = (self.data[self.current_lg_counter].pose.position.x,
                                       self.data[self.current_lg_counter].pose.position.y)
            print("Have updated current local goal:")
        else:
            # Check if the *next* local goal is even closer
            next_lg_counter = self.current_lg_counter + 1
            if next_lg_counter < len(self.data):
                next_lg = (
                    self.data[next_lg_counter].pose.position.x,
                    self.data[next_lg_counter].pose.position.y,
                )
                dist_to_next = self.distance_between_points(
                    next_lg, current_odom)

                if dist_to_next < dist_to_goal:
                    print(
                        f"Jumping to closer local goal {next_lg_counter} (dist: {dist_to_next:.4f})")
                    self.current_lg_counter = next_lg_counter
                    self.current_lg = next_lg
            print(
                f"Have yet to reach local goal: dist to lg = {dist_to_goal:.4f}")
            print(f"Currently at local goal index: {self.current_lg_counter}")
            return self.current_lg_counter

    def get_local_goal_counter(self):
        return self.current_lg_counter

    def get_coords(self):
        return (self.current_lg[0], self.current_lg[1])

    def set_global_path(self, new_global_path):
        if new_global_path is not None and len(new_global_path.poses) > 0:
            self.global_path = new_global_path
            self.current_lg_counter = 0
            return True
        return False

    def upscale_local_goal(self, start_lg, map_points, output_csv):

        currentLocalGoal = (start_lg[0], start_lg[1])
        lg_upsampled = []
        lg_yaw_upsampled = []
        lgCounter = 0
        odomCounter = 0
        while len(map_points) != len(lg_upsampled):
            odomPoint = (map_points[odomCounter][0],
                         map_points[odomCounter][1])
            if odomPoint == currentLocalGoal:
                # if have reached the local goal, update to next local goal
                if lgCounter + 1 < len(self.data):
                    lgCounter += 1
                    currentLocalGoal = (
                        self.data[lgCounter].pose.position.x, self.data[lgCounter].pose.position.y)
                    print("success, local goal has been reached")

            yaw = self.get_yaw(self.data[lgCounter].pose)
            lg_yaw_upsampled.append(yaw)
            lg_upsampled.append(currentLocalGoal)
            odomCounter += 1
            print("len of lg_upsampled", len(lg_upsampled),
                  len(map_points), lgCounter, len(self.data))
        print(len(lg_upsampled) == len(map_points))

        with open(output_csv, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                ['local_goals_x', 'local_goals_y', 'local_goals_yaw'])

            for i in range(len(lg_upsampled)):
                writer.writerow(
                    [lg_upsampled[i][0], lg_upsampled[i][1], lg_yaw_upsampled[i]])

    def get_yaw(self, pose: Pose) -> float:
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        # _, _, yaw = euler_from_quaternion(quat)
        _, _, yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return yaw


def odom_to_map(node, odom_x, odom_y, odom_frame='odom', map_frame='map'):
    """
    Convert coordinates from odometry frame to map frame using ROS2 TF2.

    Parameters:
    - node: ROS2 node instance
    - odom_x: x coordinate in odometry frame
    - odom_y: y coordinate in odometry frame
    - odom_frame: name of the odometry frame (default: 'odom')
    - map_frame: name of the map frame (default: 'map')

    Returns:
    - (map_x, map_y): coordinates in map frame
    - None if transformation failed
    """
    # Create point in odometry frame
    point = PointStamped()
    point.header.stamp = rclpy.time.Time().to_msg()

    point.header.frame_id = odom_frame
    point.point.x = float(odom_x)
    point.point.y = float(odom_y)
    point.point.z = 0.0

    try:
        # Transform point from odom frame to map frame
        transformed_point = node.tf_buffer.transform(
            point, map_frame, rclpy.duration.Duration(seconds=0.5))
        return transformed_point.point.x, transformed_point.point.y
    except TransformException as ex:
        node.get_logger().warn(
            f"Could not transform point from {odom_frame} to {map_frame}: {ex}")
        return None


class MapTraining(Node):
    def __init__(self, input_bag_path=None):
        super().__init__('map_training_node')

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data setup
        self.odom_x = None
        self.odom_y = None
        self.map_x = []
        self.map_y = []
        self.map_points = []
        self.global_path = None
        self.shutdown_requested = False
        self.current_odom = (0.0, 0.0)
        self.OFFSET = 0
        self.RADIUS = 0
        self.NUM_VALID_OBS = 20
        self.NUM_LIDAR = 1080
        self.Obstacle_list = []

        self.SENSOR_YAW_OFFSET = -math.pi/2   # lidar x-axis relative to robot x-axis
        self.FOV_MIN = -math.pi               # or -math.pi/2 if you’re doing a 180° forward wedge
        self.FOV_MAX =  math.pi               # or  math.pi/2 for 180°
        self.LIDAR_MIN_R = 0.164
        self.LIDAR_MAX_R = 12.0
        self.Obstacle_list = []
        self.dist_between_goals = .2
        # Delay the odom-to-map conversion until TF is ready
        self.odom_timer = self.create_timer(2.0, self.check_tf_and_run)

        self.distances = [0] * self.NUM_LIDAR

        self.lidar_header_flag = True
        # Files for training data to be stored

        if input_bag_path:
            self.input_bag = input_bag_path
        else:
            # Keep your default for backward compatibility
            self.input_bag = "/home/mobrob/ros_ws/gauss_2_asym_cap/2025-08-21_19-38-54_gaus"
        # self.input_bag = "/home/mobrob/ros_ws/ros_bag/gauss_2/2025-08-30_15-40-10_gaus"

        self.yaml_reader()
        self.write_meta_data()
        self.frame_dkr = f"{self.input_bag}/input_data/"
        os.makedirs(self.frame_dkr, exist_ok=True)
        self.odom_csv_file = os.path.join(self.frame_dkr, "odom_data.csv")
        self.cmd_csv = os.path.join(self.frame_dkr, "cmd_vel.csv")
        self.lidar_file = os.path.join(self.frame_dkr, "lidar_data.csv")

        self.local_goals_output = os.path.join(
            self.frame_dkr, "local_goals.csv")
        self.cmd_output_csv = os.path.join(
            self.frame_dkr, "cmd_vel_output.csv")

    def validate_obstacles(self):
        output_folder = "obstacle_validation"
        os.makedirs(output_folder, exist_ok=True)

        segment_size = 20  # Number of odom points per segment
        window_size = 8    # Number of local goals to include in each window

        # Iterate through the odom path in segments
        for segment_start in range(0, len(self.map_points), segment_size):
            segment_end = min(segment_start + segment_size,
                              len(self.map_points))

            # Create a figure for this segment
            plt.figure(figsize=(12, 8))

            # Get the current segment of the path
            segment_points = self.map_points[segment_start:segment_end]
            segment_x = [point[0] for point in segment_points]
            segment_y = [point[1] for point in segment_points]

            # Plot the entire path (lightly) for context
            path_x = [point[0] for point in self.map_points]
            path_y = [point[1] for point in self.map_points]
            plt.plot(path_x, path_y, 'k-', linewidth=0.5,
                     alpha=0.2, label='Full Path')

            # Highlight the current segment
            plt.plot(segment_x, segment_y, 'b-', linewidth=2,
                     label=f'Segment {segment_start//segment_size + 1}')

            # Find center point of this segment
            if segment_points:
                segment_center_x = sum(
                    point[0] for point in segment_points) / len(segment_points)
                segment_center_y = sum(
                    point[1] for point in segment_points) / len(segment_points)
                segment_center = (segment_center_x, segment_center_y)

                # Find the closest local goal to the segment center
                closest_lg_idx = 0
                min_dist = float('inf')

                for i, goal in enumerate(self.local_goal_manager_.data):
                    goal_pos = (goal.pose.position.x, goal.pose.position.y)
                    dist = math.sqrt((goal_pos[0] - segment_center[0])**2 +
                                     (goal_pos[1] - segment_center[1])**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_lg_idx = i

                # Create a window of local goals centered around the closest one
                window_start = max(0, closest_lg_idx - window_size // 2)
                window_end = min(
                    len(self.local_goal_manager_.data), window_start + window_size)

                # If we're at the end, adjust the window start to maintain the window size
                if window_end - window_start < window_size:
                    window_start = max(0, window_end - window_size)

                window_local_goals = self.local_goal_manager_.data[window_start:window_end]

                # Plot the relevant local goals
                lg_x = [goal.pose.position.x for goal in window_local_goals]
                lg_y = [goal.pose.position.y for goal in window_local_goals]
                plt.scatter(lg_x, lg_y, c='blue', s=50, label='Local Goals')

                # Plot segments between consecutive local goals
                for i in range(len(window_local_goals) - 1):
                    current = window_local_goals[i]
                    next_goal = window_local_goals[i+1]
                    plt.plot([current.pose.position.x, next_goal.pose.position.x],
                             [current.pose.position.y, next_goal.pose.position.y],
                             'g-', linewidth=2, alpha=0.7)

                # Find obstacles associated with these local goals
                relevant_obstacles = []
                for i in range(window_start, window_end - 1):
                    # Get the two obstacles associated with this local goal segment
                    obs_idx_start = i * 2
                    if obs_idx_start + 1 < len(self.obstacle_manager_.obstacle_array):
                        relevant_obstacles.append(
                            self.obstacle_manager_.obstacle_array[obs_idx_start])
                        relevant_obstacles.append(
                            self.obstacle_manager_.obstacle_array[obs_idx_start + 1])

                # Plot the relevant obstacles
                for obstacle in relevant_obstacles:
                    # Draw the obstacle
                    circle = plt.Circle((obstacle.center_x, obstacle.center_y),
                                        radius=obstacle.radius,
                                        fill=False,
                                        color='red',
                                        linewidth=1)
                    plt.gca().add_patch(circle)

                # Connect obstacles to their midpoints
                for i in range(window_start, window_end - 1):
                    if i * 2 + 1 < len(self.obstacle_manager_.obstacle_array):
                        current_lg = self.local_goal_manager_.data[i]
                        next_lg = self.local_goal_manager_.data[i+1]

                        # Calculate midpoint of the segment
                        mid_x = (current_lg.pose.position.x +
                                 next_lg.pose.position.x) / 2
                        mid_y = (current_lg.pose.position.y +
                                 next_lg.pose.position.y) / 2

                        # Draw the midpoint
                        plt.scatter(mid_x, mid_y, c='purple', s=30)

                        # Draw connecting lines to obstacles
                        obstacle1 = self.obstacle_manager_.obstacle_array[i * 2]
                        obstacle2 = self.obstacle_manager_.obstacle_array[i * 2 + 1]

                        plt.plot([mid_x, obstacle1.center_x], [mid_y, obstacle1.center_y],
                                 'm-', linewidth=1, alpha=0.7)
                        plt.plot([mid_x, obstacle2.center_x], [mid_y, obstacle2.center_y],
                                 'm-', linewidth=1, alpha=0.7)

                # Label the first and last points of the segment
                plt.scatter(segment_points[0][0], segment_points[0]
                            [1], c='green', s=100, label='Segment Start')
                plt.scatter(
                    segment_points[-1][0], segment_points[-1][1], c='red', s=100, label='Segment End')

            # Add segment range to the title
            plt.title(
                f'Obstacle Validation - Odom Points {segment_start} to {segment_end-1}')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')

            # Save this segment's plot
            plt.savefig(
                f"{output_folder}/validation_segment_{segment_start:04d}_{segment_end-1:04d}.png", dpi=300)
            plt.close()

        print(f"Saved obstacle validation plots to {output_folder}/")

    def check_tf_and_run(self):
        # Try checking for transform once TF listener has had time to populate
        if self.tf_buffer.can_transform(
            'map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
        ):
            self.get_logger().info(
                "TF transform map → odom is now available. Proceeding with data transformation.")
            self.odom_timer.cancel()  # Stop calling this timer
            self.odom_data()  # Run the conversion
        else:
            self.get_logger().warn("TF not ready: waiting for map → odom...")

    def odom_data(self):

        # turned bag into csv, this is in the odom frame
        self.save_to_csv(self.input_bag, self.odom_csv_file, '/odom')

        self.save_to_csv(self.input_bag, self.cmd_csv,
                         '/cmd_vel')  # turned bag into csv
        self.oversample_cmdVel3(
            self.odom_csv_file, self.cmd_csv, self.cmd_output_csv)

        df = pd.read_csv(self.odom_csv_file)
        self.odom_x = df['odom_x'].tolist()
        self.odom_y = df['odom_y'].tolist()

        for i, (x_odom, y_odom) in enumerate(zip(self.odom_x, self.odom_y)):
            result = odom_to_map(self, x_odom, y_odom)
            if result is not None:
                x_map, y_map = result
                self.map_x.append(x_map)
                self.map_y.append(y_map)
            else:
                self.get_logger().warn(
                    f"Skipping index {i} due to transform failure.")
                self.map_x.append(None)
                self.map_y.append(None)

        self.segment_setup()
        # self.setup()
        self.get_logger().info(
            f"Transformed {len(self.map_x)} points to map frame.")
        self.get_logger().info("Shutdown requested from odom callback")
        self.shutdown_requested = True  
        sys.exit(0)

    def visualize_path_with_yaw(self, sample_rate=20):
        """
        Create a visualization of the path with yaw arrows every sample_rate positions.

        Args:
            sample_rate: Plot yaw arrows every sample_rate positions
        """
        output_folder = "path_with_yaw"
        os.makedirs(output_folder, exist_ok=True)

        plt.figure(figsize=(12, 8))
        ax = plt.gca()
        ax.set_aspect('equal')

        # Plot the full path
        path_x = [point[0] for point in self.map_points]
        path_y = [point[1] for point in self.map_points]
        plt.plot(path_x, path_y, '-', color='blue',
                 linewidth=1.0, alpha=0.7, label='Path')

        # Extract and plot yaw arrows
        yaw_length = 0.3  # Length of the arrow

        for i in range(0, len(self.global_path.poses), sample_rate):
            pos = self.global_path.poses[i].pose.position
            orientation = self.global_path.poses[i].pose.orientation
            _, _, yaw = euler_from_quaternion([orientation.x, orientation.y,
                                               orientation.z, orientation.w])

            # Plot position point
            plt.plot(pos.x, pos.y, 'o', color='black', markersize=3)

            # Plot yaw arrow
            dx = yaw_length * math.cos(yaw)
            dy = yaw_length * math.sin(yaw)
            plt.arrow(pos.x, pos.y, dx, dy,
                      head_width=0.08, head_length=0.15,
                      fc='red', ec='red')

            # Optionally add text labels for angles
            if i % (sample_rate * 5) == 0:  # Add labels less frequently
                angle_degrees = math.degrees(yaw) % 360
                plt.text(pos.x + dx + 0.05, pos.y + dy + 0.05,
                         f"{angle_degrees:.1f}°",
                         fontsize=8, color='darkred')

        plt.title(f'Path with Yaw Angles (every {sample_rate} positions)')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Save the visualization
        plt.savefig(f"{output_folder}/path_with_yaw.png", dpi=300)
        plt.close()

        print(
            f"Saved path with yaw visualization to {output_folder}/path_with_yaw.png")

    def segment_setup(self):

        self.map_points = list(zip(self.map_x, self.map_y))

        self.global_path = self.create_path_from_points(self.map_points)
        self.local_goal_manager_ = Local_Goal_Manager(self.current_odom)
        self.local_goal_manager_.global_path = self.global_path
        self.local_goal_manager_.generate_local_goals_claude(self.global_path)

        self.local_goal_manager_.upscale_local_goal(
            (self.local_goal_manager_.data[0].pose.position.x, self.local_goal_manager_.data[0].pose.position.y), self.map_points, self.local_goals_output)

        self.current_odom = (self.map_points[0][0], self.map_points[0][1])
        self.segments = self.create_segments(self.map_points)

        # self.create_excels(self.segments), only do this if not cached

        # Does not work, must do one by one ??, need to fix
        # self.per_seg_loop_once(self.segments[2], 2)
        for i, seg in enumerate(self.segments):
            print(
                f"checking start and end index : {seg.start_index} and {seg.end_index}")
            self.per_seg_loop_once(seg, i)
            print(f"done with seg : {i}")
        # self.main_loop()

    def setup(self):
        # self.test_obs_700()
        self.map_points = list(zip(self.map_x, self.map_y))
        self.global_path = self.create_path_from_points(self.map_points)

        self.current_odom = (self.map_points[0][0], self.map_points[0][1])

        self.local_goal_manager_ = Local_Goal_Manager(self.current_odom)
        self.local_goal_manager_.global_path = self.global_path
        self.local_goal_manager_.generate_local_goals_claude(self.global_path)

        self.local_goal_manager_.upscale_local_goal(
            (self.local_goal_manager_.data[0].pose.position.x, self.local_goal_manager_.data[0].pose.position.y), self.map_points, self.local_goals_output)

        self.obstacle_manager_ = Obstacle_Manager(
            self.OFFSET, self.RADIUS, self.local_goal_manager_)
        self.obstacle_manager_.create_all_obstacle()
        # self.validate_obstacles()
        print("VALIDATED ****************************")
        print(
            f"FIRST POINT IS ********************************8 {self.map_points[0]}")

        print("checking if a local goal exists in map_points")
        # Check if a specific local goal exists in map pointsp
        found, matching_point = self.is_pose_in_map_points(
            self.local_goal_manager_.data[0], self.map_points)
        if found:
            print(f"local goal exists within map points at {matching_point}")
        else:
            print("error local goal not in map points list")

        output_folder = "obstacle_and_path_2"
        os.makedirs(output_folder, exist_ok=True)
        plt.figure(figsize=(8, 6))

        plt.clf()  # Clear previous plot
        ax = plt.gca()
        ax.set_aspect('equal')
        # Replot the base elements
        # plt.plot(self.current_odom[0], self.current_odom[1], marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
        active_obstacle = self.obstacle_manager_.get_active_obstacles_claude()
        for obstacle in self.obstacle_manager_.obstacle_array:
            circle = patches.Circle(
                (obstacle.center_x, obstacle.center_y),
                radius=obstacle.radius,
                fill=False,
                color='red',
                linewidth=1.5,
                linestyle='-'
            )
            ax.add_patch(circle)

        path_x = [point[0] for point in self.map_points]
        path_y = [point[1] for point in self.map_points]

        # Plot the entire path
        plt.plot(path_x, path_y, marker='o', linestyle='-',
                 markersize=3, color='black', label='odom path')

        frame_path = f"{output_folder}/obst_path.png"
        plt.savefig(frame_path)

        print("Saved png of obstacles and path")
        self.visualize_path_with_yaw()
        # self.segments = self.create_segments(self.map_points, self.global_path)
        self.per_seg_loop(self.segments)
        # self.main_loop()

    def create_excels(self, segments):

        for i, seg in enumerate(segments):
            output_folder = f"{self.input_bag}/seg_{i}/input_data"

            os.makedirs(output_folder, exist_ok=True)

            odom_all = pd.read_csv(self.odom_csv_file)
            odom_curr = odom_all[seg.start_index:seg.end_index]
            odom_curr.to_csv(f"{output_folder}/odom_data.csv")

            cmd_all = pd.read_csv(self.cmd_output_csv)
            cmd_curr = cmd_all[seg.start_index:seg.end_index]
            cmd_curr.to_csv(f"{output_folder}/cmd_vel_output.csv")

            local_goal_all = pd.read_csv(self.local_goals_output)
            local_goal_curr = local_goal_all[seg.start_index:seg.end_index]
            local_goal_curr.to_csv(f"{output_folder}/local_goals.csv")

            print(
                f"Segment: start_index={seg.start_index}, end_index={seg.end_index}")

        print("created input csvs")

    def per_seg_loop_once(self, seg, seg_index):
        """
        Args: Segment
        Output: Nothing, data just written to CSV
        Only creates lidar data
        """
        output_folder = f"{self.input_bag}/seg_{seg_index}/input_data"
        os.makedirs(output_folder, exist_ok=True)

        self.current_odom_index = seg_index
        counter = 0

        self.lidar_header_flag = True

        lidar_file = f"{output_folder}/lidar_data.csv"
        if os.path.exists(lidar_file):
            os.remove(lidar_file)
            print("lidar file")

        frames_folder = f"{output_folder}/frames"
        os.makedirs(frames_folder, exist_ok=True)

        for i, map_point in enumerate(seg.map_points):

            path_x = [point[0] for point in seg.map_points[i:i+1000]]
            path_y = [point[1] for point in seg.map_points[i:i+1000]]
            self.current_odom = (map_point[0], map_point[1])
            seg.local_goal_manager_.current_odom = self.current_odom
            active_obstacles = seg.get_obstacles(i)
            ray_data = self.ray_tracing_capped(
                seg.global_path.poses[i].pose, seg, active_obstacles)
            self.ray_data_append(filename=f"{output_folder}/lidar_data.csv")

            if i % 500 != 0:
                counter += 1
                self.current_odom_index += 1
                continue
            plt.clf()  # clear previous plot
            ax = plt.gca()
            ax.set_aspect('equal')
            # replot the base elements
            plt.plot(self.current_odom[0], self.current_odom[1], marker='o',
                     linestyle='-', markersize=3, color='blue', label="odometry path")


            print("local goal count")
            for obstacle in active_obstacles:
                circle = patches.Circle(
                (obstacle.center_x, obstacle.center_y),
                radius=obstacle.radius,
                fill=False,
                color='red',
                linewidth=1.5,
                linestyle='-'
            )
                ax.add_patch(circle)

            plt.scatter(
                self.current_odom[0], self.current_odom[1], color='cyan', s=200, label='robot')

            # plot the entire path
            plt.plot(path_x, path_y, marker='o', linestyle='-',
                     markersize=3, color='black', label='odom path')
            self.draw_rays_claude_2(
                seg.global_path.poses[i].pose.position.x, seg.global_path.poses[i].pose.position.y, ray_data, self.get_yaw(seg.global_path.poses[i].pose))
            # save the frame

            frame_path = f"{frames_folder}/frame_{counter:03d}.png"
            counter+=1
            plt.savefig(frame_path)

        plt.close()
        self.current_odom_index += 1
        print("done with main loop")

    def create_segments(self, map_points):
        """
        Create segments which act like little global paths, so need to create obstacles for just that segment

        """
        print("starting create_segments")
        segments = []
        current_segment = [map_points[0]]
        start_index = 0
        end_index = 0
        threshold = .15
        cache_filename = f"{self.input_bag}/segments_cache.csv"
        if os.path.exists(cache_filename):
            with open(cache_filename, 'r') as file:
                csv_reader = csv.reader(file)

                header = next(csv_reader)
                print(f"header: {header}")

                for row in csv_reader:

                    start_index = int(row[1])
                    end_index = int(row[2])
                    curr_seg = Segment(
                        self.map_points[start_index:end_index+1], self, self.RADIUS, self.OFFSET, start_index, end_index)
                    curr_seg.init()
                    segments.append(curr_seg)

        else:
            print("values not cached yet")
            for i in range(1, len(map_points)):

                current_segment.append(map_points[i])

                for j in range(len(current_segment) - 200):
                    if self.distance_between_points(current_segment[j], map_points[i]) < threshold:
                        end_index = i
                        seg_seed = end_index + BASE_SEED
                        curr_seg_ = Segment(
                            current_segment, self, self.RADIUS, self.OFFSET, start_index, end_index, seg_seed)
                        curr_seg_.init()
                        start_index = i
                        segments.append(curr_seg_)
                        current_segment = [map_points[i]]

                        break
            if current_segment:

                print(
                    f"last segment : start index {start_index} and end {len(map_points)-1}")
                print("*************************************")
                end_index = len(map_points) - 1
                seg_seed = BASE_SEED + end_index
                curr_seg_ = Segment(
                    current_segment, self, self.RADIUS, self.OFFSET,
                    start_index, end_index, seg_seed
                )
                curr_seg_.init()
                segments.append(curr_seg_)

            with open(cache_filename, 'w') as file:
                csv_writer = csv.writer(file)

                csv_writer.writerow(["seg_id", "start_index", "end_index"])
                for i, seg in enumerate(segments):
                    csv_writer.writerow(
                        [f"seg_{i}", seg.start_index, seg.end_index])

                print(f"finish caching results at {cache_filename}")
                self.create_excels(segments)
        self.plot_segments(segments)

        return segments

    def plot_segments(self, segments):
        plt.figure(figsize=(10, 8))

        # Define a colormap to get distinct colors for each segment
        colors = plt.cm.jet(np.linspace(0, 1, len(segments)))

        for i, segment in enumerate(segments):
            path_x = [point[0] for point in segment.map_points]
            path_y = [point[1] for point in segment.map_points]

            # Plot each segment with a different color and add to legend
            plt.plot(path_x, path_y, marker='o', linestyle='-', markersize=3,
                     color=colors[i], label=f'Segment {i+1}')

            # Optionally mark the start and end of each segment
            # Start point (green)
            plt.plot(path_x[0], path_y[0], 'go', markersize=6)
            plt.plot(path_x[-1], path_y[-1], 'ro',
                     markersize=6)  # End point (red)

            # would also like to add the obstacles of this segment
            for obstacle in segment.obstacle_manager_.obstacle_array:
                circle = plt.Circle((obstacle.center_x, obstacle.center_y),
                                    radius = obstacle.radius,
                                    fill = False,
                                    color='red',
                                    linewidth=1)
                plt.gca().add_patch(circle)
                        

        plt.title('Path Segments')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.legend(loc='best')
        plt.tight_layout()
        plt.savefig(os.path.join(self.input_bag, "odom_plot.png"))
        # plt.show()
        return plt.gcf()  # Return the figure if you want to save it later

    # def draw_rays_claude_2(self, odom_x, odom_y, lidar_readings, segment):
    #     # Get robot's yaw from the current pose (you need to pass this as a parameter)
    #     robot_yaw = self.get_yaw(
    #         segment.global_path.poses[self.current_odom_index].pose)
    #
    #     # Define lidar offset relative to robot (90 degrees = π/2 radians)
    #     lidar_offset = -math.pi/2
    #
    #     # Draw rays
    #     for lidar_counter in range(self.NUM_LIDAR):
    #         # Calculate ray angle in the global frame
    #         lidar_angle = lidar_counter * (2*np.pi / self.NUM_LIDAR)
    #         global_angle = robot_yaw + lidar_angle + lidar_offset
    #         global_angle = self.normalize_angle(global_angle)
    #
    #         distance = lidar_readings[lidar_counter]
    #
    #         # Ensure reasonable distance values
    #         if distance <= 0.001 or distance >= 12.0:  # Likely invalid value
    #             continue
    #
    #         # Draw a single line from robot to endpoint using the global angle
    #         projection_x = odom_x + distance * math.cos(global_angle)
    #         projection_y = odom_y + distance * math.sin(global_angle)
    #
    #         # Individual rays should be single lines, not connected
    #
    #         plt.plot([odom_x, projection_x], [odom_y, projection_y],
    #                  linestyle='-', color='green', linewidth=0.5)
    #
    #     # Optionally draw the robot's orientation and lidar frame
    #     arrow_length = 1.0
    #
    #     # Draw robot orientation (in red)
    #     dx_robot = arrow_length * math.cos(robot_yaw)
    #     dy_robot = arrow_length * math.sin(robot_yaw)
    #     plt.arrow(odom_x, odom_y, dx_robot, dy_robot,
    #               head_width=0.1, head_length=0.15, fc='red', ec='red', label='Robot Heading')
    #
    #     # Draw lidar orientation (in orange)
    #     lidar_direction = robot_yaw + lidar_offset
    #     dx_lidar = arrow_length * math.cos(lidar_direction)
    #     dy_lidar = arrow_length * math.sin(lidar_direction)
    #     plt.arrow(odom_x, odom_y, dx_lidar, dy_lidar,
    #               head_width=0.1, head_length=0.15, fc='black', ec='black', label='Lidar Direction')
    #
    #     print("Done drawing rays")
    def draw_rays_claude_2(self, odom_x, odom_y, scan, yaw):
        OFF = -math.pi/2
        RMIN, RMAX = 0.164, 12.0
        inc = 2*math.pi / self.NUM_LIDAR   # must match tracer

        for k, r in enumerate(scan):
            if r <= RMIN + 1e-6 or r >= RMAX - 1e-6:
                continue
            theta = yaw + OFF + k*inc
            x = odom_x + r*math.cos(theta)
            y = odom_y + r*math.sin(theta)
            plt.plot([odom_x, x], [odom_y, y], '-', color='green', linewidth=0.5)

        # arrows: robot +x (red) and beam 0 / angle_min (black)
        plt.arrow(odom_x, odom_y, math.cos(yaw),         math.sin(yaw),
                  head_width=0.1, head_length=0.15, fc='red', ec='red')
        plt.arrow(odom_x, odom_y, math.cos(yaw+OFF),     math.sin(yaw+OFF),
                  head_width=0.1, head_length=0.15, fc='black', ec='black')
        plt.gca().set_aspect('equal', adjustable='box')
    # def draw_rays_claude_2(self, odom_x, odom_y, lidar_readings, segment):
    #     yaw = self.get_yaw(segment.global_path.poses[self.current_odom_index].pose)
    #
    #     FOV_MIN = self.FOV_MIN
    #     FOV_MAX = self.FOV_MAX
    #     RMIN = self.LIDAR_MIN_R
    #     RMAX = self.LIDAR_MAX_R
    #
    #     n = len(lidar_readings)
    #     if n < 2:
    #         return
    #     inc = (2*math.pi) / self.NUM_LIDAR  # == (FOV_MAX - FOV_MIN)/(n-1)
    #     OFF= -math.pi /2.0
    #     for i, r in enumerate(lidar_readings):
    #         if r <= RMIN + 1e-6 or r >= RMAX - 1e-6:
    #             continue
    #
    #         # angle in SENSOR frame, then to WORLD with yaw + mounting offset
    #         ang_sensor = i * inc
    #         ang_world  = self.normalize_angle(yaw + OFF + ang_sensor)
    #
    #         x = odom_x + r * math.cos(ang_world)
    #         y = odom_y + r * math.sin(ang_world)
    #         plt.plot([odom_x, x], [odom_y, y], '-', color='green', linewidth=0.5)
    #
    #     # robot +x (forward) arrow in red
    #     plt.arrow(odom_x, odom_y, math.cos(yaw), math.sin(yaw),
    #               head_width=0.1, head_length=0.15, fc='red', ec='red', label='Robot heading')
    #
    #     # angle_min direction in black (exactly matches how we index rays)
    #     ang_min_world = self.normalize_angle(yaw + OFF + FOV_MIN)
    #     plt.arrow(odom_x, odom_y, math.cos(ang_min_world), math.sin(ang_min_world),
    #               head_width=0.1, head_length=0.15, fc='black', ec='black', label='LiDAR angle_min')
    #
    #     plt.gca().set_aspect('equal', adjustable='box')
    #
    # def draw_rays_claude_2(self, odom_x, odom_y, lidar_readings, segment):
    #     # current pose/yaw
    #     robot_yaw = self.get_yaw(segment.global_path.poses[self.current_odom_index].pose)
    #
    #     # same FOV and offset as the tracer
    #     inc = (2*math.pi / self.NUM_LIDAR )
    #
    #     for i in range(self.NUM_LIDAR):
    #         # angle of this ray in the ROBOT frame
    #         ang_r = -math.pi + i * inc
    #         # convert to WORLD frame
    #         global_angle = self.normalize_angle(robot_yaw + -math.pi/2 + ang_r)
    #
    #         r = lidar_readings[i]
    #         # skip hits outside sensor range
    #         if r <= .164 + 1e-6 or r >= 12 - 1e-6:
    #             continue
    #
    #         x = odom_x + r * math.cos(global_angle)
    #         y = odom_y + r * math.sin(global_angle)
    #         plt.plot([odom_x, x], [odom_y, y], '-', color='green', linewidth=0.5)
    #
    #     # robot forward (+x in robot frame) in red
    #     dx_robot = math.cos(robot_yaw)
    #     dy_robot = math.sin(robot_yaw)
    #     plt.arrow(odom_x, odom_y, dx_robot, dy_robot,
    #               head_width=0.1, head_length=0.15, fc='red', ec='red', label='Robot Heading')
    #
    #     # black arrow = angle_min direction
    #     angle_min_world = self.normalize_angle(robot_yaw + -math.pi/2 -math.pi)
    #     plt.arrow(odom_x, odom_y, math.cos(angle_min_world), math.sin(angle_min_world),
    #               head_width=0.1, head_length=0.15, fc='black', ec='black', label='Lidar angle_min')
    #
    #     # keep geometry from looking stretched
    #     ax = plt.gca()
    #     ax.set_aspect('equal', adjustable='box')
    def is_pose_in_map_points(self, pose_stamped, map_points, tolerance=0.01):
        """Check if a PoseStamped exists in map points within tolerance"""
        pose_x = pose_stamped.pose.position.x
        pose_y = pose_stamped.pose.position.y

        for x, y in map_points:
            distance = math.sqrt((pose_x - x)**2 + (pose_y - y)**2)
            if distance < tolerance:
                return True, (x, y)
        return False, None

# Then in your setup method:
    def create_path_from_points(self, map_points, frame_id='map'):
        """
        Create a Path message from a list of (x, y) map points.

        Parameters:
        - map_points: List of (x, y) coordinates in the map frame
        - frame_id: The frame ID for the path (default: 'map')

        Returns:
        - path: nav_msgs/Path message
        """
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(map_points):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # If not the last point, set orientation toward the next point
            if i < len(map_points) - 1:
                next_x, next_y = map_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                # Convert yaw to quaternion (simplified)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # For the last point, use the same orientation as the previous one
                pose.pose.orientation.w = 1.0

            path.poses.append(pose)
        print("Path has been created")
        return path

    def distance_between_points(self, point1, point2):
        """
        Calculate Euclidean distance between two points (x1, y1) and (x2, y2)
        """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def distance_between_poses(self, pose1, pose2):
        """
        Calculate distance between two PoseStamped messages
        """
        return self.distance_between_points(
            (pose1.pose.position.x, pose1.pose.position.y),
            (pose2.pose.position.x, pose2.pose.position.y)
        )

    def extract_messages(self, bag_path, topic):
        """Extract messages from a ROS 2 bag and store them in a dictionary grouped by timestamp."""
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        allowed_topics = {topic}
        # Dictionary to group messages by timestamp
        grouped_data = defaultdict(dict)

        while reader.has_next():
            topic, msg, timestamp = reader.read_next()

            if topic not in allowed_topics:
                continue
            # Deserialize message
            msg_type = get_message(type_map[topic])
            msg_deserialized = deserialize_message(msg, msg_type)
            if topic == "/odom":
                # Extract x, y from position
                x = msg_deserialized.pose.pose.position.x
                y = msg_deserialized.pose.pose.position.y

                odom_v = msg_deserialized.twist.twist.linear.x
                odom_w = msg_deserialized.twist.twist.angular.z
                # Extract orientation quaternion and convert to yaw
                qx = msg_deserialized.pose.pose.orientation.x
                qy = msg_deserialized.pose.pose.orientation.y
                qz = msg_deserialized.pose.pose.orientation.z
                qw = msg_deserialized.pose.pose.orientation.w
                yaw = R.from_quat([qx, qy, qz, qw]).as_euler('xyz')[2]

                grouped_data.setdefault(timestamp, {}).update({  # add getting local velocity and local angular velocity
                    "odom_x": x,
                    "odom_y": y,
                    "odom_yaw": yaw,
                    "odom_v": odom_v,
                    "odom_w": odom_w
                })
            elif topic == "/scan":
                # Convert LaserScan ranges to individual columns
                range_data = list(msg_deserialized.ranges)

                # Store each range as a separate column with an indexed key
                for i, value in enumerate(range_data):
                    grouped_data.setdefault(timestamp, {}).update({
                        f"scan_range_{i}": value
                    })

            elif topic == "/cmd_vel":
                v = msg_deserialized.linear.x
                w = msg_deserialized.angular.z
                print(f" v value {v}, w {w}")

                grouped_data.setdefault(timestamp, {}).update({
                    "cmd_v": v,
                    "cmd_w": w
                })
        return grouped_data

    def save_to_csv(self, bag_path, output_csv, topic):
        """Converts extracted messages to CSV format."""

        messages = self.extract_messages(bag_path, topic)

        if not messages:
            print("No messages found in the bag file.")
            return

        # Convert dictionary to Pandas DataFrame
        df = pd.DataFrame.from_dict(messages, orient="index")

        # Reset index to turn timestamp into a column
        df.reset_index(inplace=True)
        df.rename(columns={'index': 'timestamp'}, inplace=True)

        df.to_csv(output_csv, index=False)
        print(f"Saved {len(df)} messages to {output_csv}")

    # def ray_tracing(self, pose, segment, active_obstacles):
    #     """
    #     Args: Takes the yaw, and the obstacle data
    #     Output: Lidar data with 1080 values
    #     """
    #     local_data = [0] * self.NUM_LIDAR
    #     # active_obstacles = self.test_seg.obstacle_manager_.get_active_obstacles_claude(self.global_path, self.current_odom_index)
    #
    #     print(f"I am located at {(pose.position.x, pose.position.y)}")
    #     # print(f" len of active obstacles {len(active_obstacles)}")
    #     yaw = self.get_yaw(pose)
    #     incrementor = (2 * math.pi) / self.NUM_LIDAR
    #     lidar_offset = -math.pi/2
    #     for index in range(self.NUM_LIDAR):
    #         # Calculate direction vector for this ray
    #         theta_prev = incrementor * index
    #         theta = yaw + lidar_offset + theta_prev
    #         theta = self.normalize_angle(theta)
    #         dx = math.cos(theta)
    #         dy = math.sin(theta)
    #
    #         # Initialize to max distance to find closest
    #         min_distance = float('inf')
    #
    #         # Test against each obstacle
    #         for obs in active_obstacles:
    #             # Get obstacle data
    #             Cx = obs.center_x
    #             Cy = obs.center_y
    #             radius = obs.radius
    #
    #             # Compute ray-circle intersection
    #             mx = pose.position.x - Cx
    #             my = pose.position.y - Cy
    #
    #             a = dx * dx + dy * dy
    #             b = 2.0 * (mx * dx + my * dy)
    #             c = mx * mx + my * my - radius * radius
    #
    #             # Compute discriminant
    #             discriminant = b * b - 4.0 * a * c
    #
    #             if discriminant >= 0.0:
    #                 # Has intersection(s)
    #                 sqrt_discriminant = math.sqrt(discriminant)
    #                 t1 = (-b - sqrt_discriminant) / (2.0 * a)
    #                 t2 = (-b + sqrt_discriminant) / (2.0 * a)
    #
    #                 # Find closest valid intersection
    #                 if t1 > 0.0 and t1 < min_distance:
    #                     min_distance = t1
    #
    #                 if t2 > 0.0 and t2 < min_distance:
    #                     min_distance = t2
    #
    #         # If we found an intersection, update the distances array
    #         if min_distance != float('inf'):
    #             self.distances[index] = min_distance
    #             local_data[index] = min_distance
    #     self.get_logger().info("Calculated distances")
    #     return local_data
    def ray_tracing_capped(self, pose, segment, active_obstacles):
        """
        LiDAR with planar front cap: x_r <= FRONT_CAP_X (robot frame),
        so the far boundary appears as a straight line at 3 m ahead.
        """
        # --- sensor + cap params ---
        MIN_R = 0.164
        MAX_R = 12.0            # sensor hard max (keep it large)
        FRONT_CAP_X = 3.0       # meters straight ahead in robot frame
        FOV_MIN = -math.pi  # front FOV (adjust if needed)
        FOV_MAX =  math.pi
        EPS = 1e-9

        # noise/dropouts (your values, tweak as needed)
        noise_drop_prob = 0.10
        base_sigma = 0.003

        distances = [MAX_R] * self.NUM_LIDAR

        x0 = pose.position.x
        y0 = pose.position.y
        yaw = self.get_yaw(pose)

        # forward unit vector (robot x-axis in world)
        fx, fy = math.cos(yaw), math.sin(yaw)

        # angular sampling across front FOV
        inc = (2 * math.pi) / self.NUM_LIDAR

        for i in range(self.NUM_LIDAR):
            ang = self.normalize_angle(yaw+(-math.pi/2)+i*inc)
            # ray direction in world
            dx = math.cos(ang)
            dy = math.sin(ang)

            # ---------- circle intersections (environment) ----------
            best = MAX_R
            for obs in active_obstacles:
                Cx, Cy, r = obs.center_x, obs.center_y, obs.radius
                mx = x0 - Cx
                my = y0 - Cy
                a = dx*dx + dy*dy            # = 1 but keep for clarity
                b = 2.0 * (mx*dx + my*dy)
                c = mx*mx + my*my - r*r
                disc = b*b - 4.0*a*c
                if disc < 0.0:
                    continue
                s = math.sqrt(disc)
                t1 = (-b - s) / (2.0*a)
                t2 = (-b + s) / (2.0*a)
                if t1 > EPS and t1 < best: best = t1
                if t2 > EPS and t2 < best: best = t2

            # ---------- planar front cap: x_r <= FRONT_CAP_X ----------
            # projection of ray direction onto forward axis
            cdir = dx*fx + dy*fy   # cos(angle between ray & forward)
            # If ray points forward, intersect with plane x_r = FRONT_CAP_X at distance t_plane
            if cdir > 1e-6:
                t_plane = FRONT_CAP_X / cdir
                # Limit by the front plane
                if t_plane < best:
                    best = t_plane
            else:
                # Ray is sideways/backward; treat as no return inside front cap
                best = MAX_R

            # ---------- noise + clamping ----------
            r = max(MIN_R, min(best, MAX_R))
            # add mild, range-dependent noise when valid hit
            if MIN_R + 1e-6 < r < MAX_R - 1e-6:
                sigma = base_sigma * (1.0 + 0.4 * (r / FRONT_CAP_X))
                if random.random() >= noise_drop_prob:
                    r += random.gauss(0.0, sigma)
                r = max(MIN_R, min(r, MAX_R))

            distances[i] = r

        self.distances = distances
        self.get_logger().info("Calculated distances (front-cap)")
        return distances
    def ray_tracing(self, pose, segment, active_obstacles):
        """
        Return one LiDAR scan (len = self.NUM_LIDAR) with circle intersections.
        Distances are clamped to [MIN_R, MAX_R].
        """
        MIN_R = 0.164
        MAX_R = 12.0   # set to your sensor's max
        EPS   = 1e-9 
        
        drop_p = .1 # 10 percent of lidar no noise
        sigma  = 0.003   # meters

        # fresh scan; default = "no hit"
        distances = [MAX_R] * self.NUM_LIDAR

        x0 = pose.position.x
        y0 = pose.position.y
        yaw = self.get_yaw(pose)
        lidar_offset = -math.pi / 2.0
        inc = (2 * math.pi) / self.NUM_LIDAR

        for i in range(self.NUM_LIDAR):
            theta = self.normalize_angle(yaw + lidar_offset + i * inc)
            dx, dy = math.cos(theta), math.sin(theta)

            # search nearest positive intersection
            best = MAX_R
            for obs in active_obstacles:
                Cx, Cy, r = obs.center_x, obs.center_y, obs.radius

                # Ray-circle intersection in param t: (x0, y0) + t*(dx, dy), t>=0
                mx = x0 - Cx
                my = y0 - Cy
                a = dx*dx + dy*dy                 # == 1, but keep for clarity
                b = 2.0 * (mx*dx + my*dy)
                c = mx*mx + my*my - r*r
                disc = b*b - 4.0*a*c
                if disc < 0.0:
                    continue

                sqrt_disc = math.sqrt(disc)
                t1 = (-b - sqrt_disc) / (2.0*a)
                t2 = (-b + sqrt_disc) / (2.0*a)

                # Keep the closest positive t
                if t1 > EPS and t1 < best:
                    best = t1
                if t2 > EPS and t2 < best:
                    best = t2
            # Adding some noise to the lidar
            r = max(MIN_R, min(best, MAX_R))
            if (r > MIN_R + 1e-9) and (r < MAX_R - 1e-9):
                    if random.random() >= drop_p:  # apply noise with prob (1 - drop_p)
                        r += random.gauss(0.0, sigma)

            # single final clip
            distances[i] = max(MIN_R, min(r, MAX_R))
            # Clamp to sensor limits
        # store + return
        self.distances = distances
        self.get_logger().info("Calculated distances")
        return distances
    def ray_data_append(self, filename=None):

        if filename is None:
            filename = self.lidar_file
        output_dir = os.path.dirname(filename)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        with open(filename, 'a', newline='') as csvfile:  # Changed 'w' to 'a' for append mode
            writer = csv.writer(csvfile)
            if self.lidar_header_flag:
                headers = [f'lidar_{i}' for i in range(len(self.distances))]
                writer.writerow(headers)
                self.lidar_header_flag = False

            # Write data row - just the ray distances
            writer.writerow(self.distances)

    def get_yaw(self, pose: Pose) -> float:
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        # _, _, yaw = euler_from_quaternion(quat)
        _, _, yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return yaw

    def normalize_angle(self, value):
        return (value + math.pi) % (2 * math.pi) - math.pi

    def oversample_cmdVel3(self, odom_csv, cmd_csv, output_csv):
        import pandas as pd

        # Read the CSV files for odom and cmd
        odom_df = pd.read_csv(odom_csv)
        cmd_df = pd.read_csv(cmd_csv)

        # Convert timestamps to numeric for accurate merging
        odom_df['timestamp'] = pd.to_numeric(odom_df['timestamp'])
        cmd_df['timestamp'] = pd.to_numeric(cmd_df['timestamp'])

        print("have grabbed values")
        # Merge the command velocities onto odom timestamps using merge_asof.
        # We only keep the timestamp from odom, and the cmd_v and cmd_w from the cmd DataFrame.
        merged_df = pd.merge_asof(
            odom_df[['timestamp']],  # Use only the odom timestamp
            cmd_df[['timestamp', 'cmd_v', 'cmd_w']],
            on='timestamp',
            direction='nearest'
        )

        print("saving to csv")
        # Save only the timestamp, cmd_v, and cmd_w columns to the output CSV
        merged_df.to_csv(output_csv, index=False)
        return merged_df

    def yaml_reader(self):
        """Read the configs from config.yaml"""
        filepath = os.path.join(os.path.expanduser(
            '~'), 'ros_ws', 'config.yaml')  # Fixed typo: trail -> trial
        with open(filepath, "r") as file:
            config = yaml.safe_load(file)

            # Example access
            self.RADIUS = config["RADIUS"]
            self.OFFSET = config["OFFSET"]

            print(f"Loaded: RADIUS={self.RADIUS}, OFFSET={self.OFFSET}")

    def write_meta_data(self):

        filepath = os.path.join(self.input_bag, 'config_meta_data.yaml')
        meta = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S%z"),
            "OFFSET": self.OFFSET,
            "RADIUS": self.RADIUS,
        }
        with open(filepath, "w") as f:
            yaml.safe_dump(meta, f)
        self.get_logger().info(
            f"Wrote OFFSET={self.OFFSET}, RADIUS={self.RADIUS} at {meta['timestamp']} to {filepath}"
        )
    def check_shutdown(self):
        """Timer callback to check if shutdown was requested"""
        if self.shutdown_requested:
            self.get_logger().info("Shutting down node...")
            rclpy.shutdown()
            sys.exit(0)
def main(args=None):
    import argparse
    
    # Parse command line arguments before ROS2 init
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_bag', type=str, 
                       help='Path to input bag directory')
    
    # Parse known args to allow ROS2 args to pass through
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=unknown)  # Pass remaining args to ROS2
    test_node = MapTraining(input_bag_path=parsed_args.input_bag)
    
    try:
        # Keep spinning until shutdown is requested
        while rclpy.ok() and not test_node.shutdown_requested:
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        print("Processing complete - shutting down")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
