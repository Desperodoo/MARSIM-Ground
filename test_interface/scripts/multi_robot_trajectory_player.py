#!/usr/bin/env python3

import argparse
import math
import sys
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def make_point(x, y, z):
    pt = Point()
    pt.x = x
    pt.y = y
    pt.z = z
    return pt


def rotate_vec_by_quat(vec, quat):
    qx, qy, qz, qw = quat
    tx = 2.0 * (qy * vec[2] - qz * vec[1])
    ty = 2.0 * (qz * vec[0] - qx * vec[2])
    tz = 2.0 * (qx * vec[1] - qy * vec[0])

    rx = vec[0] + qw * tx + (qy * tz - qz * ty)
    ry = vec[1] + qw * ty + (qz * tx - qx * tz)
    rz = vec[2] + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


def load_pcd_xyz(path):
    with open(path, "rb") as handle:
        meta = {}
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("unexpected end of PCD header")
            text = line.decode("ascii", "ignore").strip()
            parts = text.split()
            if parts and not parts[0].startswith("#"):
                meta[parts[0]] = parts[1:]
            if text.startswith("DATA"):
                break

        fields = meta["FIELDS"]
        sizes = [int(v) for v in meta["SIZE"]]
        types = meta["TYPE"]
        counts = [int(v) for v in meta["COUNT"]]
        point_count = int(meta["POINTS"][0])
        data_type = meta["DATA"][0].lower()
        if data_type != "binary":
            raise RuntimeError("only binary PCD files are supported")

        dtype_fields = []
        for name, size, typ, count in zip(fields, sizes, types, counts):
            base = {
                ("F", 4): "f4",
                ("F", 8): "f8",
                ("U", 1): "u1",
                ("U", 2): "u2",
                ("U", 4): "u4",
                ("I", 1): "i1",
                ("I", 2): "i2",
                ("I", 4): "i4",
            }[(typ, size)]
            if count == 1:
                dtype_fields.append((name, base))
            else:
                dtype_fields.append((name, base, (count,)))

        points = np.fromfile(handle, dtype=np.dtype(dtype_fields), count=point_count)
        return (
            np.asarray(points["x"], dtype=np.float32),
            np.asarray(points["y"], dtype=np.float32),
            np.asarray(points["z"], dtype=np.float32),
        )


def nearest_free_seed(free_mask, x_index, y_index, search_radius):
    x_size, y_size = free_mask.shape
    x_index = int(clamp(x_index, 0, x_size - 1))
    y_index = int(clamp(y_index, 0, y_size - 1))
    for radius in range(search_radius + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                ix = x_index + dx
                iy = y_index + dy
                if ix < 0 or ix >= x_size or iy < 0 or iy >= y_size:
                    continue
                if free_mask[ix, iy]:
                    return (ix, iy)
    return None


def inflate_mask(mask, inflation_cells):
    if inflation_cells <= 0:
        return mask.copy()

    inflated = mask.copy()
    x_size, y_size = mask.shape
    for dx in range(-inflation_cells, inflation_cells + 1):
        for dy in range(-inflation_cells, inflation_cells + 1):
            if dx * dx + dy * dy > inflation_cells * inflation_cells:
                continue
            src_x0 = max(0, -dx)
            src_x1 = min(x_size, x_size - dx)
            src_y0 = max(0, -dy)
            src_y1 = min(y_size, y_size - dy)
            dst_x0 = max(0, dx)
            dst_x1 = dst_x0 + (src_x1 - src_x0)
            dst_y0 = max(0, dy)
            dst_y1 = dst_y0 + (src_y1 - src_y0)
            inflated[dst_x0:dst_x1, dst_y0:dst_y1] |= mask[src_x0:src_x1, src_y0:src_y1]
    return inflated


def compute_reachable_mask(free_mask, seeds):
    reachable = np.zeros_like(free_mask, dtype=np.uint8)
    queue = deque()
    for seed in seeds:
        if seed is None:
            continue
        if not reachable[seed]:
            reachable[seed] = 1
            queue.append(seed)

    x_size, y_size = free_mask.shape
    while queue:
        ix, iy = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            jx = ix + dx
            jy = iy + dy
            if jx < 0 or jx >= x_size or jy < 0 or jy >= y_size:
                continue
            if not free_mask[jx, jy] or reachable[jx, jy]:
                continue
            reachable[jx, jy] = 1
            queue.append((jx, jy))

    return reachable.astype(bool)


def orient_path_from_nearest_endpoint(path_points, start_xy):
    if len(path_points) < 2:
        return list(path_points)

    start_dist_sq = (path_points[0][0] - start_xy[0]) ** 2 + (path_points[0][1] - start_xy[1]) ** 2
    end_dist_sq = (path_points[-1][0] - start_xy[0]) ** 2 + (path_points[-1][1] - start_xy[1]) ** 2
    if start_dist_sq <= end_dist_sq:
        return list(path_points)
    return list(reversed(path_points))


def downsample_path(path_points, min_spacing):
    if not path_points:
        return []
    result = [path_points[0]]
    min_spacing_sq = min_spacing * min_spacing
    for point in path_points[1:]:
        dx = point[0] - result[-1][0]
        dy = point[1] - result[-1][1]
        if dx * dx + dy * dy >= min_spacing_sq:
            result.append(point)
    if len(result) == 1 and len(path_points) > 1:
        result.append(path_points[-1])
    return result


def chaikin_open(points, iterations):
    result = list(points)
    for _ in range(iterations):
        if len(result) < 3:
            break
        refined = [result[0]]
        for idx in range(len(result) - 1):
            p0 = result[idx]
            p1 = result[idx + 1]
            q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
            r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
            refined.extend([q, r])
        refined.append(result[-1])
        result = refined
    return result


def generate_band_path(reachable_mask, x_coords, y_coords, row_begin, row_end, lane_step_cells):
    band_rows = list(range(row_begin + 1, row_end - 1, lane_step_cells))
    if not band_rows:
        return []

    band_slice = reachable_mask[:, row_begin:row_end]
    occupied_columns = np.where(band_slice.any(axis=1))[0]
    if occupied_columns.size == 0:
        return []

    x_min = int(occupied_columns.min())
    x_max = int(occupied_columns.max())
    path_points = []
    forward = True
    current_row = None

    def nearest_reachable_row(ix, nominal_row, reference_row):
        max_offset = max(2, (row_end - row_begin) // 2)
        best_row = None
        best_cost = None
        for offset in range(max_offset + 1):
            candidates = [nominal_row] if offset == 0 else [nominal_row - offset, nominal_row + offset]
            for candidate in candidates:
                if candidate < row_begin or candidate >= row_end:
                    continue
                if not reachable_mask[ix, candidate]:
                    continue
                continuity = 0 if reference_row is None else abs(candidate - reference_row)
                cost = offset * 4 + continuity
                if best_cost is None or cost < best_cost:
                    best_cost = cost
                    best_row = candidate
        return best_row

    for lane_idx, nominal_row in enumerate(band_rows):
        if forward:
            edge_ix = x_min
            sweep = range(x_min, x_max + 1, 2)
        else:
            edge_ix = x_max
            sweep = range(x_max, x_min - 1, -2)

        start_row = nearest_reachable_row(edge_ix, nominal_row, current_row)
        if start_row is None:
            forward = not forward
            continue

        if current_row is not None:
            connector_rows = np.linspace(current_row, start_row, num=max(2, abs(start_row - current_row) + 1))
            for connector_row in connector_rows[1:]:
                row_int = nearest_reachable_row(edge_ix, int(round(connector_row)), current_row)
                if row_int is None:
                    continue
                path_points.append((float(x_coords[edge_ix]), float(y_coords[row_int])))
                current_row = row_int

        lane_points = []
        previous_row = start_row
        for ix in sweep:
            best_row = nearest_reachable_row(ix, nominal_row, previous_row)
            if best_row is None:
                continue
            lane_points.append((float(x_coords[ix]), float(y_coords[best_row])))
            previous_row = best_row

        if lane_points:
            path_points.extend(lane_points)
            current_row = previous_row

        forward = not forward

    return path_points


def fallback_paths():
    return [
        [
            (1.8, -0.8),
            (5.2, -0.6),
            (9.8, -0.7),
            (13.0, -1.5),
            (15.8, -2.6),
            (13.8, -4.2),
            (9.5, -4.4),
            (5.3, -3.7),
            (2.0, -2.6),
            (0.8, -1.5),
        ],
        [
            (0.8, -3.9),
            (4.4, -5.3),
            (8.9, -5.7),
            (13.6, -5.2),
            (17.0, -4.0),
            (15.7, -2.8),
            (11.5, -2.4),
            (7.0, -2.8),
            (3.3, -3.1),
            (1.3, -3.4),
        ],
        [
            (0.5, 2.2),
            (3.8, 3.2),
            (7.8, 3.8),
            (11.8, 3.6),
            (15.2, 2.7),
            (16.8, 1.2),
            (13.8, 0.2),
            (9.6, 0.4),
            (5.1, 1.0),
            (1.7, 1.6),
        ],
    ]


def default_path_seed_starts():
    return [
        (0.0, 0.0),
        (-2.0, -1.5),
        (-2.0, 1.5),
    ]


def compute_showcase_init_states(path_sets, fixed_z):
    init_states = []
    for path in path_sets:
        if len(path) < 2:
            raise RuntimeError("path is too short to compute showcase init state")
        p0 = path[0]
        p1 = path[1]
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        init_states.append((p0[0], p0[1], fixed_z, yaw))
    return init_states


def generate_coverage_paths(map_name, starts, grid_resolution, obstacle_z_min, obstacle_z_max,
                            inflation_radius, lane_spacing, quantile_low, quantile_high):
    x_values, y_values, z_values = load_pcd_xyz(map_name)

    x_low = float(np.quantile(x_values, quantile_low))
    x_high = float(np.quantile(x_values, quantile_high))
    y_low = float(np.quantile(y_values, quantile_low))
    y_high = float(np.quantile(y_values, quantile_high))

    x_low = min(x_low, min(start[0] for start in starts)) - 1.0
    x_high = max(x_high, max(start[0] for start in starts)) + 1.0
    y_low = min(y_low, min(start[1] for start in starts)) - 0.8
    y_high = max(y_high, max(start[1] for start in starts)) + 0.8

    x_size = int(math.ceil((x_high - x_low) / grid_resolution)) + 1
    y_size = int(math.ceil((y_high - y_low) / grid_resolution)) + 1
    x_coords = x_low + np.arange(x_size) * grid_resolution
    y_coords = y_low + np.arange(y_size) * grid_resolution

    low_obs = (z_values >= obstacle_z_min) & (z_values <= obstacle_z_max)
    obs_x = x_values[low_obs]
    obs_y = y_values[low_obs]

    occupancy = np.zeros((x_size, y_size), dtype=np.uint8)
    if obs_x.size > 0:
        gx = np.clip(((obs_x - x_low) / grid_resolution).astype(np.int32), 0, x_size - 1)
        gy = np.clip(((obs_y - y_low) / grid_resolution).astype(np.int32), 0, y_size - 1)
        occupancy[gx, gy] = 1

    inflated = inflate_mask(occupancy.astype(bool), max(0, int(round(inflation_radius / grid_resolution))))
    free_mask = ~inflated

    seeds = []
    for start_x, start_y in starts:
        seed = nearest_free_seed(
            free_mask,
            int(round((start_x - x_low) / grid_resolution)),
            int(round((start_y - y_low) / grid_resolution)),
            8,
        )
        seeds.append(seed)

    reachable = compute_reachable_mask(free_mask, seeds)
    reachable_indices = np.argwhere(reachable)
    if reachable_indices.size == 0:
        raise RuntimeError("failed to find reachable free space from start positions")

    row_min = int(reachable_indices[:, 1].min())
    row_max = int(reachable_indices[:, 1].max()) + 1
    band_edges = np.linspace(row_min, row_max, 4).astype(int)
    lane_step_cells = max(2, int(round(lane_spacing / grid_resolution)))

    paths = []
    debug = []
    for robot_index, start_xy in enumerate(starts):
        row_begin = int(band_edges[robot_index])
        row_end = int(band_edges[robot_index + 1])
        raw_path = generate_band_path(reachable, x_coords, y_coords, row_begin, row_end, lane_step_cells)
        if len(raw_path) < 4:
            raise RuntimeError(f"coverage path for robot {robot_index} is too short")
        ordered = orient_path_from_nearest_endpoint(raw_path, start_xy)
        spaced = downsample_path(ordered, lane_spacing * 0.45)
        smoothed = chaikin_open(spaced, 2)
        final_path = downsample_path(smoothed, lane_spacing * 0.28)
        if len(final_path) < 4:
            raise RuntimeError(f"smoothed coverage path for robot {robot_index} is too short")
        paths.append(final_path)
        debug.append((row_begin, row_end, len(raw_path), len(final_path)))

    rospy.loginfo("Generated coverage paths from %s", map_name)
    for idx, band_info in enumerate(debug):
        rospy.loginfo(
            "Robot %d band rows [%d, %d), raw points=%d, final points=%d",
            idx,
            band_info[0],
            band_info[1],
            band_info[2],
            band_info[3],
        )
    return paths


class RobotProfile:
    def __init__(self, name, init_xyz, init_yaw, path_points, speed_mps, color):
        self.name = name
        self.init_x, self.init_y, self.init_z = init_xyz
        self.init_yaw = init_yaw
        self.path_points = list(path_points)
        self.speed_mps = max(0.2, speed_mps)
        self.color = color
        self.last_yaw = init_yaw
        self.history_points = deque(maxlen=5000)
        self.latest_odom = None

        self.segment_lengths = []
        self.cumulative_lengths = [0.0]
        total = 0.0
        for idx in range(len(self.path_points) - 1):
            dx = self.path_points[idx + 1][0] - self.path_points[idx][0]
            dy = self.path_points[idx + 1][1] - self.path_points[idx][1]
            seg_length = math.hypot(dx, dy)
            if seg_length < 1e-6:
                continue
            self.segment_lengths.append(seg_length)
            total += seg_length
            self.cumulative_lengths.append(total)
        self.total_length = total
        if self.total_length < 1.0:
            raise RuntimeError(f"path for {self.name} is too short")

        ns = f"/{name}"
        self.cmd_pub = rospy.Publisher(f"{ns}/planning/pos_cmd", PositionCommand, queue_size=10)
        self.path_pub = rospy.Publisher(f"{ns}/planning/travel_traj", Marker, queue_size=1, latch=True)
        self.cmd_vis_pub = rospy.Publisher(f"{ns}/planning/position_cmd_vis", Marker, queue_size=10)
        self.history_pub = rospy.Publisher(f"{ns}/planning/history_traj", Marker, queue_size=10)
        self.axes_pub = rospy.Publisher(f"{ns}/planning/body_axes", Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber(f"{ns}/lidar_slam/odom", Odometry, self.odom_callback, queue_size=20)

    def odom_callback(self, msg):
        self.latest_odom = msg
        position = msg.pose.pose.position
        self.history_points.append((position.x, position.y, position.z))

    def sample_path(self, distance_along):
        if distance_along <= 0.0:
            p0 = self.path_points[0]
            p1 = self.path_points[1]
            heading = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            return p0[0], p0[1], heading
        if distance_along >= self.total_length:
            p0 = self.path_points[-2]
            p1 = self.path_points[-1]
            heading = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            return p1[0], p1[1], heading

        seg_index = 0
        while seg_index + 1 < len(self.cumulative_lengths) and self.cumulative_lengths[seg_index + 1] < distance_along:
            seg_index += 1

        start_length = self.cumulative_lengths[seg_index]
        end_length = self.cumulative_lengths[seg_index + 1]
        p0 = self.path_points[seg_index]
        p1 = self.path_points[seg_index + 1]
        alpha = (distance_along - start_length) / max(1e-6, end_length - start_length)
        x = p0[0] + alpha * (p1[0] - p0[0])
        y = p0[1] + alpha * (p1[1] - p0[1])
        heading = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return x, y, heading

    def route_reference(self, t, fixed_z):
        route_duration = self.total_length / self.speed_mps
        if route_duration <= 1e-6:
            route_duration = 1.0

        cycle = 2.0 * route_duration
        t_cycle = t % cycle
        moving_forward = t_cycle <= route_duration
        if moving_forward:
            distance = t_cycle * self.speed_mps
            velocity_sign = 1.0
        else:
            distance = (cycle - t_cycle) * self.speed_mps
            velocity_sign = -1.0

        x, y, heading = self.sample_path(distance)
        vx = velocity_sign * self.speed_mps * math.cos(heading)
        vy = velocity_sign * self.speed_mps * math.sin(heading)
        if moving_forward:
            yaw = heading
        else:
            yaw = wrap_angle(heading + math.pi)

        self.last_yaw = yaw
        return {
            "x": x,
            "y": y,
            "z": fixed_z,
            "vx": vx,
            "vy": vy,
            "vz": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,
            "yaw": yaw,
        }

    def reference(self, t, fixed_z, warmup_sec):
        if t < warmup_sec:
            if warmup_sec <= 1e-6:
                return self.route_reference(t, fixed_z)

            tau = clamp(t / warmup_sec, 0.0, 1.0)
            s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            ds_dt = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / warmup_sec
            d2s_dt2 = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (warmup_sec**2)

            x0 = self.init_x
            y0 = self.init_y
            x1, y1 = self.path_points[0]
            dx = x1 - x0
            dy = y1 - y0

            x = x0 + s * dx
            y = y0 + s * dy
            vx = ds_dt * dx
            vy = ds_dt * dy
            ax = d2s_dt2 * dx
            ay = d2s_dt2 * dy

            position_error = math.hypot(dx, dy)
            speed = math.hypot(vx, vy)
            if position_error < 0.02:
                self.last_yaw = self.init_yaw
            elif speed > 0.03:
                self.last_yaw = math.atan2(vy, vx)

            return {
                "x": x,
                "y": y,
                "z": fixed_z,
                "vx": vx,
                "vy": vy,
                "vz": 0.0,
                "ax": ax,
                "ay": ay,
                "az": 0.0,
                "yaw": wrap_angle(self.last_yaw),
            }

        return self.route_reference(t - warmup_sec, fixed_z)

    def publish_path(self, fixed_z):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "scripted_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color = ColorRGBA(*self.color)
        for x, y in self.path_points:
            marker.points.append(make_point(x, y, fixed_z + 0.02))
        self.path_pub.publish(marker)

    def publish_cmd_vis(self, ref):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.18
        marker.scale.z = 0.24
        marker.color = ColorRGBA(*self.color)

        start = make_point(ref["x"], ref["y"], ref["z"] + 0.04)
        end = make_point(
            ref["x"] + 0.6 * math.cos(ref["yaw"]),
            ref["y"] + 0.6 * math.sin(ref["yaw"]),
            ref["z"] + 0.04,
        )
        marker.points = [start, end]
        self.cmd_vis_pub.publish(marker)

    def publish_history_traj(self):
        history_points = list(self.history_points)
        if len(history_points) < 2:
            return

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "history_traj"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.color = ColorRGBA(self.color[0], self.color[1], self.color[2], 0.85)
        for point in history_points:
            marker.points.append(make_point(point[0], point[1], point[2] + 0.03))
        self.history_pub.publish(marker)

    def publish_body_axes(self):
        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        origin = (pose.position.x, pose.position.y, pose.position.z + 0.05)
        quat = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        axis_scale = 1.1
        axes = [
            ((1.0, 0.0, 0.0), ColorRGBA(0.95, 0.28, 0.24, 0.95)),
            ((0.0, 1.0, 0.0), ColorRGBA(0.22, 0.78, 0.34, 0.95)),
            ((0.0, 0.0, 1.0), ColorRGBA(0.20, 0.48, 0.95, 0.95)),
        ]

        for axis_id, (axis_vec, color) in enumerate(axes):
            rotated = rotate_vec_by_quat(axis_vec, quat)
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "body_axes"
            marker.id = axis_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.08
            marker.scale.y = 0.16
            marker.scale.z = 0.20
            marker.color = color
            marker.points = [
                make_point(*origin),
                make_point(
                    origin[0] + rotated[0] * axis_scale,
                    origin[1] + rotated[1] * axis_scale,
                    origin[2] + rotated[2] * axis_scale,
                ),
            ]
            self.axes_pub.publish(marker)

    def publish_command(self, ref, traj_id):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.position.x = ref["x"]
        cmd.position.y = ref["y"]
        cmd.position.z = ref["z"]
        cmd.velocity.x = ref["vx"]
        cmd.velocity.y = ref["vy"]
        cmd.velocity.z = ref["vz"]
        cmd.acceleration.x = ref["ax"]
        cmd.acceleration.y = ref["ay"]
        cmd.acceleration.z = ref["az"]
        cmd.jerk = Vector3(0.0, 0.0, 0.0)
        cmd.yaw = ref["yaw"]
        cmd.yaw_dot = 0.0
        cmd.trajectory_id = traj_id
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.kx = [5.7, 5.7, 6.2]
        cmd.kv = [3.4, 3.4, 4.0]
        self.cmd_pub.publish(cmd)


class MultiRobotTrajectoryPlayer:
    def __init__(self):
        rospy.init_node("multi_robot_trajectory_player")

        self.fixed_z = rospy.get_param("~fixed_z", 0.30)
        self.warmup_sec = rospy.get_param("~warmup_sec", 6.0)
        self.publish_rate = rospy.get_param("~publish_rate", 40.0)
        self.map_name = rospy.get_param("~map_name", "")
        self.path_speed = rospy.get_param("~path_speed", 0.55)
        self.grid_resolution = rospy.get_param("~grid_resolution", 0.45)
        self.obstacle_z_min = rospy.get_param("~obstacle_z_min", 0.15)
        self.obstacle_z_max = rospy.get_param("~obstacle_z_max", 1.8)
        self.obstacle_inflation = rospy.get_param("~obstacle_inflation", 0.45)
        self.lane_spacing = rospy.get_param("~lane_spacing", 1.1)
        self.bounds_quantile_low = rospy.get_param("~bounds_quantile_low", 0.03)
        self.bounds_quantile_high = rospy.get_param("~bounds_quantile_high", 0.97)

        path_seed_starts = default_path_seed_starts()

        try:
            if not self.map_name:
                raise RuntimeError("map_name parameter is empty")
            path_sets = generate_coverage_paths(
                self.map_name,
                path_seed_starts,
                self.grid_resolution,
                self.obstacle_z_min,
                self.obstacle_z_max,
                self.obstacle_inflation,
                self.lane_spacing,
                self.bounds_quantile_low,
                self.bounds_quantile_high,
            )
        except Exception as exc:
            rospy.logwarn("Failed to generate map-driven coverage paths: %s", exc)
            path_sets = fallback_paths()

        default_init_states = compute_showcase_init_states(path_sets, self.fixed_z)
        showcase_init_states = [
            (
                rospy.get_param("~quad0_init_x", default_init_states[0][0]),
                rospy.get_param("~quad0_init_y", default_init_states[0][1]),
                self.fixed_z,
                rospy.get_param("~quad0_init_yaw", default_init_states[0][3]),
            ),
            (
                rospy.get_param("~quad1_init_x", default_init_states[1][0]),
                rospy.get_param("~quad1_init_y", default_init_states[1][1]),
                self.fixed_z,
                rospy.get_param("~quad1_init_yaw", default_init_states[1][3]),
            ),
            (
                rospy.get_param("~quad2_init_x", default_init_states[2][0]),
                rospy.get_param("~quad2_init_y", default_init_states[2][1]),
                self.fixed_z,
                rospy.get_param("~quad2_init_yaw", default_init_states[2][3]),
            ),
        ]

        self.robots = [
            RobotProfile("quad_0", showcase_init_states[0][:3], showcase_init_states[0][3], path_sets[0], self.path_speed, (0.82, 0.60, 0.53, 0.95)),
            RobotProfile("quad_1", showcase_init_states[1][:3], showcase_init_states[1][3], path_sets[1], self.path_speed, (0.62, 0.66, 0.63, 0.95)),
            RobotProfile("quad_2", showcase_init_states[2][:3], showcase_init_states[2][3], path_sets[2], self.path_speed, (0.65, 0.63, 0.56, 0.95)),
        ]

        self.start_time = rospy.Time.now()
        self.traj_id = 1

        rospy.sleep(0.5)
        for robot in self.robots:
            robot.publish_path(self.fixed_z)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            for robot in self.robots:
                ref = robot.reference(elapsed, self.fixed_z, self.warmup_sec)
                robot.publish_command(ref, self.traj_id)
                robot.publish_cmd_vis(ref)
                robot.publish_history_traj()
                robot.publish_body_axes()
            self.traj_id += 1
            rate.sleep()


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--print-init-states", action="store_true")
    parser.add_argument("--map", dest="map_name", default="")
    parser.add_argument("--fixed-z", dest="fixed_z", type=float, default=0.30)
    parser.add_argument("--grid-resolution", dest="grid_resolution", type=float, default=0.45)
    parser.add_argument("--obstacle-z-min", dest="obstacle_z_min", type=float, default=0.15)
    parser.add_argument("--obstacle-z-max", dest="obstacle_z_max", type=float, default=1.8)
    parser.add_argument("--obstacle-inflation", dest="obstacle_inflation", type=float, default=0.45)
    parser.add_argument("--lane-spacing", dest="lane_spacing", type=float, default=1.1)
    parser.add_argument("--bounds-quantile-low", dest="bounds_quantile_low", type=float, default=0.03)
    parser.add_argument("--bounds-quantile-high", dest="bounds_quantile_high", type=float, default=0.97)
    return parser


def print_init_states_cli(args):
    if not args.map_name:
        raise RuntimeError("--map is required with --print-init-states")

    path_sets = generate_coverage_paths(
        args.map_name,
        default_path_seed_starts(),
        args.grid_resolution,
        args.obstacle_z_min,
        args.obstacle_z_max,
        args.obstacle_inflation,
        args.lane_spacing,
        args.bounds_quantile_low,
        args.bounds_quantile_high,
    )
    init_states = compute_showcase_init_states(path_sets, args.fixed_z)

    for idx, (x, y, _, yaw) in enumerate(init_states):
        print(f"quad{idx}_init_x:={x}")
        print(f"quad{idx}_init_y:={y}")
        print(f"quad{idx}_init_yaw:={yaw}")


if __name__ == "__main__":
    cli_args, _ = build_arg_parser().parse_known_args()
    if cli_args.print_init_states:
        print_init_states_cli(cli_args)
        sys.exit(0)
    MultiRobotTrajectoryPlayer().run()
