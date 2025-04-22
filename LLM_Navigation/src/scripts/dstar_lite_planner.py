#!/usr/bin/env python3
"""
LLM‑D* Hybrid global planner
ROS I/O is identical to the original `dstar_lite_planner.py`
  • Sub : /map (OccupancyGrid), /odom (Odometry), /llm_waypoints_pose_array (PoseArray), /obstacles (String‑JSON)
  • Pub : /dstar_path (Path)
  • Act : compute_path_to_pose (nav2_msgs/action/ComputePathToPose)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from builtin_interfaces.msg import Duration as BuiltinDuration
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
import math, heapq, json, copy, os, re
#
# ────────────────────────────────  D* Lite core  ────────────────────────────────
#
class DStarLite:
    """Incremental four‑connected D* Lite over a static‑sized grid.
       Call  ▸ update_cell_cost(i,j,new_cost)  whenever a cell cost changes.
       Then  ▸ repair()  to incrementally restore consistency.
    """

    def __init__(self, grid, w, h, start, goal, heuristic):
        self.grid = grid              # 2‑D list[rows][cols]
        self.w, self.h = w, h
        self.s_start, self.s_goal = start, goal
        self.h_fn = heuristic         # must be consistent
        self.k_m = 0                  # key modifier
        self.g  = { }                 # cost‑to‑come
        self.rhs = { }                # one‑step look‑ahead
        self.OPEN = []                # heap of (key, cell)
        self.open_set = set()

        for j in range(h):
            for i in range(w):
                self.g[(i,j)] = math.inf
                self.rhs[(i,j)] = math.inf
        self.rhs[self.s_goal] = 0.0
        self._push(self.s_goal)

    # ── helpers ────────────────────────────────────────────────────────────────
    def _heuristic(self, u):  # only depends on start, so recomputed when start moves
        return self.h_fn(self.s_start, u)

    def _key(self, u):
        v = min(self.g[u], self.rhs[u])
        return (v + self._heuristic(u) + self.k_m, v)

    def _push(self, u):
        heapq.heappush(self.OPEN, (self._key(u), u))
        self.open_set.add(u)

    def _remove_if_present(self, u):
        if u in self.open_set:
            # lazy deletion – don’t search+remove from heap, just mark gone
            self.open_set.remove(u)

    # ── topology & costs ───────────────────────────────────────────────────────
    def in_bounds(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def cost(self, u, v):
        """returns traversal cost from u→v (4‑connected)."""
        i, j = v
        return self.grid[j][i]

    def neighbors(self, u):
        i, j = u
        for di, dj in ((1,0),(-1,0),(0,1),(0,-1)):
            ni, nj = i+di, j+dj
            if self.in_bounds(ni, nj):
                yield (ni, nj)

    # ── core D* Lite routines ─────────────────────────────────────────────────
    def update_vertex(self, u):
        if u != self.s_goal:
            self.rhs[u] = min(
                self.g[v] + self.cost(u, v) for v in self.neighbors(u)
            )
        self._remove_if_present(u)
        if self.g[u] != self.rhs[u]:
            self._push(u)

    def _compute_shortest_path(self):
        """runs until start is locally consistent"""
        while self.OPEN:
            k_old, u = heapq.heappop(self.OPEN)
            if u not in self.open_set:
                continue  # deleted entry
            k_new = self._key(u)
            if k_old < k_new:
                self._push(u)
                continue
            self.open_set.remove(u)

            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = math.inf
                self.update_vertex(u)
                for s in self.neighbors(u):
                    self.update_vertex(s)

            if self._key(self.s_start) >= self.OPEN[0][0] and \
               self.g[self.s_start] == self.rhs[self.s_start]:
                break  # already consistent

    def move_start(self, new_start):
        """robot advanced along path → adjust heuristic anchor."""
        if new_start == self.s_start:
            return
        self.k_m += self.h_fn(self.s_start, new_start)
        self.s_start = new_start
        self._compute_shortest_path()

    def update_cell_cost(self, cell, new_cost):
        """Change traversability and trigger incremental repair."""
        i, j = cell
        self.grid[j][i] = new_cost
        self.update_vertex(cell)   # RHS of neighbours may change
        for n in self.neighbors(cell):
            self.update_vertex(n)

    def repair(self):
        """Public hook after one or more `update_cell_cost` calls."""
        self._compute_shortest_path()

    # ── path extraction ───────────────────────────────────────────────────────
    def extract_path(self, max_len=10000):
        """Greedy descent along g‑values."""
        if math.isinf(self.g[self.s_start]):
            return []  # no path
        path, u = [self.s_start], self.s_start
        while u != self.s_goal and len(path) < max_len:
            u = min(self.neighbors(u), key=lambda v: self.g[v] + self.cost(u, v))
            if math.isinf(self.g[u]):
                return []
            path.append(u)
        return path
#
# ─────────────────────────  Planner node (ROS wrappers)  ──────────────────────
#
class LlmDstarPlanner(Node):
    def __init__(self):
        super().__init__('dstar_lite_planner')    # name unchanged
        # ⟦ configuration ⟧
        self.llm_model = os.getenv('OLLAMA_MODEL', 'llama3')  # adapt to your runtime
        self.safe_margin = 1                      # cells around obstacles
        # ⟦ state ⟧
        self.map_msg        = None                # last OccupancyGrid
        self.pose_grid      = None                # (i,j) current robot cell
        self.grid, self.w, self.h = None, 0, 0
        self.waypoints      = []                  # list[(i,j)]
        self.dstar          = None                # DStarLite instance
        self.current_path   = []                  # list[(i,j)]

        # ── TF listener ────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── ROS subs / pubs / action ───────────────────────────────────────
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qos)
        self.create_subscription(Odometry,      '/odom', self.cb_odom, 10)
        self.create_subscription(PoseArray,     '/llm_waypoints_pose_array',
                                 self.cb_waypoints, 10)
        self.create_subscription(String,        '/obstacles', self.cb_obstacles, 10)

        self.path_pub = self.create_publisher(Path, '/dstar_path', 10)
        self.action_srv = ActionServer(self, ComputePathToPose,
                                       'compute_path_to_pose', self.execute_cb)

        self.get_logger().info('LLM‑D* planner ready.')

    # ── Map conversion utilities ───────────────────────────────────────────
    def world_to_grid(self, x, y):
        res = self.map_msg.info.resolution
        ox  = self.map_msg.info.origin.position.x
        oy  = self.map_msg.info.origin.position.y
        return (int((x - ox) / res), int((y - oy) / res))

    def grid_to_world(self, i, j):
        res = self.map_msg.info.resolution
        ox  = self.map_msg.info.origin.position.x
        oy  = self.map_msg.info.origin.position.y
        return (i * res + ox, j * res + oy)

    # ── Callbacks ──────────────────────────────────────────────────────────
    def cb_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.grid, self.w, self.h = self.build_grid(msg)
        # invalidate planner – will rebuild on next trigger
        self.dstar = None

    def cb_odom(self, msg: Odometry):
        if self.map_msg is None:
            return
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose
        try:
            tf_ps = self.tf_buffer.transform(ps, 'map',
                                             timeout=Duration(seconds=0.3))
            x, y = tf_ps.pose.position.x, tf_ps.pose.position.y
        except Exception:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.pose_grid = self.world_to_grid(x, y)

        # As the robot advances, shift D* anchor ⇒ cheap incremental repair
        if self.dstar is not None and self.pose_grid is not None:
            self.dstar.move_start(self.pose_grid)

    def cb_waypoints(self, msg: PoseArray):
        if self.map_msg is None:
            return
        self.waypoints.clear()
        for pose in msg.poses:
            self.waypoints.append(self.world_to_grid(pose.position.x,
                                                     pose.position.y))
        self.waypoints = self.sanitize_waypoints(self.waypoints)
        self.get_logger().info(f"Received {len(self.waypoints)} sanitized way‑points.")
        self.plan_and_publish()

    def cb_obstacles(self, msg: String):
        """Dynamic obstacle list = array of grid cells  [{'x':i,'y':j}, … ]"""
        if self.dstar is None:
            return
        try:
            data = json.loads(msg.data)
            changed = []
            for ob in data.get('obstacles', []):
                i, j = int(ob['x']), int(ob['y'])
                if not (0 <= i < self.w and 0 <= j < self.h):
                    continue
                if self.grid[j][i] != math.inf:
                    self.grid[j][i] = math.inf
                    self.dstar.update_cell_cost((i, j), math.inf)
                    changed.append((i, j))
            if changed:
                self.dstar.repair()
                if self.is_obstructed():          # path/way‑point broken
                    self.requery_llm_and_merge()
                self.publish_current_path()
        except Exception as e:
            self.get_logger().error(f"obstacle parse error: {e}")

    def execute_cb(self, goal_handle):
        self.get_logger().info('Nav2 compute_path request')
        self.plan_and_publish()
        result = ComputePathToPose.Result()
        result.path = self.build_path_msg(self.current_path)
        goal_handle.succeed()
        return result

    # ── Grid builder ───────────────────────────────────────────────────────
    @staticmethod
    def build_grid(map_msg: OccupancyGrid):
        w, h = map_msg.info.width, map_msg.info.height
        g = [[1.0] * w for _ in range(h)]
        for j in range(h):
            for i in range(w):
                occ = map_msg.data[j*w + i]
                if occ > 50:
                    g[j][i] = math.inf
        return g, w, h

    # ── Planning pipeline ──────────────────────────────────────────────────
    def sanitize_waypoints(self, W):
        """ Clamp to free cells, enforce min spacing """
        out = []
        last = None
        for i, j in W:
            if not (0 <= i < self.w and 0 <= j < self.h):
                continue
            if self.grid[j][i] == math.inf:
                continue
            if last and abs(i-last[0])+abs(j-last[1]) < 2:   # too close
                continue
            out.append((i, j)); last = (i, j)
        # always keep at least start→goal
        return out

    def plan_and_publish(self, force=False):
        if self.map_msg is None or self.pose_grid is None:
            return
        if not self.waypoints:
            # first call: fetch way‑points from LLM
            self.waypoints = self.request_llm_waypoints(self.pose_grid)
        if not force and self.current_path and not self.is_obstructed():
            return  # still valid

        start = self.pose_grid
        all_points = [start] + self.waypoints
        full_path = []
        self.dstar = None  # rebuild fresh
        for a, b in zip(all_points[:-1], all_points[1:]):
            if self.dstar is None:
                self.dstar = DStarLite(copy.deepcopy(self.grid),
                                       self.w, self.h, a, b,
                                       lambda u,v: abs(u[0]-v[0]) + abs(u[1]-v[1]))
            else:
                self.dstar.s_goal = b      # reuse instance
                self.dstar.update_vertex(b)
                self.dstar.repair()
            seg = self.dstar.extract_path()
            if not seg:
                self.get_logger().warn('Segment planning failed – requesting new LLM plan')
                self.requery_llm_and_merge()
                return
            if full_path: seg = seg[1:]
            full_path.extend(seg)
            # advance anchor for next leg
            self.dstar.move_start(b)

        self.current_path = full_path
        self.publish_current_path()

    # ── Replanning helpers ────────────────────────────────────────────────
    def is_obstructed(self):
        """True if any cell in path or in remaining way‑points became lethal."""
        for i, j in self.current_path:
            if self.grid[j][i] == math.inf:
                return True
        for i, j in self.waypoints:
            if self.grid[j][i] == math.inf:
                return True
        return False

    def requery_llm_and_merge(self):
        new_wp = self.request_llm_waypoints(self.pose_grid)
        # merge: keep prefix already reached & unobstructed
        merged = []
        for wp in self.waypoints:
            if self.grid[wp[1]][wp[0]] == math.inf:
                break
            merged.append(wp)
        merged.extend(new_wp)
        self.waypoints = self.sanitize_waypoints(merged)
        self.plan_and_publish(force=True)

    # ── LLM interface (local call – no ROS change) ────────────────────────
    def request_llm_waypoints(self, start_grid):
        try:
            import ollama
            client = ollama.Client()
            sx, sy = start_grid
            gx, gy = self.world_to_grid(
                self.map_msg.info.origin.position.x + self.map_msg.info.width * self.map_msg.info.resolution,
                self.map_msg.info.origin.position.y + self.map_msg.info.height * self.map_msg.info.resolution)
            prompt = (
                "You are a navigation assistant. "
                "Map is a {}x{} grid, obstacles are cells with cost=inf. "
                "Give a JSON list of 2‑D integer way‑points (x,y) "
                "from start {} to goal {} spaced ≥3 cells apart and avoiding obstacles. "
                "Return ONLY the list."
            ).format(self.w, self.h, [sx,sy], [gx,gy])
            resp = client.generate(model=self.llm_model, prompt=prompt, stream=False)
            text = resp['response'].strip()
            match = re.search(r'\[.*\]', text, re.DOTALL)
            wp = json.loads(match.group(0)) if match else []
            waypoints = [(int(p['x']), int(p['y'])) for p in wp]
            return self.sanitize_waypoints(waypoints)
        except Exception as e:
            self.get_logger().error(f"LLM query failed: {e}")
            return []

    # ── Path publishing ────────────────────────────────────────────────────
    def build_path_msg(self, cells):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, j in cells:
            x, y = self.grid_to_world(i, j)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def publish_current_path(self):
        msg = self.build_path_msg(self.current_path)
        self.path_pub.publish(msg)
        self.get_logger().info(f'Published path with {len(msg.poses)} poses.')

#
# ─────────────────────────────────── main ───────────────────────────────────────
#
def main(args=None):
    rclpy.init(args=args)
    node = LlmDstarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
