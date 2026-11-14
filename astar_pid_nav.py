#!/usr/bin/env python3
# =============================================================================
# File: astar_pid_nav.py
# Purpose: Navigation autonome TurtleBot3 sous Gazebo (monde "house") :
#          - charge une carte (map.yaml + .pgm)
#          - planifie le plus court chemin avec A* (8-connexe, obstacles gonfles)
#          - suit le chemin avec des regulateurs PID (distance + cap) sur /cmd_vel
#
# Run:
#   source /opt/ros/humble/setup.bash
#   python3 astar_pid_nav.py --ros-args \
#       -p map_file:=$HOME/house_map.yaml -p goal_x:=3.0 -p goal_y:=1.5 \
#       -p use_sim_time:=true
# =============================================================================

import heapq
import math
import os

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

SQ2 = math.sqrt(2.0)


# ----------------------------------------------------------------------------
# Utilitaires
# ----------------------------------------------------------------------------
def yaw_from_quat(qx, qy, qz, qw) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def read_pgm(path):
    """Lit une image PGM binaire (P5) -> tableau numpy uint8 (h, w)."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':          # commentaire
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1                                     # un espace apres maxval
    if tokens[0] != b'P5':
        raise ValueError("Format PGM non supporte (P5 attendu)")
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)


# ----------------------------------------------------------------------------
# Carte d'occupation + A*
# ----------------------------------------------------------------------------
class GridMap:
    def __init__(self, yaml_path: str, inflate_radius: float,
                 unknown_free: bool = False, unknown_cost: float = 3.0):
        yaml_path = os.path.expanduser(yaml_path)
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        img = read_pgm(img_path)

        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        negate = int(meta.get('negate', 0))
        free_t = float(meta.get('free_thresh', 0.25))

        # probabilite d'occupation (0 = libre, 1 = occupe)
        p = (img / 255.0) if negate else ((255.0 - img) / 255.0)
        p = np.flipud(p)                         # ligne 0 = y le plus bas
        self.h, self.w = p.shape

        # Libre seulement si clairement libre ; inconnu (gris 205) = bloque
        occ_t = float(meta.get('occupied_thresh', 0.65))
        self.occ = p >= occ_t                                    # vrais obstacles
        self.unknown = (p >= 0.19) & (p < occ_t)                 # zones non explorees (gris)
        self.unknown_cost = float(unknown_cost)
        if unknown_free:
            free = p < occ_t        # inconnu = traversable (comme Nav2), mais plus cher
        else:
            free = p < min(free_t, 0.19)
        blocked = ~free
        r = int(math.ceil(inflate_radius / self.res))
        self.inflated = self._inflate(blocked, r)

    @staticmethod
    def _inflate(blocked, r):
        out = blocked.copy()
        h, w = blocked.shape
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    out[max(0, dy):min(h, h + dy), max(0, dx):min(w, w + dx)] |= \
                        blocked[max(0, -dy):min(h, h - dy), max(0, -dx):min(w, w - dx)]
        return out

    def clear_around(self, c, radius):
        """Libere la marge de securite autour du robot (sauf les vrais obstacles),
        pour que le depart ne soit jamais deplace de l'autre cote d'un mur."""
        r = int(math.ceil(radius / self.res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                x, y = c[0] + dx, c[1] + dy
                if dx * dx + dy * dy <= r * r and 0 <= x < self.w and 0 <= y < self.h \
                        and not self.occ[y, x]:
                    self.inflated[y, x] = False

    # conversions monde <-> cellule (col, ligne)
    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.ox) / self.res)),
                int(math.floor((y - self.oy) / self.res)))

    def cell_to_world(self, c):
        return (self.ox + (c[0] + 0.5) * self.res,
                self.oy + (c[1] + 0.5) * self.res)

    def in_bounds(self, c):
        return 0 <= c[0] < self.w and 0 <= c[1] < self.h

    def is_free(self, c):
        return self.in_bounds(c) and not self.inflated[c[1], c[0]]

    def nearest_free(self, c, max_r=60):
        if self.is_free(c):
            return c
        for rr in range(1, max_r):
            for dy in range(-rr, rr + 1):
                for dx in range(-rr, rr + 1):
                    if max(abs(dx), abs(dy)) != rr:
                        continue
                    n = (c[0] + dx, c[1] + dy)
                    if self.is_free(n):
                        return n
        return None

    def astar(self, start, goal):
        """A* 8-connexe, heuristique octile. Retourne la liste de cellules ou None."""
        def heur(c):
            dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
            return (dx + dy) + (SQ2 - 2.0) * min(dx, dy)

        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]
        open_heap = [(heur(start), 0.0, start)]
        g = {start: 0.0}
        parent = {}
        closed = set()

        while open_heap:
            _, gc, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == goal:
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                return path[::-1]
            closed.add(cur)

            for dx, dy, cost in moves:
                nx, ny = cur[0] + dx, cur[1] + dy
                if not (0 <= nx < self.w and 0 <= ny < self.h):
                    continue
                if self.inflated[ny, nx]:
                    continue
                # pas de coupe de coin en diagonale
                if dx != 0 and dy != 0 and (self.inflated[cur[1], nx] or self.inflated[ny, cur[0]]):
                    continue
                nxt = (nx, ny)
                ng = gc + cost * (self.unknown_cost if self.unknown[ny, nx] else 1.0)
                if ng < g.get(nxt, float('inf')):
                    g[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(open_heap, (ng + heur(nxt), ng, nxt))
        return None

    def line_free(self, a, b):
        """Bresenham : la droite a->b traverse-t-elle seulement des cellules libres ?"""
        x0, y0 = a
        x1, y1 = b
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if self.inflated[y0, x0]:
                return False
            if (x0, y0) == (x1, y1):
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def simplify(self, path):
        """Supprime les points intermediaires inutiles (visibilite directe)."""
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.line_free(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out


# ----------------------------------------------------------------------------
# Regulateur PID
# ----------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, out_min, out_max, i_max=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max, self.i_max = out_min, out_max, i_max
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def step(self, err, dt):
        self.integral = clamp(self.integral + err * dt, -self.i_max, self.i_max)  # anti-windup
        deriv = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        return clamp(out, self.out_min, self.out_max)


# ----------------------------------------------------------------------------
# Noeud ROS 2
# ----------------------------------------------------------------------------
class AStarPID(Node):
    def __init__(self) -> None:
        super().__init__('astar_pid_nav')

        # --- parametres ---
        self.declare_parameter('map_file', '~/house_map.yaml')
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('init_x', 0.0)      # pose initiale du robot dans la carte
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('inflate_radius', 0.28)
        self.declare_parameter('kp_lin', 0.8)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.05)
        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.1)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('tol_pos', 0.10)
        self.declare_parameter('wp_tol', 0.20)
        # use_tf=True : pose du robot dans la carte lue via TF (map -> base_footprint),
        # fournie par AMCL (Nav2). Plus besoin de init_x/init_y/init_yaw.
        self.declare_parameter('use_tf', False)
        # unknown_free=True : les zones grises (non explorees) sont traversables mais
        # evitees si possible (cout x unknown_cost). A utiliser si le robot demarre dans
        # une zone non cartographiee.
        self.declare_parameter('unknown_free', False)
        self.declare_parameter('unknown_cost', 3.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        g = lambda n: self.get_parameter(n).value
        self.goal = (float(g('goal_x')), float(g('goal_y')))
        self.init = (float(g('init_x')), float(g('init_y')), float(g('init_yaw')))
        self.v_max = float(g('v_max'))
        self.w_max = float(g('w_max'))
        self.tol_pos = float(g('tol_pos'))
        self.wp_tol = float(g('wp_tol'))
        self.use_tf = bool(g('use_tf'))
        self.map_frame = str(g('map_frame'))
        self.base_frame = str(g('base_frame'))

        self.pid_lin = PID(float(g('kp_lin')), float(g('ki_lin')), float(g('kd_lin')),
                           0.0, self.v_max)
        self.pid_ang = PID(float(g('kp_ang')), float(g('ki_ang')), float(g('kd_ang')),
                           -self.w_max, self.w_max)

        # --- carte ---
        self.map = GridMap(str(g('map_file')), float(g('inflate_radius')),
                           bool(g('unknown_free')), float(g('unknown_cost')))
        self.get_logger().info(
            f"[A*PID] Carte {self.map.w}x{self.map.h} cellules, res={self.map.res} m")

        # --- ROS I/O ---
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 1)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        if self.use_tf:
            self.tf_buf = Buffer()
            self.tf_listener = TransformListener(self.tf_buf, self)
            self.get_logger().info("[A*PID] Pose lue via TF (AMCL) : map -> base_footprint")
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_step)

        # --- etat ---
        self.o0 = None            # premiere odometrie (pour recaler dans la carte)
        self.x = self.y = self.yaw = None
        self.waypoints = None
        self.idx = 0
        self.done = False
        self._t_log = 0

    # odom -> pose dans le repere de la carte
    def update_from_tf(self) -> None:
        try:
            t = self.tf_buf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return
        tr, q = t.transform.translation, t.transform.rotation
        self.x, self.y = tr.x, tr.y
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def odom_cb(self, msg: Odometry) -> None:
        if self.use_tf:
            return
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        oyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.o0 is None:
            self.o0 = (ox, oy, oyaw)
        x0, y0, yaw0 = self.o0
        dx, dy = ox - x0, oy - y0
        lx = math.cos(-yaw0) * dx - math.sin(-yaw0) * dy
        ly = math.sin(-yaw0) * dx + math.cos(-yaw0) * dy
        ix, iy, iyaw = self.init
        self.x = ix + math.cos(iyaw) * lx - math.sin(iyaw) * ly
        self.y = iy + math.sin(iyaw) * lx + math.cos(iyaw) * ly
        self.yaw = wrap_pi(iyaw + oyaw - yaw0)

    def stop(self) -> None:
        self.pub_cmd.publish(Twist())

    def plan(self) -> bool:
        m = self.map
        s_cell = m.world_to_cell(self.x, self.y)
        g_cell = m.world_to_cell(*self.goal)
        if not m.in_bounds(g_cell):
            self.get_logger().error("[A*PID] But hors de la carte.")
            return False
        m.clear_around(s_cell, 0.35)
        s_free = m.nearest_free(s_cell)
        g_free = m.nearest_free(g_cell)
        if s_free is None or g_free is None:
            self.get_logger().error("[A*PID] Depart ou but dans une zone bloquee.")
            return False
        if g_free != g_cell:
            self.get_logger().warn("[A*PID] But dans un obstacle : deplace a la cellule libre la plus proche.")

        self.get_logger().info(
            f"[A*PID] Pose robot dans la carte = ({self.x:.2f}, {self.y:.2f}), "
            f"but = ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
        path = m.astar(s_free, g_free)
        if path is None:
            self.get_logger().error(
                "[A*PID] Aucun chemin trouve entre la pose du robot et le but. "
                "Verifiez la pose du robot dans RViz (2D Pose Estimate) : si elle est "
                "fausse (ex. (0,0) par defaut), le robot est dans une zone fermee de la carte.")
            return False

        pts = [m.cell_to_world(c) for c in path]
        length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        simp = m.simplify(path)
        self.waypoints = [m.cell_to_world(c) for c in simp][1:]
        if g_free == g_cell:
            self.waypoints[-1] = self.goal          # but exact
        self.idx = 0
        self.get_logger().info(
            f"[A*PID] Chemin : {len(path)} cellules, {length:.2f} m, {len(self.waypoints)} waypoints")

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (px, py) in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = px, py
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)
        return True

    def control_step(self) -> None:
        if self.use_tf:
            self.update_from_tf()
        if self.x is None or self.done:
            return
        if self.waypoints is None:
            if not self.plan():
                self.done = True
                self.stop()
            return

        gx, gy = self.waypoints[self.idx]
        last = self.idx == len(self.waypoints) - 1
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        e_yaw = wrap_pi(math.atan2(dy, dx) - self.yaw)

        if dist <= (self.tol_pos if last else self.wp_tol):
            if last:
                self.done = True
                self.stop()
                self.get_logger().info("[A*PID] But atteint. Arret.")
                return
            self.idx += 1
            self.pid_lin.reset()
            self.pid_ang.reset()
            return

        w = self.pid_ang.step(e_yaw, self.dt)
        # waypoint intermediaire : vitesse de croisiere ; dernier : PID sur la distance
        v = self.pid_lin.step(dist, self.dt) if last else self.v_max
        v *= max(0.0, math.cos(e_yaw))
        if abs(e_yaw) > math.radians(60):          # mal oriente : pivoter sur place
            v = 0.0
        v = clamp(v, 0.0, self.v_max)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        self._t_log += 1
        if self._t_log >= 10:
            self._t_log = 0
            self.get_logger().info(
                f"[wp {self.idx + 1}/{len(self.waypoints)} dist={dist:.2f} m "
                f"e_yaw={e_yaw:+.2f}] pose=({self.x:.2f},{self.y:.2f}) v={v:.2f} w={w:+.2f}")

    def destroy_node(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AStarPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
