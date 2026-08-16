"""GPT-5 Mini High: compact community controller submission.

Authorship: this file, including its strategy and implementation, was
entirely coded by GPT 5 Mini High without human guidance.
"""

from __future__ import annotations

from math import hypot, sqrt

from swarmbench import BaseSwarmController, DroneStatus, DroneType


class SwarmController(BaseSwarmController):
    """A small deterministic hybrid: slow drones push the goal lanes while
    fast drones screen and locally intercept dangerous enemies.
    """

    def initialize(self, game_info):
        self.goal = game_info.target_goal
        self.own_goal = game_info.own_goal
        self.obstacles = tuple(game_info.obstacles)
        self.specs = dict(game_info.drone_specs)
        self.width = game_info.arena_width
        self.height = game_info.arena_height
        initial = tuple(game_info.own_initial_drones)
        ordered = sorted(initial, key=lambda d: (d.position[1], d.id))
        low = self.goal.y_min + 0.5
        high = self.goal.y_max - 0.5
        usable = max(1e-6, high - low)
        self.lanes = {
            drone.id: low + usable * (rank + 0.5) / max(1, len(ordered))
            for rank, drone in enumerate(ordered)
        }
        self.tick = 0

    @staticmethod
    def _distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _clamp(x, low, high):
        return min(high, max(low, x))

    def _accel_toward(self, drone, target):
        spec = self.specs[drone.drone_type]
        dx = target[0] - drone.position[0]
        dy = target[1] - drone.position[1]
        dist = hypot(dx, dy)
        if dist < 1e-9:
            desired_x = desired_y = 0.0
        else:
            desired_speed = min(spec.max_speed, sqrt(2.0 * spec.max_acceleration * dist))
            desired_x = dx / dist * desired_speed
            desired_y = dy / dist * desired_speed

        ax = 2.2 * (desired_x - drone.velocity[0])
        ay = 2.2 * (desired_y - drone.velocity[1])
        mag = hypot(ax, ay)
        if mag > spec.max_acceleration:
            scale = spec.max_acceleration / mag
            ax *= scale
            ay *= scale
        return (ax, ay)

    def step(self, state):
        self.tick += 1
        own = {d.id: d for d in state.own_drones if d.status is DroneStatus.ACTIVE}
        enemies = [d for d in state.opponent_drones if d.status is DroneStatus.ACTIVE]
        actions = {}

        for drone in own.values():
            if drone.drone_type is DroneType.SLOW:
                lane_y = self.lanes.get(drone.id, self.goal.center[1])
                target = (
                    self.goal.center[0],
                    self._clamp(lane_y, self.goal.y_min + 0.3, self.goal.y_max - 0.3),
                )
                actions[drone.id] = self._accel_toward(drone, target)
                continue

            # FAST drone: pick a nearby dangerous enemy or advance in a screening
            # posture toward the target goal.
            nearest = None
            nearest_d = float("inf")
            for e in enemies:
                d = self._distance(drone.position, e.position)
                if d < nearest_d:
                    nearest_d = d
                    nearest = e

            pursue_pos = None
            if nearest is not None:
                # If enemy is close or threatens our own goal, pursue.
                goal_dist = self._distance(nearest.position, self.own_goal.center)
                if goal_dist < 30.0 or nearest_d < 18.0:
                    pursue_pos = nearest.position

            if pursue_pos is not None:
                tx = self._clamp(pursue_pos[0], 0.3, self.width - 0.3)
                ty = self._clamp(pursue_pos[1], 0.3, self.height - 0.3)
                target = (tx, ty)
            else:
                # Screen slightly offset from the goal center to avoid perfect overlapping
                offset = 0.6 if drone.id % 2 == 0 else -0.6
                target = (
                    self._clamp(self.goal.center[0] - 2.0, 0.3, self.width - 0.3),
                    self._clamp(self.goal.center[1] + offset, 0.3, self.height - 0.3),
                )

            actions[drone.id] = self._accel_toward(drone, target)

        return actions
