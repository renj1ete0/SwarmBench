from swarmbench import BaseSwarmController, DroneStatus


class SwarmController(BaseSwarmController):
    def initialize(self, game_info):
        # Store goal center for reference
        self.goal_center = game_info.target_goal.center

    def step(self, state):
        # Simple strategy: move each active drone towards the goal center
        commands = {}
        for drone in state.own_drones:
            if drone.status is not DroneStatus.ACTIVE:
                continue
            dx = self.goal_center[0] - drone.position[0]
            dy = self.goal_center[1] - drone.position[1]
            # Normalize to max speed
            dist = (dx**2 + dy**2)**0.5
            if dist == 0:
                ax, ay = 0.0, 0.0
            else:
                # Scale to max speed 5.0 m/s for FAST, 2.5 m/s for SLOW
                max_speed = 5.0 if drone.type == "FAST" else 2.5
                ax = max_speed * dx / dist
                ay = max_speed * dy / dist
            commands[drone.id] = (ax, ay)
        return commands
