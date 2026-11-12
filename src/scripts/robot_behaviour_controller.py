# Robot controller using subsumption architecture
# 3 layers: wander (base), wall follow (middle), obstacle avoid (top priority)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

import math
import random
import time
import csv
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Tuple, List


class BehaviourName(Enum):
    WANDER = "wander"
    WALL_FOLLOW = "wall_follow"
    OBSTACLE_AVOID = "obstacle_avoid"


@dataclass
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass
class BehaviourOutput:
    active: bool = False
    command: VelocityCommand = field(default_factory=VelocityCommand)
    name: BehaviourName = BehaviourName.WANDER


@dataclass
class TestMetrics:
    # for tracking performance during testing
    start_time: float = 0.0
    collision_threshold: float = 0.25
    behaviour_switches: int = 0
    last_behaviour: Optional[BehaviourName] = None
    distance_travelled: float = 0.0
    last_x: float = 0.0
    last_y: float = 0.0
    positions_initialised: bool = False
    behaviour_time: dict = field(default_factory=lambda: {
        BehaviourName.WANDER: 0.0,
        BehaviourName.WALL_FOLLOW: 0.0,
        BehaviourName.OBSTACLE_AVOID: 0.0,
    })
    last_metric_time: float = 0.0
    min_distance_log: List[float] = field(default_factory=list)
    near_collision_events: int = 0


# helper function to get lidar readings in a specific arc
def get_sector(ranges, angle_min, angle_inc, start_deg, end_deg):
    lo = math.radians(start_deg)
    hi = math.radians(end_deg)
    out = []
    for i in range(len(ranges)):
        a = angle_min + i * angle_inc
        if lo <= a <= hi and 0.08 < ranges[i] < 10.0:
            out.append((i, ranges[i]))
    return out


# Layer 0 - Wander behaviour (lowest priority)
class WanderBehaviour:
    def __init__(self):
        self.forward_speed = 0.5
        self.turn_rate = 0.0
        self.turn_timer = 0.0
        self.turn_duration = 0.0
        self.time_since_turn = 0.0
        self.turn_interval = 4.0  # make a random turn every 4 seconds
        self._pick_new_turn()

    def _pick_new_turn(self):
        # pick random turn direction and how long to turn for
        self.turn_rate = random.uniform(-1.0, 1.0)
        self.turn_duration = random.uniform(0.5, 1.5)
        self.turn_timer = 0.0

    def compute(self, dt: float) -> BehaviourOutput:
        # wander is always active since its the base layer
        self.time_since_turn += dt
        output = BehaviourOutput(active=True, name=BehaviourName.WANDER)

        if self.time_since_turn >= self.turn_interval:
            self._pick_new_turn()
            self.time_since_turn = 0.0

        if self.turn_timer < self.turn_duration:
            self.turn_timer += dt
            output.command = VelocityCommand(
                linear_x=self.forward_speed * 0.5,
                angular_z=self.turn_rate
            )
        else:
            output.command = VelocityCommand(
                linear_x=self.forward_speed,
                angular_z=0.0
            )
        return output


# Layer 1 - Wall following (medium priority)
class WallFollowBehaviour:
    def __init__(self):
        self.desired_distance = 0.9  # try to stay 0.9m from wall
        self.detect_distance = 1.6
        self.kp = 1.8  # proportional gain
        self.base_speed = 0.35

    def compute(self, ranges, angle_min, angle_inc) -> BehaviourOutput:
        output = BehaviourOutput(name=BehaviourName.WALL_FOLLOW)

        if not ranges:
            return output

        # check if front is blocked first
        front = get_sector(ranges, angle_min, angle_inc, -40, 40)
        if front:
            if min(d for _, d in front) < 0.9:
                return output  # let obstacle avoidance handle it

        # look at right side for wall
        right = get_sector(ranges, angle_min, angle_inc, -100, -20)
        if not right:
            return output

        dists = [d for _, d in right]
        min_right = min(dists)
        avg_right = sum(dists) / len(dists)

        if min_right > self.detect_distance:
            return output
        
        # make sure its actually a wall not just a cylinder
        # need at least 5 lidar points hitting it
        wall_readings = [d for d in dists if d < self.detect_distance]
        if len(wall_readings) < 5:
            return output

        output.active = True

        # proportional controller to maintain distance
        error = avg_right - self.desired_distance
        angular_z = max(-0.8, min(0.8, -self.kp * error))

        # slow down when close to wall
        closeness = min_right / self.desired_distance
        speed = self.base_speed * max(0.3, min(1.0, closeness))

        output.command = VelocityCommand(linear_x=speed, angular_z=angular_z)
        return output


# Layer 2 - Obstacle avoidance (highest priority)
class ObstacleAvoidBehaviour:
    def __init__(self):
        self.danger_dist = 0.4  # really close
        self.caution_dist = 0.7  # start reacting
        self.turn_speed = 2.2
        self.committed_dir = None  # remember which way we're turning
        self.commit_timer = 0.0
        self.commit_duration = 1.5  # stick with turn for this long

    def compute(self, ranges, angle_min, angle_inc, dt=0.05) -> BehaviourOutput:
        output = BehaviourOutput(name=BehaviourName.OBSTACLE_AVOID)

        if not ranges:
            self.committed_dir = None
            return output

        # only look at front arc
        front = get_sector(ranges, angle_min, angle_inc, -30, 30)
        if not front:
            self.committed_dir = None
            return output

        min_front = min(d for _, d in front)

        # if we're already turning, keep going
        if self.committed_dir is not None:
            self.commit_timer += dt
            if self.commit_timer < self.commit_duration and min_front < self.caution_dist:
                output.active = True
                output.command = VelocityCommand(
                    linear_x=0.0,
                    angular_z=self.turn_speed * self.committed_dir
                )
                return output
            else:
                self.committed_dir = None
                self.commit_timer = 0.0

        if min_front > self.caution_dist:
            return output

        output.active = True

        # compare left vs right to decide which way to turn
        left = get_sector(ranges, angle_min, angle_inc, 10, 170)
        right = get_sector(ranges, angle_min, angle_inc, -170, -10)
        avg_l = sum(d for _, d in left) / len(left) if left else 0
        avg_r = sum(d for _, d in right) / len(right) if right else 0
        
        # add hysteresis so it doesn't flip flop
        if self.committed_dir is not None:
            if self.committed_dir > 0:
                turn_dir = -1.0 if avg_r > avg_l * 1.25 else 1.0
            else:
                turn_dir = 1.0 if avg_l > avg_r * 1.25 else -1.0
        else:
            turn_dir = 1.0 if avg_l >= avg_r else -1.0

        self.committed_dir = turn_dir
        self.commit_timer = 0.0

        # back up a bit if really close
        if min_front < self.danger_dist:
            output.command = VelocityCommand(
                linear_x=-0.1,
                angular_z=self.turn_speed * turn_dir
            )
        else:
            output.command = VelocityCommand(
                linear_x=0.05,
                angular_z=self.turn_speed * 0.7 * turn_dir
            )
        return output


# Subsumption arbiter - picks which behaviour to use
class SubsumptionArbiter:
    def __init__(self):
        self.layers: List[Tuple[int, BehaviourName]] = []

    def add_layer(self, priority: int, name: BehaviourName):
        self.layers.append((priority, name))
        self.layers.sort(key=lambda x: x[0], reverse=True)

    def arbitrate(self, outputs: dict) -> Tuple[BehaviourName, VelocityCommand]:
        # go through layers from highest to lowest priority
        # first one that's active wins
        for _, name in self.layers:
            if name in outputs and outputs[name].active:
                return (name, outputs[name].command)
        return (BehaviourName.WANDER, VelocityCommand())


# Main ROS2 node
class RobotBehaviourController(Node):
    def __init__(self):
        super().__init__('robot_behaviour_controller')

        # set up publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cam_pan_pub = self.create_publisher(Float64, '/camera_pan_cmd', 10)
        self.behav_pub = self.create_publisher(String, '/active_behaviour', 10)

        # set up subscribers
        self.create_subscription(LaserScan, '/lidar', self._on_lidar, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Image, '/camera', self._on_camera, 10)

        self.latest_lidar: Optional[LaserScan] = None
        self.camera_frames = 0

        # create behaviour instances
        self.wander = WanderBehaviour()
        self.wall_follow = WallFollowBehaviour()
        self.obstacle_avoid = ObstacleAvoidBehaviour()

        # set up arbiter with priorities
        self.arbiter = SubsumptionArbiter()
        self.arbiter.add_layer(0, BehaviourName.WANDER)
        self.arbiter.add_layer(1, BehaviourName.WALL_FOLLOW)
        self.arbiter.add_layer(2, BehaviourName.OBSTACLE_AVOID)

        self.metrics = TestMetrics(start_time=time.time())

        # camera sweep
        self.cam_angle = 0.0
        self.cam_dir = 1.0

        # run control loop at 20Hz
        self.dt = 0.05
        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info("Controller started")
        self.get_logger().info("Layer 2: Obstacle Avoidance")
        self.get_logger().info("Layer 1: Wall Following")
        self.get_logger().info("Layer 0: Wander")

    def _on_lidar(self, msg):
        self.latest_lidar = msg

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        m = self.metrics
        if not m.positions_initialised:
            m.last_x, m.last_y = x, y
            m.positions_initialised = True
        else:
            dx, dy = x - m.last_x, y - m.last_y
            m.distance_travelled += math.sqrt(dx*dx + dy*dy)
            m.last_x, m.last_y = x, y

    def _on_camera(self, msg):
        self.camera_frames += 1
        if self.camera_frames % 150 == 0:
            self.get_logger().info(f"Camera frames: {self.camera_frames}")

    def _control_loop(self):
        ranges, a_min, a_inc = [], 0.0, 0.0
        if self.latest_lidar:
            ranges = list(self.latest_lidar.ranges)
            a_min = self.latest_lidar.angle_min
            a_inc = self.latest_lidar.angle_increment

        # compute all behaviours
        outputs = {
            BehaviourName.WANDER:
                self.wander.compute(self.dt),
            BehaviourName.WALL_FOLLOW:
                self.wall_follow.compute(ranges, a_min, a_inc),
            BehaviourName.OBSTACLE_AVOID:
                self.obstacle_avoid.compute(ranges, a_min, a_inc, self.dt),
        }

        # let arbiter pick winner
        active, cmd = self.arbiter.arbitrate(outputs)

        # publish velocity command
        twist = Twist()
        twist.linear.x = cmd.linear_x
        twist.angular.z = cmd.angular_z
        self.cmd_vel_pub.publish(twist)

        # publish which behaviour is active
        self.behav_pub.publish(String(data=active.value))

        # sweep camera back and forth
        self.cam_angle += 0.04 * self.cam_dir
        if abs(self.cam_angle) > 1.2:
            self.cam_dir *= -1.0
        self.cam_pan_pub.publish(Float64(data=self.cam_angle))

        # update metrics
        self._update_metrics(active, ranges)

    def _update_metrics(self, active, ranges):
        now = time.time()
        m = self.metrics
        if m.last_behaviour and active != m.last_behaviour:
            m.behaviour_switches += 1
        m.last_behaviour = active
        if m.last_metric_time > 0:
            m.behaviour_time[active] += now - m.last_metric_time
        m.last_metric_time = now
        if ranges:
            valid = [r for r in ranges if 0.08 < r < 10.0]
            if valid:
                md = min(valid)
                m.min_distance_log.append(md)
                if md < m.collision_threshold:
                    m.near_collision_events += 1
        elapsed = now - m.start_time
        if int(elapsed) % 20 == 0 and int(elapsed) > 0:
            self._print_metrics()

    def _print_metrics(self):
        m = self.metrics
        t = time.time() - m.start_time
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"Time: {t:.0f}s | Current: {m.last_behaviour.value}\n"
            f"Distance: {m.distance_travelled:.1f}m | Switches: {m.behaviour_switches}\n"
            f"Near collisions: {m.near_collision_events}\n"
            f"Wander: {m.behaviour_time[BehaviourName.WANDER]:.1f}s | "
            f"Wall: {m.behaviour_time[BehaviourName.WALL_FOLLOW]:.1f}s | "
            f"Avoid: {m.behaviour_time[BehaviourName.OBSTACLE_AVOID]:.1f}s\n"
            f"{'='*50}")

    def save_metrics_csv(self, path="test_results.csv"):
        m = self.metrics
        elapsed = time.time() - m.start_time
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["Metric", "Value"])
            w.writerow(["Total Time (s)", f"{elapsed:.2f}"])
            w.writerow(["Distance Travelled (m)", f"{m.distance_travelled:.2f}"])
            w.writerow(["Behaviour Switches", m.behaviour_switches])
            w.writerow(["Near-Collision Events", m.near_collision_events])
            w.writerow(["Collision Threshold (m)", m.collision_threshold])
            w.writerow(["Time Wander (s)", f"{m.behaviour_time[BehaviourName.WANDER]:.2f}"])
            w.writerow(["Time Wall Follow (s)", f"{m.behaviour_time[BehaviourName.WALL_FOLLOW]:.2f}"])
            w.writerow(["Time Obstacle Avoid (s)", f"{m.behaviour_time[BehaviourName.OBSTACLE_AVOID]:.2f}"])
            if m.min_distance_log:
                avg = sum(m.min_distance_log) / len(m.min_distance_log)
                w.writerow(["Avg Min Distance (m)", f"{avg:.3f}"])
                w.writerow(["Absolute Min Distance (m)", f"{min(m.min_distance_log):.3f}"])
            w.writerow(["Camera Frames Received", self.camera_frames])
        self.get_logger().info(f"Saved metrics to {path}")

    def destroy_node(self):
        self.save_metrics_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviourController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        node.save_metrics_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
