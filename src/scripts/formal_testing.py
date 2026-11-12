# Runs 3 scenarios x 3 trials and saves results to CSV
# Uses a single polling timer instead of creating new timers

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import math
import csv
from dataclasses import dataclass, field
from typing import List


@dataclass
class TrialData:
    scenario: str = ""
    trial: int = 0
    duration: float = 0.0
    distance: float = 0.0
    near_collisions: int = 0
    switches: int = 0
    min_dist: float = float('inf')
    avg_min_dist: float = 0.0
    t_wander: float = 0.0
    t_wall_follow: float = 0.0
    t_avoid: float = 0.0
    min_dists: List[float] = field(default_factory=list)


class FormalTester(Node):
    def __init__(self):
        super().__init__('formal_tester')

        self.create_subscription(LaserScan, '/lidar', self._lidar_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(String, '/active_behaviour', self._behav_cb, 10)

        self.trial = TrialData()
        self.results: List[TrialData] = []
        self.recording = False
        self.trial_start = 0.0

        self.lx, self.ly = 0.0, 0.0
        self.pos_init = False
        self.last_behav = ""
        self.last_behav_t = 0.0
        self.threshold = 0.25

        # test queue
        self.queue = []
        for name, dur in [("Open Space", 30), ("Obstacle Course", 45), ("Wall Following", 30)]:
            for t in range(1, 4):
                self.queue.append((name, t, dur))
        self.queue_idx = 0

        # state machine
        self.state = "waiting"
        self.state_start = time.time()
        self.wait_time = 5.0
        self.gap_time = 3.0

        # one timer that drives everything
        self.create_timer(0.1, self._tick)

        self.get_logger().info(f"Tester ready. {len(self.queue)} trials queued.")
        self.get_logger().info(f"Starting in {self.wait_time:.0f} seconds...\n")

    def _lidar_cb(self, msg):
        if not self.recording:
            return
        valid = [r for r in msg.ranges if 0.08 < r < 10.0]
        if not valid:
            return
        md = min(valid)
        self.trial.min_dists.append(md)
        if md < self.trial.min_dist:
            self.trial.min_dist = md
        if md < self.threshold:
            self.trial.near_collisions += 1

    def _odom_cb(self, msg):
        if not self.recording:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not self.pos_init:
            self.lx, self.ly = x, y
            self.pos_init = True
        else:
            self.trial.distance += math.sqrt((x - self.lx)**2 + (y - self.ly)**2)
            self.lx, self.ly = x, y

    def _behav_cb(self, msg):
        if not self.recording:
            return
        now = time.time()
        b = msg.data
        if self.last_behav and b != self.last_behav:
            self.trial.switches += 1
        if self.last_behav and self.last_behav_t > 0:
            dt = now - self.last_behav_t
            if self.last_behav == "wander":
                self.trial.t_wander += dt
            elif self.last_behav == "wall_follow":
                self.trial.t_wall_follow += dt
            elif self.last_behav == "obstacle_avoid":
                self.trial.t_avoid += dt
        self.last_behav = b
        self.last_behav_t = now

    def _tick(self):
        now = time.time()
        elapsed = now - self.state_start

        if self.state == "waiting":
            if elapsed >= self.wait_time:
                self.get_logger().info(
                    f"\n{'='*60}\n"
                    f"  Starting {len(self.queue)} test trials\n"
                    f"{'='*60}\n")
                self._start_trial()

        elif self.state == "running":
            _, _, duration = self.queue[self.queue_idx]
            if elapsed >= duration:
                self._end_trial()
                self.queue_idx += 1
                if self.queue_idx >= len(self.queue):
                    self.state = "done"
                    self._finish()
                else:
                    self.state = "gap"
                    self.state_start = now

        elif self.state == "gap":
            if elapsed >= self.gap_time:
                self._start_trial()

    def _start_trial(self):
        name, num, dur = self.queue[self.queue_idx]
        self.trial = TrialData(scenario=name, trial=num)
        self.recording = True
        self.pos_init = False
        self.last_behav = ""
        self.last_behav_t = 0.0
        self.trial_start = time.time()
        self.state = "running"
        self.state_start = time.time()
        self.get_logger().info(f"  >> {name} trial {num} ({dur}s)")

    def _end_trial(self):
        self.recording = False
        self.trial.duration = time.time() - self.trial_start
        if self.trial.min_dists:
            self.trial.avg_min_dist = sum(self.trial.min_dists) / len(self.trial.min_dists)
        self.results.append(self.trial)
        t = self.trial
        md = f"{t.min_dist:.3f}" if t.min_dist < 100 else "n/a"
        self.get_logger().info(
            f"     dist={t.distance:.1f}m  collisions={t.near_collisions}  "
            f"switches={t.switches}  min_d={md}m")

    def _finish(self):
        self.get_logger().info("\nAll done, saving...")
        self._save()

    def _save(self):
        path = "formal_test_results.csv"
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                "Scenario", "Trial", "Duration(s)", "Distance(m)",
                "Near-Collisions", "Switches", "Min Dist(m)",
                "Avg Min Dist(m)", "Wander(s)", "Wall Follow(s)", "Avoid(s)"])
            for t in self.results:
                md = f"{t.min_dist:.3f}" if t.min_dist < 100 else "n/a"
                ad = f"{t.avg_min_dist:.3f}" if t.avg_min_dist > 0 else "n/a"
                w.writerow([
                    t.scenario, t.trial, f"{t.duration:.1f}",
                    f"{t.distance:.2f}", t.near_collisions, t.switches,
                    md, ad,
                    f"{t.t_wander:.1f}", f"{t.t_wall_follow:.1f}",
                    f"{t.t_avoid:.1f}"])

        self.get_logger().info(f"Results saved to {path}")

        # summary table
        self.get_logger().info(f"\n{'='*75}")
        self.get_logger().info(
            f"  {'Scenario':<20} {'T':>2} {'Dur':>5} {'Dist':>7} {'Coll':>5} "
            f"{'Sw':>4} {'MinD':>6} {'W':>6} {'WF':>6} {'OA':>6}")
        self.get_logger().info(f"  {'-'*70}")
        for t in self.results:
            md = f"{t.min_dist:.3f}" if t.min_dist < 100 else "  n/a"
            self.get_logger().info(
                f"  {t.scenario:<20} {t.trial:>2} {t.duration:>4.0f}s "
                f"{t.distance:>6.1f}m {t.near_collisions:>5} {t.switches:>4} "
                f"{md:>6} {t.t_wander:>5.1f}s {t.t_wall_follow:>5.1f}s "
                f"{t.t_avoid:>5.1f}s")
        self.get_logger().info(f"{'='*75}")

        # averages per scenario
        scenarios = {}
        for t in self.results:
            scenarios.setdefault(t.scenario, []).append(t)
        self.get_logger().info(f"\n  Averages:")
        for name, trials in scenarios.items():
            n = len(trials)
            self.get_logger().info(
                f"    {name:<20} dist={sum(t.distance for t in trials)/n:.1f}m  "
                f"collisions={sum(t.near_collisions for t in trials)/n:.1f}  "
                f"switches={sum(t.switches for t in trials)/n:.0f}")
        self.get_logger().info(f"\nDone. Ctrl+C to exit.")


def main(args=None):
    rclpy.init(args=args)
    node = FormalTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.results:
            node._save()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
