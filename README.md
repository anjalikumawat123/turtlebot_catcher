# TurtleBot Catcher — Pursuit & Evasion Round 1

Minimal, working ROS 2 Catcher node for the TurtleBot 4 Pursuit & Evasion Challenge.

**Target environment:** Ubuntu 22.04 · ROS 2 Humble · Gazebo Ignition Fortress · TurtleBot 4 Lite

---

## A. Project File Structure

```
turtlebot_catcher/
├── package.xml                          # ROS 2 package manifest
├── setup.py                             # Python build config
├── setup.cfg                            # ament install paths
├── resource/
│   └── turtlebot_catcher                # ament resource marker (empty file)
├── turtlebot_catcher/
│   ├── __init__.py
│   └── catcher_node.py                  # ← MAIN CATCHER NODE
├── launch/
│   └── catcher.launch.py                # Launch file with all parameters
├── config/
│   └── catcher_params.yaml              # Tunable parameters
└── test/
    ├── test_catcher_standalone.py        # pytest integration tests (no Gazebo needed)
    └── fake_runner_publisher.py          # Publishes a moving /runner/pose for manual testing
```

Place this package inside a ROS 2 workspace:
```
~/ros2_ws/
└── src/
    └── turtlebot_catcher/   ← this directory
```

---

## B. How It Works

```
/runner/pose (PoseStamped)  ──►┐
/odom        (Odometry)     ──►│  CatcherNode  ──► /cmd_vel (Twist)
/scan        (LaserScan)    ──►┘
```

1. **Locates Runner** — subscribes to `/runner/pose` (PoseStamped).  
   The competition bridge or organiser node publishes this topic.  
   If they use a different name, pass `runner_topic:=/their/topic` at launch.

2. **Knows its own position** — from `/odom`.

3. **Computes bearing + distance** to the runner using basic geometry.

4. **Drives forward** with proportional control: speed ∝ distance, turn rate ∝ heading error.

5. **Stops** when distance ≤ `capture_distance` (default 0.5 m).

6. **Avoids obstacles** by checking the ±30° frontal LiDAR arc against `obstacle_distance`.

---

## C. Installation Commands

```bash
# ── 1. Install ROS 2 Humble (skip if already installed) ──────────────────
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# ── 2. Install TurtleBot 4 packages ───────────────────────────────────────
sudo apt install -y \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-bringup \
    ros-humble-irobot-create-msgs

# ── 3. Create workspace and clone this package ────────────────────────────
mkdir -p ~/ros2_ws/src
cp -r turtlebot_catcher ~/ros2_ws/src/

# ── 4. Install Python dependencies ────────────────────────────────────────
# (all deps are standard ROS 2 messages — no pip packages needed)
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## D. Build Commands

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Build only the catcher package
colcon build --packages-select turtlebot_catcher --symlink-install

# Source the workspace
source install/setup.bash
```

---

## E. Run Commands

### Option 1 — With TurtleBot 4 Gazebo simulation (full setup)

```bash
# Terminal 1: Start TurtleBot 4 Gazebo simulation
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

# Terminal 2: Start the runner bridge (competition-provided, or use fake runner below)
# The competition organiser will provide this node/topic.

# Terminal 3: Launch the Catcher
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot_catcher catcher.launch.py
```

### Option 2 — With params file
```bash
ros2 launch turtlebot_catcher catcher.launch.py \
    capture_distance:=0.5 \
    runner_topic:=/runner/pose \
    max_linear_speed:=0.3
```

### Option 3 — Direct run (no launch file)
```bash
ros2 run turtlebot_catcher catcher_node \
    --ros-args \
    --params-file ~/ros2_ws/src/turtlebot_catcher/config/catcher_params.yaml
```

### Override runner topic (if competition uses different name)
```bash
ros2 launch turtlebot_catcher catcher.launch.py runner_topic:=/target/pose
```

---

## F. How to Test the Catcher (No Gazebo Required)

### Quick manual test with fake runner

```bash
# Terminal 1 — Source workspace, run catcher with sim_time OFF
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot_catcher catcher.launch.py use_sim_time:=false

# Terminal 2 — Publish a moving fake runner (circles at 2m radius)
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/turtlebot_catcher/test/fake_runner_publisher.py

# Terminal 3 — Watch velocity commands
ros2 topic echo /cmd_vel

# Terminal 4 — Confirm runner topic is arriving
ros2 topic echo /runner/pose
```

**Expected output in Terminal 3:**
```
linear:
  x: 0.28   ← moving forward toward runner
angular:
  z: 0.42   ← turning toward runner
```

### Automated pytest tests (no Gazebo, no robot)

```bash
cd ~/ros2_ws
source install/setup.bash
pytest src/turtlebot_catcher/test/test_catcher_standalone.py -v
```

**Expected:**
```
test_catcher_moves_toward_runner  PASSED
test_catcher_stops_when_close     PASSED
```

### Verify topics exist on the actual robot/sim

```bash
ros2 topic list | grep -E "cmd_vel|odom|scan|runner"
ros2 topic info /runner/pose      # must show publisher count > 0
ros2 topic info /odom
ros2 topic info /scan
```

---

## G. Submission Packaging

```bash
# Create a clean archive for submission
cd ~/ros2_ws/src
tar --exclude='*.pyc' --exclude='__pycache__' \
    -czvf turtlebot_catcher_submission.tar.gz turtlebot_catcher/

# Verify contents
tar -tzvf turtlebot_catcher_submission.tar.gz
```

The judges should be able to build and run it with:
```bash
mkdir -p ~/judge_ws/src
tar -xzvf turtlebot_catcher_submission.tar.gz -C ~/judge_ws/src/
cd ~/judge_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot_catcher
source install/setup.bash
ros2 launch turtlebot_catcher catcher.launch.py
```

---

## Key Parameters — Quick Reference

| Parameter | Default | Description |
|---|---|---|
| `runner_topic` | `/runner/pose` | **Change this if competition uses different topic name** |
| `capture_distance` | `0.5` m | Stop distance — tune based on rules |
| `linear_gain` | `0.6` | Speed = distance × gain (larger = faster) |
| `angular_gain` | `1.5` | Turn rate = error × gain |
| `max_linear_speed` | `0.30` m/s | Hard cap (TB4 Lite max: 0.31 m/s) |
| `obstacle_distance` | `0.40` m | LiDAR avoidance trigger |
| `use_sim_time` | `true` | Set to `false` for real hardware |

---

## Competition Day Checklist

- [ ] Confirm runner topic name with organizers → update `runner_topic` param
- [ ] Confirm `/cmd_vel` namespace (may be `/robot1/cmd_vel` etc.) → update `cmd_vel_topic`
- [ ] Confirm capture distance rules → update `capture_distance`  
- [ ] Do a quick `ros2 topic echo /runner/pose` before start to verify data is flowing
- [ ] Watch `ros2 topic echo /cmd_vel` during first run to confirm catcher responds
- [ ] If robot spins in place: `angular_gain` is too high — reduce to `1.0`
- [ ] If robot overshoots: `linear_gain` is too high — reduce to `0.4`

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Waiting for runner pose...` keeps printing | Runner topic not being published — check `ros2 topic list` |
| Robot spins in circles | Reduce `angular_gain`, check `/odom` is correct |
| Robot drives away from runner | `runner_topic` frame mismatch — ensure pose is in `odom` or `map` frame |
| Captured immediately | `capture_distance` too large, or odom is broken — reduce to `0.3` |
| No `/cmd_vel` output | Node crashed — check `ros2 node info /catcher_node` |
