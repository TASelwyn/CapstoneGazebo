# ARISE Lab Simulation

Gazebo simulation of the ARISE drone cage (Carleton) with the lab's two ground robots, a **Clearpath Jackal** and a **Waveshare Wave Rover**, positioned by a **simulated OptiTrack**.

**Versions:** ROS 2 Jazzy + Gazebo Harmonic.

There are two ways to run the sim:
- **Native Linux** (Ubuntu 24.04): fastest, and Gazebo opens as a normal window. [Jump there.](#native-linux-ubuntu-2404)
- **Docker**: Gazebo shows in your web browser. [Jump there.](#docker-windows-mac-linux)

---

## Contents

```
src/
  lab_controllers/        Runs on BOTH sim and real robots
    config/robots.yaml      shared robot settings: IDs, topics, speeds, follower
    follow                  starts target_follower with robots.yaml settings
    teleop                  WASD driving window
    target_follower         the lab's follower script
  arise_sim/              Simulation only (never goes on a robot)
    config/sim.yaml         spawn positions, simulated OptiTrack frame/noise
    launch/sim.launch.py    starts Gazebo, robots, bridge, simulated OptiTrack
    worlds/, models/        the cage world (floor, walls, logos, QR panels, obstacles)
    urdf/                   Jackal and Wave Rover models
    reset                   puts sim robots back at their start positions
  external/               OptiTrack message definitions (copied from TU Delft's driver)
lab_env.sh                Native sim: run "source lab_env.sh" in each new terminal
scripts/setup_native.sh   Native sim: one-time install + build (Ubuntu 24.04)
compose.yaml, Dockerfile  Docker setup
```

---

## Commands

| What | Sim (native) | Sim (Docker) | Real robots |
|---|---|---|---|
| Start | `ros2 launch arise_sim sim.launch.py` | `docker compose up` | (turn the robots on) |
| Drive (WASD) | `ros2 run lab_controllers teleop` | `docker compose exec sim teleop` | `ros2 run lab_controllers teleop` |
| Follow | `ros2 run lab_controllers follow` | `docker compose exec sim follow` | `ros2 run lab_controllers follow` |
| Drive randomly | `ros2 run lab_controllers wander` | `docker compose exec sim wander` | `ros2 run lab_controllers wander` |
| Back to start | `ros2 run arise_sim reset` | `docker compose exec sim reset` | (by hand!) |

Each command goes in its **own terminal**. The sim starts with nothing moving. Native sim users run `source lab_env.sh` first in every terminal; for real robots see [Using the real robots](#using-the-real-robots).

### Driving (teleop)

`teleop` opens a small window. Click it, then **hold** keys:

| Keys | Robot does |
|---|---|
| **W** / **S** | forward / back |
| **A** / **D** | turn left / right |
| **Space** | stop |

- By default it drives the robot that **isn't** the follower. Add a name to choose: `teleop jackal`.
- Speeds come from the `teleop` section of `robots.yaml`. Override them for one session with `--ros-args -p linear_speed:=0.1`.
- Closing the window or pressing Ctrl+C stops the robot.
- When no key is held, the window sends nothing, so it never fights the follower.

Neither robot can slide sideways (both are skid-steer), so A and D turn instead of strafing.

In Docker, the window appears inside the browser view, next to Gazebo.

### Following

`follow` starts `target_follower.py` with the settings from `robots.yaml`. The follower is the Jackal by default, or add a name: `follow wave_rover`. **Ctrl+C stops the script and sends the robot a stop command.**

### Resetting (sim only)

`reset` puts both sim robots back at their start positions (from `sim.yaml`) without restarting; `reset jackal` does just one. If the follower is running, it starts following again right away.

### Sim launch options

| Option | Default | What it does |
|---|---|---|
| `gui:=false` | `true` | Run Gazebo without its window (lighter; topics still work) |
| `obstacles:=true` | `false` | Add the original drone-world boxes and landmarks |
| `sim_config:=/path/file.yaml` | `arise_sim/config/sim.yaml` | Use a different sim settings file |

---

## Settings

Two files, split by where they apply. Edits take effect the next time you start a command, with no rebuild needed.

**`src/lab_controllers/config/robots.yaml`** is **shared by sim and real robots**:
- Each robot's Motive Streaming ID, `cmd_vel` topic, and `Twist` vs `TwistStamped`
- Teleop speeds
- Which robot follows, and extra `target_follower` parameters

The sim creates its robots with these exact topics, so whatever works in the sim works on the real robots unchanged. To use a different copy of the file, e.g. one per machine, run `export LAB_ROBOTS_CONFIG=/path/to/robots.yaml`.

| | Jackal | Wave Rover |
|---|---|---|
| Rigid-body ID | 3 | 2 |
| Command topic | `/j100_0000/cmd_vel` (**placeholder**) | `/wave_rover/cmd_vel` (**placeholder**) |
| Message type | `TwistStamped` (Clearpath on Jazzy) | `Twist` |

**The two topics are placeholders.** Replace them with the real robots' topics (`ros2 topic list` on each robot) before using the real robots.

**`src/arise_sim/config/sim.yaml`** is **sim only**: start positions, and how the simulated OptiTrack behaves (rate, noise, coordinate frame).

---

## Using the real robots

The real-robot commands are the same as the sim's: `lab_controllers follow` and `teleop`, reading the same `robots.yaml`.

1. **Fill in the real topics** in `robots.yaml` (see above).
2. **Use a terminal that has NOT run `source lab_env.sh`.** That file deliberately blocks network traffic, which would stop you reaching the robots. Instead run:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=<the robots' domain ID>
   ros2 topic list          # check you can see the robots' topics
   ```
3. **Teleop:** run it from a laptop on the lab network, not over SSH. The window needs a screen, and an SSH terminal can't report held key combinations. The laptop publishes straight to the robot over ROS 2.
4. **Follow:** run it on the robot (e.g. over SSH on the Wave Rover's Pi, as the lab did) or from the laptop. Both work, since it only needs the OptiTrack topic and the robot's `cmd_vel`.

**Start slow** (`--ros-args -p linear_speed:=0.1` for teleop, `v_max` in `follower_params`) and keep the e-stop within reach.

**Wi-Fi drops:** if the connection drops mid-command, a robot only stops if its own driver has a command timeout. Check that both robots stop by themselves when commands stop arriving, before relying on this.

---

## Native Linux (Ubuntu 24.04)

This is the recommended way to run the sim on a Linux laptop. Gazebo opens as a normal window, with no browser or VNC involved, and it uses your graphics card directly. Integrated graphics are fine for this world.

**One-time setup** (installs ROS 2 Jazzy if needed, plus Gazebo Harmonic, then builds; takes a while the first time):

```bash
cd lab_sim
./scripts/setup_native.sh
```

**Every new sim terminal:**

```bash
cd lab_sim
source lab_env.sh
```

Only **new** files need a rebuild: `colcon build --symlink-install`.

**On Windows**, you can get the same native experience through WSL2. Install "Ubuntu 24.04" from the Microsoft Store, then follow these steps inside it. Windows 11 shows Linux windows directly.

---

## Docker (Windows, Mac, Linux)

You need **Docker Desktop** (Windows/Mac) or **Docker Engine** (Linux).

```bash
cd lab_sim
docker compose up --build
```

The first run downloads several GB and takes 10–20 minutes. After that, plain `docker compose up` starts in under a minute. When the terminal shows `Open http://localhost:6080/...`, open:

**http://localhost:6080/vnc.html?autoconnect=1&resize=remote**

You'll see the cage, the **yellow Jackal** and the **blue Wave Rover**. The small dark strip on each robot marks its front. To stop, press **Ctrl+C**.

Other handy commands:

```bash
docker compose exec sim ros-env ros2 topic echo /mocap_rigid_bodies --once
docker compose run --rm --service-ports sim ros2 launch arise_sim sim.launch.py obstacles:=true
```

Edits to files in this folder are picked up on the next start. Run `--build` again only after updating to a new version of this repo.

**Linux users who prefer Docker but want a normal window:**

```bash
xhost +local:docker
docker compose -f compose.yaml -f compose.x11.yaml up --build
```

---

## Matching the real lab

The sim is only useful if it behaves like the lab. These are the things to confirm on the real setup.

### 1. Topics and IDs
In `robots.yaml`, replace the placeholder `cmd_vel_topic`s with the real ones. Check each robot's **Streaming ID** in Motive and set `rigid_body_id` (defaults: Jackal 3, Wave Rover 2, from `target_follower.py`).

### 2. The `angular_sign: -1.0` mystery (coordinate frame)
The lab's `target_follower.py` setup uses `angular_sign: -1.0`, which flips every turn command. Under normal ROS conventions this should be `+1.0`. The most likely reason is that the OptiTrack frame is **mirrored** relative to ROS conventions.

So the defaults reproduce that:
- `robots.yaml` has `angular_sign: -1.0`, as in the lab.
- `sim.yaml` has `mirror_y: true`, so the simulated OptiTrack is mirrored the same way.

In testing, this combination follows correctly, while `-1.0` without the mirror spins the wrong way.

**To confirm, in the lab:**

1. With the OptiTrack driver running, run `ros2 topic echo /mocap_rigid_bodies` and watch one robot.
2. **Rotate the robot counter-clockwise by hand, as seen from above.** Compute yaw from the quaternion the same way the script does: `atan2(2(wz+xy), 1-2(y²+z²))`.
   - If yaw **decreases**, the frame is mirrored, and the defaults are right. Nothing to change.
   - If yaw **increases**, the frame is normal, and the `-1.0` must come from the robot's own driver turning backwards. The sim can't reproduce that yet, so report it and it can be added.
3. **Push the robot forward.** Of `x`, `y` and `z`, the one that stays constant is the up axis. If it's `y`, Motive streams Y-up; set `up_axis: y` in `sim.yaml`. (`target_follower.py` assumes Z-up, though, since it uses `x` and `y` as floor position.)

The `origin` / `origin_yaw` settings in `sim.yaml` let you move the OptiTrack origin, if it isn't at the cage centre.

### 3. Wave Rover dimensions
Waveshare doesn't publish the wheelbase or track (the distance between left and right wheels). Both are estimates in `src/arise_sim/urdf/wave_rover.urdf.xacro`. Measure the real robot and update them if its turning in the sim doesn't match.

---

## About `target_follower.py`

`src/lab_controllers/lab_controllers/target_follower.py` is the lab script with **one addition**: a `cmd_vel_stamped` parameter. When `false` (the default), it behaves exactly like the original. When `true`, it publishes `TwistStamped`, which a **Clearpath Jackal needs on ROS 2 Jazzy**. `follow` sets it automatically from `robots.yaml`.

Notes from testing (the script itself is unchanged):

- **On Ctrl+C, the script's own final "stop" command is never sent.** ROS has already shut down when it tries, and you'll see a `publisher's context is invalid` error. `follow` sends the stop for it, so **use `follow` rather than running `target_follower` directly**, on real robots too.
- `k_theta` is `0.01`, so heading corrections while driving are very weak. Turning mostly happens at the fixed `w_min` speed.

---

## Safety

The simulated OptiTrack publishes on **`/mocap_rigid_bodies`**, and the sim robots use the **real robots' topic names**. If a sim and a real robot ever share a network, they'd mix: real robots could react to fake positions, or sim commands could drive real robots.

- **Docker:** already isolated. ROS traffic stays inside the container.
- **Native sim:** `source lab_env.sh` sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, which keeps ROS traffic on your computer. **Always use it for the sim**, and never in a terminal meant for real robots.

---

## How it works

```
Gazebo (cage + robots)
  ├─ DiffDrive plugin          ← <robot's cmd_vel topic>  (via ros_gz_bridge)
  └─ OdometryPublisher plugin  → /sim/<robot>/ground_truth (true pose)
                                      ↓
                              mocap_sim node (mirrors/moves the frame per sim.yaml)
                                      ↓
                          /mocap_rigid_bodies  (RigidBodyArray, like the real driver)
                                      ↓
                    target_follower (via "follow") → <robot's cmd_vel topic>
```

The world was adapted from the drone lab's `arise.world`, with these changes:

- The drone is removed.
- Gravity is restored. The original had it off, which only suits a drone.
- The QR panels are fixed in place, so they no longer fall over.
- The floor and walls use simple collision boxes, which work better for wheels than the thin cage mesh.
- There are no hard-coded paths.

---

## Troubleshooting

- **Docker: browser page is black or won't connect.** Wait until the terminal shows the `Open http://localhost:6080` line, then refresh. Gazebo can take 30+ s to appear the first time.
- **Teleop window doesn't react.** Click inside it first, since it only reads keys while it has focus.
- **`package 'arise_sim' not found` (native).** You forgot `source lab_env.sh` in that terminal.
- **teleop/follow do nothing in the sim.** That terminal probably isn't set up the same as the sim's. Use `source lab_env.sh` in both.
- **Docker: `port 6080 is already allocated`.** Another copy is running. Run `docker compose down`, then try again.
- **Windows: `bad interpreter` or `\r` errors.** Your clone converted line endings. Run `git config core.autocrlf false`, delete the folder and clone again.

---

## Status

**Tested:**
- All packages build on ROS 2 Jazzy.
- The sim launch runs end-to-end (Gazebo replaced by a simple stand-in), using the shared settings.
- `follow` with the lab's `angular_sign -1.0` and the mirrored simulated OptiTrack drives straight to the target and turns correctly while chasing. Ctrl+C stops it and the robot.
- The teleop window was tested with simulated key presses, including held combinations, overrides and window close/Ctrl+C.
- `reset` sends the correct request.
- The OptiTrack message format matches the real driver.
- The browser display stack works, with software OpenGL 4.5.