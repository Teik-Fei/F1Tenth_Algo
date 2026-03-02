# F1TENTH Hackathon Team Effin submission
Team member:       
Wei Xuan    
Kai Wen    
Josiah   
Teik Fei

## Setup

### 1. Clone the Repository

```bash
git clone https://github.com/Teik-Fei/F1Tenth_Algo.git
cd F1Tenth_Algo
```

### 2. Launch the Environment

Start the containers:

```bash
docker-compose up --build
```
> `--build` is required on first run — it compiles the ROS workspace inside the container. Keep this terminal open.

Access the Display:
Open your browser and navigate to http://localhost:8080/vnc.html. Click Connect to view the RViz simulation interface.

### 3. Launch the Simulator

In a new terminal on your host machine, enter the simulator container:

```bash
docker compose exec sim /bin/bash
# Inside the container:
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

---

## Running the Algorithm

With the simulator running and visible in your browser, open a second terminal on your host machine to run the algo:

```bash
docker compose exec sim /bin/bash
# Inside the container:
source /sim_ws/install/setup.bash
ros2 run f1tenth_gym_ros gap_finder_algo
```

---

### Key Components

**Lidar Pre-processing:** The node subscribes to `/scan`. It filters out noisy data and identify large jumps in distance which indicate the edges of obstacles or corners.

**Safety Bubble:** To prevent the car's body from clipping walls, a "Safety Bubble" is projected around the closest detected obstacle. Any LiDAR points within this radius are set to zero, forcing the car to steer clear.

**Max Gap Identification:** The algorithm scans the remaining processed points to find the largest contiguous set of non-zero values.

**Target Steering:** The car computes a range²-weighted mean angle across all beams in the best gap, steering toward the deepest open region rather than the geometric centre.

**Dynamic Velocity:**

- Straightaways: Speed is increased when the steering angle is small and the forward clearance is high.
- Corners: Speed is dynamically reduced based on the sharpness of the required steering angle to maintain stability.

**Three-Factor Adaptive Speed:**

| Factor | Description |
|---|---|
| `speed_clear` | Quadratic brake as the forward wall approaches |
| `speed_steer` | Cosine taper with `CORNER_EXP` beyond `STRAIGHT_THRESH` |
| `speed_side` | Hard speed cap when a side wall is within `SIDE_BRAKE_DIST` |

The **side proximity brake** is a novel addition that prevents wall-scraping when entering tight corners.

---

## Algorithm Development and Thought Processes

The implemented algorithm is based on the **Follow-the-Gap Method**, which we took inspiration from the existing gap finder algorithm from F1TENTH.

### What Was Adopted
- **Safety bubble** around near obstacles to zero out dangerous beams
- **FOV trimming** to restrict the search space to the forward-facing cone
- **Lookahead distance clipping** (`MAX_RANGE`) to reduce jitter from far readings

### Observations During Testing & What We Changed

#### Observation 1 — Car clipped walls at high speed corners
The basic safety bubble used a **fixed beam count** regardless of how close the obstacle was. We observed that at high speeds, the car's body would still clip walls because the bubble wasn't wide enough for nearby obstacles.

**Fix:** We switched to a **distance-adaptive bubble** using geometry:

$$half\_angle = \arctan\left(\frac{R_{safety}}{d}\right)$$

Closer obstacles now receive proportionally wider bubbles, providing stronger protection near the car.

---

#### Observation 2 — Car oscillated between two equally deep gaps
The reference algorithm selects the **single global maximum range beam** as the goal point. During testing, when two gaps of similar depth existed on opposite sides, the car would rapidly switch between them, causing dangerous oscillation.

**Fix 1 — Gap Scoring:** Instead of argmax, we score every valid gap:

$$score = depth \times width \times e^{TURN\_PERSIST \times sign(\psi_{prev}) \times \theta_{gap}}$$

The **turn persistence** term biases the score toward gaps aligned with the current steering direction, so the car commits to a turn rather than second-guessing itself mid-corner.

**Fix 2 — Weighted Heading:** Instead of steering to the geometric midpoint of a gap, we compute a **range²-weighted mean angle**:

$$\theta_{target} = \frac{\sum r_i^2 \cdot \theta_i}{\sum r_i^2}$$

This steers toward the **deepest, most open region** of the gap rather than its centre, which we found produced smoother and safer cornering.

---

#### Observation 3 — Car crashed into side walls when entering tight corners
At high speed, even when the forward path was clear, the car would drift into the outer wall during sharp turns because the speed controller only considered **forward clearance and steering angle**.

**Fix:** We added a **third speed factor** — a side proximity brake:

```
speed = min(speed_clear, speed_steer, speed_side)
```

| Factor | Description |
|---|---|
| `speed_clear` | Quadratic brake as forward wall approaches |
| `speed_steer` | Cosine taper based on steering angle sharpness |
| `speed_side` | Hard speed cap when side wall < `SIDE_BRAKE_DIST` (0.8m) |

The side brake (`SIDE_BRAKE_DIST = 0.8m`, `SIDE_BRAKE_SPEED = 4.0`) was tuned by gradually reducing the distance threshold until wall-scraping was eliminated without sacrificing too much cornering speed.

---

#### Observation 4 — Steering was jerky from scan noise
Raw LiDAR scans contain frame-to-frame noise, causing rapid small changes in the detected gap that translated directly into jittery steering.

**Fix:** A first-order exponential smoother on the steering output:

$$\psi_t = \alpha \cdot \psi_{t-1} + (1 - \alpha) \cdot \psi_{raw}$$

`STEER_SMOOTH = 0.25` was chosen as the gain — lower values were too sluggish to respond to the corners, higher values didn't reduce the jitter enough.

---

### Parameter Tuning Summary

| Parameter | Final Value | Reasoning |
|---|---|---|
| `SAFETY_RADIUS` | 0.35m | Slightly wider than half the car width (0.30m) for safety margin |
| `BUBBLE_THRESHOLD` | 1.5m | Only apply bubbles to obstacles within braking distance |
| `TURN_PERSIST` | 0.30 | Enough to commit to a corner without ignoring genuine new gaps |
| `CORNER_EXP` | 3.0 | Sharp speed drop at corner entry, quick recovery on exit |
| `SIDE_BRAKE_DIST` | 0.8m | Tuned to catch wall proximity before it becomes a crash |
| `STEER_SMOOTH` | 0.25 | Balances noise rejection vs. steering responsiveness |
| `SPEED_MAX` | 55.0 | Maximum speed observed on straights in testing |

