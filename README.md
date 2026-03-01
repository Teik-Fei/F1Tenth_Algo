# F1TENTH Hackathon Team Effin submission
Team member:       
Wei Xuan    
Kai Wen    
Joisah   
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
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
# Inside the container:
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
> If you get `No such container`, run `docker ps` to find the exact container name and replace `f1tenth_gym_ros-sim-1` with it.

---

## Running the Algorithm

With the simulator running and visible in your browser, open a second terminal on your host machine to run the algo:

```bash
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
# Inside the container:
source /sim_ws/install/setup.bash
ros2 run f1tenth_gym_ros gap_finder_algo
```

---

## Algorithm thought process

The implemented algorithm is based on the Follow-the-Gap method, which allows the vehicle to navigate complex tracks by finding the "deepest" available space in its LiDAR scans.

### Key Components

**Lidar Pre-processing:** The node subscribes to `/scan`. It filters out noisy data and identify large jumps in distance which indicate the edges of obstacles or corners.

**Safety Bubble:** To prevent the car's body from clipping walls, a "Safety Bubble" is projected around the closest detected obstacle. Any LiDAR points within this radius are set to zero, forcing the car to steer clear.

**Max Gap Identification:** The algorithm scans the remaining processed points to find the largest contiguous set of non-zero values.

**Target Steering:** The car identifies the center of this gap as its goal. It calculates the steering angle required to head toward that center point.

**Dynamic Velocity:**

- Straightaways: Speed is increased when the steering angle is small and the forward clearance is high.
- Corners: Speed is dynamically reduced based on the sharpness of the required steering angle to maintain stability.

---

## Configuration Parameters

The behavior can be tuned in `gap_finder_algo.py`:

| Parameter | Description |
|---|---|
| `SPEED_MAX` | Maximum velocity on straights |
| `FWRD_CLEAR_DEG` | The field of view used to calculate forward clearance |
| `STRAIGHT_THRESH` | steering angle for to determine the speed 
