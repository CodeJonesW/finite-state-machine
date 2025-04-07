# Finite State Machine Demo

## What is a Finite State Machine (FSM)?

An FSM is a computational model used to represent a system that transitions between finite states based on inputs. It's widely used in robotics, game development, and AI decision-making.

## How FSMs Work in Your ROS 2 Project

Your ROS 2 system follows an FSM model where different nodes transition between states based on sensor inputs:

- Sensor Simulator → Detects obstacles (True or False)

- Planner FSM → Decides whether to STOP or MOVE

- Actuator Logger → Logs actions based on the planner’s decision

- Brake Controller → Applies or releases brakes

👉 In this case, the FSM states could be:

- MOVING

- STOPPED

- SLOWING DOWN (optional)

The planner node is where the FSM logic happens—it listens to sensor data and decides what to do next.

## 🔥 Why FSMs Are Useful

✅ Predictable → Clear transitions between states
✅ Efficient → Only one state is active at a time
✅ Scalable → Easy to add more states (e.g., SLOWING DOWN)

## Steps to Build, Run Container and install dependences

1. docker build -t ros2_fsm_demo -f docker/Dockerfile .
2. docker run -it --rm ros2_fsm_demo bash
3. pip install -e /ros2_ws/src/fsm_demo
4. source /ros2_ws/install/setup.bash
5. ros2 pkg list | grep fsm_demo
6. ros2 run fsm_demo sensor_simulator

## Enter container already running
- docker exec -it ros2_fsm_demo_dev bash

## 🚀 Running All ROS 2 Nodes Together in Docker

Now that your ROS 2 package is working, let’s **run all the nodes** so you can see the full interaction between the **sensor, planner, actuator, and brake controller**.

---

### ✅ 1️⃣ Open Multiple Terminal Tabs (or Use `docker exec`)

Since you’re running inside **Docker**, you’ll need to **open multiple terminals** or use **`docker exec`** to run different nodes.

---

### 🟢 **Terminal 1: Start the Sensor Simulator**

In your first terminal (already running the container), run:

```bash
ros2 run fsm_demo sensor_simulator
[INFO] [sensor_simulator]: Published obstacle: True
[INFO] [sensor_simulator]: Published obstacle: False
```

### 🟢 Terminal 2: Start the Planner FSM

Open a new local (host machine) terminal, then attach to the running Docker container:

```bash
docker exec -it $(docker ps -qf "ancestor=ros2_fsm_demo") bash
source /ros2_ws/install/setup.bash
```

Now, run the planner node:

```bash
ros2 run fsm_demo planner_fsm
[INFO] [planner_fsm]: State: MOVING | Decision: MOVE
[INFO] [planner_fsm]: State: STOPPED | Decision: STOP
```

### 🟢 Terminal 3: Start the Actuator Logger

Open another local (host machine) terminal, then attach to the Docker container:

```bash
docker exec -it $(docker ps -qf "ancestor=ros2_fsm_demo") bash
source /ros2_ws/install/setup.bash
```

Now, run the actuator logger:

```bash
ros2 run fsm_demo actuator_logger
[INFO] [actuator_logger]: Action taken: MOVE
[INFO] [actuator_logger]: Action taken: STOP
```

### 🟢 Terminal 4: Start the Brake Controller

Open one more local (host machine) terminal, then attach to the Docker container:

```bash
docker exec -it $(docker ps -qf "ancestor=ros2_fsm_demo") bash
source /ros2_ws/install/setup.bash
```

Now, run the brake controller:

```bash
ros2 run fsm_demo brake_controller
[INFO] [brake_controller]: Applying brakes!
[INFO] [brake_controller]: Releasing brakes.
```

### 🎯 Now You Can See the Full Interaction!

Sensor detects obstacles → Publishes /obstacle

Planner FSM reads /obstacle → Decides MOVE or STOP

Actuator Logger listens to /decision → Logs actions

Brake Controller reacts → Applies or releases brakes

## Helpful Commands

### List Docker Containers

- docker ps

### Clean Docker Containers

- docker ps -a # List all running/stopped containers
- docker stop $(docker ps -aq) # Stop all containers
- docker rm $(docker ps -aq) # Remove all stopped containers
- docker rmi ros2_fsm_demo # Remove the built image
