# To Pallet & Beyond

Detect Pallets in a warehouse or workshop environment

## Rough Demo
https://github.com/user-attachments/assets/018108a8-dd86-4a20-8809-c0c31a2205c7
## Installation

assuming that ```/opt/ros/<distro>/setup.bash``` is sourced

### 1. make python based ros package

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
ros2 pkg create --build-type ament_python to_pallete_and_beyond
```

### 2. clone grounding dino inside ros2_ws

clone this repository outiside of ros2_ws
```bash
git clone https://github.com/IDEA-Research/GroundingDINO.git
```
TO DO: clone repo inside ros2_ws

### 3. clone this repo

```bash
cd ros2_ws/src
git clone https://github.com/cruxbots/to_pallets_and_beyond.git
```

#### 4. create virtual environment

```bash
python3 -m venv .ven
pip3 install -r requirements.txt
```

follow this steps to [install groundingDINO](https://github.com/IDEA-Research/GroundingDINO?tab=readme-ov-file#hammer_and_wrench-install)

```bash
source .venv/bin/activate
cd /path/to/groundingDINO
pip install -e .
```

#### 5. colcon build

```bash
cd ros2_ws
colcon build --symlink-install
```

#### Trooubleshooting

TO DO: fix all of this

1. if you are getting some error regarding ultralytics not being installed try rosdep or install it globally

2. Due to poor memory management, there are some some memory leakage in notebook codes so may need to restart the system.

## How to Use

#### 1. source ROS

```bash
cd /path/to/ros/package
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

#### 2. Run using simulated camera


```bash
ros2 run to_pallete_and_beyond simulated_camera
```

Run pallete detect
open a new terminal window and repeat step 1 and then run:

```bash
ros2 run to_pallete_and_beyond pallete_detect
```

Run Pallete segment
open a new terminal window and repeat step 1 and then run:

```bash
ros2 run to_pallete_and_beyond pallete_segment
```
