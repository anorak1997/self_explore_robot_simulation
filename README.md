# self_explore_robot_simulation
This is a ros2 complete solution with gazebo nav2 and UI for demonstrating self exploring robot simulation and auto detection of areas and a RRT path demonstrator

Using the UI it can show the solution and the entire thing can be controlled easily. [Screencast from 06-01-2026 03:12:08 AM.webm](https://github.com/user-attachments/assets/eb878bc0-f055-4320-85e7-415b11b0f342)



### Run this command to install required libraries
```
sudo apt update && sudo apt install -y \
ros-humble-tf2-ros \
ros-humble-tf2-geometry-msgs \
ros-humble-nav2-msgs \
ros-humble-nav2-map-server \
ros-humble-nav2-lifecycle-manager \
ros-humble-visualization-msgs \
ros-humble-sensor-msgs-py \
ros-humble-rosidl-default-generators \
ros-humble-turtlebot3 \
ros-humble-turtlebot3-gazebo \
ros-humble-gazebo-ros-pkgs \
python3-pip && \
pip3 install --break-system-packages \
fastapi \
uvicorn \
pydantic \y.
numpy \
pyyaml \
sentence-transformers \
torch \
torchvision \
pillow
```
**Note:** This considers that nav2 and ros2-humble is already installed.

### Step to run the solution
1. In termainl 1 run this command
```
ros2 run semantic_nav web_backend
```  
2. In Terminal 2 run this command
```
ros2 launch bopt_box_filter box_filter.launch.py

<img width="1843" height="1079" alt="image" src="https://github.com/user-attachments/assets/e3758ba1-70e1-4847-b617-9166b225f4bd" />
<img width="397" height="1100" alt="image" src="https://github.com/user-attachments/assets/8df4b1c0-2c1d-4623-b510-c333380287c9" />

### UI explanation
1. The UI is available on http://localhost:8080/. Here first click on Explore + Map button along with semantic button together 
2. Then it will start to map automatically.
3. Then click on Save Map button
4. Then Click on Localize button and it will start the simulation again along with it will show the identified areas and on right  below also a list of avaialble auto detected areas.
<img width="389" height="428" alt="image" src="https://github.com/user-attachments/assets/cc5c3127-7380-4f45-82fa-5eaccb1014a9" />


5. Use the go button to go to it or casually talk with it to tell it to go. 
```
