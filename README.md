# Core_System
### Pre_Requirement
- ROS2 Humble

### How to use this Project
1. Clone this Github to the workspace
```bash
git clone https://github.com/wchawaphon/Core_System.git
cd ~/Core_System
```
2. Build and Source the packages
```bash
colcon build
source install/setup.bash
```
3. Run the Core System
```bash
ros2 run core_system core_system_node
```
4. Open another terminal, Source and Run the AMR Client
```bash
cd ~/<your_workspace>
source install/setup.bash
ros2 run core_system amr_client
```
