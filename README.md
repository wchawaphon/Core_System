# Core_System
A Core System for AMR Robot and Manipulator

![Node](<Node.png>)
### Pre_Requirement
- ROS2 Humble
- Colcon build
  ```
  sudo apt install python3-colcon-common-extensions
  ```
- ROS Service
  ```
  sudo apt install ros-humble-std-srvs
  ```
### How to use this Project
Source ROS2 Humble
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
This make you doesn't need to source ROS2 Humble every time you open terminal
1. Clone this Github to the workspace
  ```bash
  git clone https://github.com/wchawaphon/Core_System.git
  cd ~/Core_System
  ```
2. Check Dependencies
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
If it has output like below you can move to the next step
  ```bash
  #Output
  #All required rosdeps installed successfully
  ```
3. Build and Source the packages
  ```bash
  colcon build
  source install/setup.bash
  ```
4. Run the Core System
  ```bash
  ros2 run core_system core_system_node
  ```
If not having errors the output will be
  ```bash
  [INFO] [1741806195.238395674] [core_system_node]: Core system node started, waiting for AMR requests.
  ```
It will wait for the `amr_client.py` to request the station from `core_system_node.py`

5. Open another terminal, Source and Run the AMR Client
  ```bash
  cd ~/Core_System
  source install/setup.bash
  ros2 run core_system amr_client
  ```
Look back at the terminal that run `core_system_node.py`
It will appear a message 
  ```
  [INFO] [1741806462.831809049] [core_system_node]: Received AMR station request.
  Enter multiple station numbers (e.g., 1,2,4) or '0' to stop: 
  ```
The ways you enter the multiple station 
ex. Station 1 -> Station 3 -> Station 4 -> Station 2 
Enter Input as : 1,3,4,2
When you finish entering the station, In the terminal that you Run `amr_client.py` will appear a messages 
  ```
  [INFO] [1741806793.359076063] [amr_client]: AMR assigned 4 stations to complete.
  Enter '1' to confirm AMR reached station 1:
  ```
As you see In this project I was simulate the amr robot approching to the station by user input 

You need to enter 1 to confirm that the AMR Robot was reached the station<br/> 
This messages will appears in the `amr_client.py` terminal after all the station reached
  ```
  Enter '1' to confirm AMR reached station 1: 1
  [INFO] [1741807027.194235919] [amr_client]: Confirming arrival at station 1...
  [INFO] [1741807027.200114365] [amr_client]: Station 1 reach confirmed.
  Enter '1' to confirm AMR reached station 2: 1
  [INFO] [1741807192.171096136] [amr_client]: Confirming arrival at station 2...
  [INFO] [1741807192.177547878] [amr_client]: Station 2 reach confirmed.
  Enter '1' to confirm AMR reached station 3: 1
  [INFO] [1741807192.850246561] [amr_client]: Confirming arrival at station 3...
  [INFO] [1741807192.856130926] [amr_client]: Station 3 reach confirmed.
  Enter '1' to confirm AMR reached station 4: 1
  [INFO] [1741807194.608041109] [amr_client]: Confirming arrival at station 4...
  [INFO] [1741807194.615783263] [amr_client]: Station 4 reach confirmed.
  [INFO] [1741807194.617381988] [amr_client]: Requesting manipulator task.
  [INFO] [1741807194.623494246] [amr_client]: Manipulator task started successfully.
  [INFO] [1741807194.625093236] [amr_client]: All tasks completed. Shutting down AMR client.
  ```
And this messages will appears in the `core_system_node.py` terminal after all the station reached
  ```
  [INFO] [1741807021.545127725] [core_system_node]: Assigned stations [1, 3, 4, 2] to AMR.
  [INFO] [1741807027.197434956] [core_system_node]: AMR confirmed arrival at Station 1.
  [INFO] [1741807192.174056252] [core_system_node]: AMR confirmed arrival at Station 3.
  [INFO] [1741807192.853466901] [core_system_node]: AMR confirmed arrival at Station 4.
  [INFO] [1741807194.611027239] [core_system_node]: AMR confirmed arrival at Station 2.
  [INFO] [1741807194.613128646] [core_system_node]: All stations completed.
  [INFO] [1741807194.620213219] [core_system_node]: Starting manipulator.
  ```
**This project is not finish and not finalized yet**

### Future Develop Plans
- Add an GUI to make it easier to use
- Integrated with AMR and Manipulator 

### Developer
Chawaphon Wachiraniramit 65340500014
