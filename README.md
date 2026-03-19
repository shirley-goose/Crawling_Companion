# Pikabu: A Companion Robot for Playing with Crawling Babies

**Contributor**: Suzy & Shirley

*(Note: Replace `your_photo_name.jpg` with your actual image file name in the asset folder)*

## 📖 Description

**Pikabu** is an interactive, autonomous companion robot built on the TurtleBot3 platform, designed to interact with a crawling baby. It combines 2D Lidar-based navigation with YOLOv8-powered computer vision to create a dynamic social state machine.

The robot autonomously patrols the room while actively avoiding obstacles like furniture and walls. Once it visually detects the baby, it switches from "Wander Mode" to "Social Mode," adjusting its distance and behavior based on how close the baby is. It uses an expressive LED strip and a waving mechanical arm to communicate its current "mood" and intent.

**Core Behaviors:**
Feel free to walk around Pikabu and see its changes!

  * 🟢 **Green (Wandering):** Patrols the area looking for the baby.
  * 🔴 **Red / 🟣 Purple (Obstacle Avoidance):** Intelligently turns away from walls or spins out of dead ends.
  * 🟡 **Yellow (Approaching):** Spots the baby from afar and moves closer while waving.
  * 🌈 **Rainbow (Interacting):** Reaches the perfect social distanc, waves, and spins joyfully.
  * 🔵 **Blue (Escaping):** Backs away to give the baby personal space if they get too close.

## 🎥 Video Demo

[![Pikabu Robot Demo](https://img.youtube.com/vi/0-G2_wm7jlk/maxresdefault.jpg)](https://www.youtube.com/shorts/0-G2_wm7jlk)


## ⚙️ Setup Instructions

### Hardware Requirements

  * **Base:** TurtleBot3 (with OpenCR board)
  * **Compute:** Raspberry Pi (Host) running a ROS 2 Docker Container
  * **Sensors:** 360° Lidar, USB Web Camera
  * **Actuators:** Dual wheel motors, 1x Dynamixel motor (for waving arm)
  * **Feedback:** Neopixel LED Strip (Connected to GPIO D18)

### Prerequisites

1.  Ensure your Raspberry Pi and your development machine are on the same local network.
2.  The YOLOv8 model (`yolov8n.pt`) must be placed in the same directory as the main execution script.
3.  The LED background service (`host_led_service.py`) must be configured on the Raspberry Pi host machine.


## 🚀 Usage Instructions

To bring the robot to life, you will need to open two terminal windows and SSH into the robot.

### Step 1: Launch the Hardware (Terminal 1)

SSH into the physical Raspberry Pi host. This step starts the Lidar, Camera, and Motor controllers.

```bash
# SSH into the robot
ssh robot-3@<your_robot_ip>

# Launch the hardware bringup
ros2 launch turtlebot3_gix_bringup hardware.launch.py
```

If by chance your camera lost connection, you could use
```
source install/setup.bash
```
to reconnect it.

*(Leave this terminal running in the background)*

### Step 2： LED Setup Instructions

The NeoPixel LED strip are controlled directly by the Raspberry Pi host machine.

We have provided a background service setup in the `resource` folder. Please execute the following commands on the **physical Raspberry Pi host**:

1. **Install the required Python library on the host:**
   ```bash
   sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel
   ```

2. **Copy the configuration files to your system:**
   ```
   # 1. Copy the Python script to the home directory
   cp ~/robot3_ws/src/t516_project/crawling_companion/resource/host_led_service.py /home/robot-3/host_led_service.py

   # 2. Copy the background service configuration to the system folder
   sudo cp ~/robot3_ws/src/t516_project/crawling_companion/resource/robot_led.service /etc/systemd/system/
   ```

3. **Activate and start the background service:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable robot_led.service
   sudo systemctl start robot_led.service
   ```

4. **Verify the service is running:**
   ```bash
   sudo systemctl status robot_led.service
   ```
   *(You should see "Active: active (running)" in green).*


### Step 3: Run the Brain inside the Docker Container

Enter your ROS 2 Docker container and execute the main Python script.

```bash
# Assuming you are already attached to or have entered your docker container:
python3 /root/robot3_ws/src/t516_project/crawling_companion/baby/baby.py
```

### 🎉 You're all set\!

The terminal will display `✅ Ultimate Full-Featured Version: Wander + Avoidance + Visual Social Started!` and the robot will immediately light up green and begin exploring the room. Check the terminal output for real-time logs of the robot's state and distance calculations. Debug photos of the baby detection will be automatically saved to the `debug_photos` directory every 5 seconds.