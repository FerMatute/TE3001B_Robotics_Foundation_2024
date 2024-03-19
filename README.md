# TE3001B_Robotics_Foundation_2024
  
Team Dynamic Orchestrators
  - Fernando Josué Matute A00833375
  - Ricardo Navarro Gómez A01708825
  - Sergio Macias Corona A01352038

## <span style="color: rgb(26, 99, 169);">**Introduction**</span>
Working with our partner Manchester Robotics, we create a system that controls the speed of a 6V DC Motor using ROS II and the amplementation of a PID controller. Manchester helped us going through the basics of ROS II and other relevant topics such as MicroRos. We worked hardly five weeks on this project learning the basics of ROS II, working on the design of our controller and working with our ESP32 and DC Motor.

## <span style="color: rgb(26, 99, 169);">**Solution**</span>
A PID controller implemented on our ESP32, which receives signals from ROS II to adjust the control of speed of our DC Motor, working with its max speed, implementing diferent input functions to modify the pwm signal given to the motor.

## <span style="color: rgb(26, 99, 169);">**Software used**</span>
  - ROS II
  <p style="text-align: center;">
  <img src="images/humble" alt="node" style="width:200px;"/>
  <br />
  ROS II Documentation:
  <br />
  Humble Distribution: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
  </p>
  
  - Ubuntu 22.04
  <p style="text-align: center;">
  <img src="images/jammy.jpeg" alt="node" style="width:200px;"/>
  <br />
  Jammy Jellyfish Distribution:
  <br />
  https://releases.ubuntu.com/jammy/
  </p>
  
  
## <span style="color: rgb(26, 99, 169);">**Hardware**</span>
- ESP32
  <p style="text-align: center;">
  <img src="images/esp32.jpeg" alt="node" style="width:200px;"/>
  <br />
  Setting up your ESP32:
  <br />
  https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
  </p>
  
- Debbuging:
  <p style="text-align: center;">
  Failed to connect to ESP32: Timed out waiting for packet header:
  <br />
  https://randomnerdtutorials.com/?s=+Timed+out+waiting+for+packet+header
  </p>
## <span style="color: rgb(26, 99, 169);">**Implementation**</span>
  - Circuit
  <p style="text-align: center;">
  <img src="images/chall3.jpeg" alt="node" style="width:200px;"/>
  <br />
  - Launch node
  <br />
  <img src="images/launch_nodes.png" alt="node" style="width:200px;"/>
  <br />
  - System working
  <br />
  <img src="images/working1.png" alt="node" style="width:200px;"/>
  <br />
  - System Working Signal
  <br />
  <img src="images/working2.png" alt="node" style="width:200px;"/>
  <br />
  - PWM Up
  <br />
  <img src="images/pwm_up.png" alt="node" style="width:200px;"/>
  </p>
