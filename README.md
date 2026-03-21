# Term-Project-0x04 — Robert Time Trials and Final Deliverables

This project presents the design, implementation, and testing of an autonomous line-following robot developed as part of a mechatronics course. The goal of the robot is to reliably and efficiently follow a predefined track using onboard sensors, control algorithms, and embedded programming.

---

## 📺 Video Demonstrations

Watch our robot in action through our full playlist of trial runs and testing:

👉 **YouTube Playlist:**  
https://youtube.com/playlist?list=PLUa6QExzi7YWwO5w1iTEE_n-R9CKloOMO&si=530ov6_uxC9BOj0y

---

## 🌐 Project Website (Sphinx Documentation)

Our full project documentation, including design details, code explanations, diagrams, and results, is available through our Sphinx-generated site.

### To view locally:

1. Download the build and source file
2. In your terminal type "make html"
3. Then type "build/html/index.html" to view the website.

### Website Tour Link
https://youtu.be/J_zPQydg4FU

---

### Code Structure Overview
1. main.py - This file initializes all of the hardware components, including the motors, encoders, line sensors, IMU, and bumpers. The file also creates the shared variables, defines all of the primary system tasks, runs the cooperative multitasking scheduler, coordinates the line following and course execution, user input, and state estimation.
2. task_user.py - This file implements the user interface task for serial communication, which allows for real-time interaction with Robert, including adjusting control parameters, calibrations, and viewing diagnostic data.
3. task_motor.py - This file contains the FSM for closed-loop motor control using encoder feedback and PI control.
4. task_imu.py - This file provides the interfacing for the IMU sensor on Robert.
5. task_observer.py - This file implements an observer-based estimation of robot position and velocity using encoder and IMU sensor data. This file was mainly used for testing and isn't directly used in the final integrated control loop.
6. task_share.py and cotask.py - These files are both pre-written given files that control the shared variable and queue classes used for communication between tasks, and also run the scheduler of the tasks.
7. motor_driver.py - This file provides the driver class for controlling Robert's DC motors using PWM signals.
8. encoder.py - This file implements an encoder interface for measuring wheel position and velocity.
9. line_sensor.py - This file handles reading and processing data from our reflectance sensor array. It includes calibration and centroid calculation for line-following control.
10. bno055.py - This file is the driver for Robert's BNO055 IMU sensor. The file provides access to orientation and heading data.
