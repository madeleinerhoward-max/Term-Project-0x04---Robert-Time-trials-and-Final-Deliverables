Calibration and Testing Procedure
=================================

Calibration
------------

Robert uses an automatic calibration procedure for the line sensor to ensure accurate detection of both the black line and the surrounding surface. This calibration is initiated using the user button.

On the first button press, the robot enters calibration mode. During this process, the line sensor must be manually swept across the black line and the white background. This allows the sensor to record the minimum and maximum reflectance values for each channel. By capturing both extremes, the system can normalize sensor readings during operation, improving the accuracy of line detection.

This calibration step is critical because lighting conditions and surface variations can significantly affect raw sensor readings. Performing calibration before each run ensures consistent performance across trials.

---

Testing Procedure
------------------

Testing was performed incrementally, focusing on individual sections of the course rather than attempting the full track immediately. This approach allowed us to isolate problems and improve specific behaviors before integrating the entire system.

The general testing process followed these steps:

- Begin with straight-line tracking (checkpoint 0 → 1)
- Tune control gains (such as Kp) for stable line following
- Test checkpoint detection using encoder counts
- Validate the garage sequence independently
- Tune turning and alignment within the garage
- Test the zig-zag section at reduced speed
- Integrate bumper-triggered cup sequence
- Combine all sections into full-course runs

Each section was tested repeatedly until Robert demonstrated consistent and reliable performance. Once a section was considered “mastered,” we moved on to the next portion of the course.

---

Tuning and Iteration
---------------------

Throughout testing, adjustments were made to improve performance, including:

- Line-following gains (Kp and speed)
- Sensor thresholds and calibration quality
- Encoder count targets for checkpoints
- Timing and distances for FSM transitions

Testing was an iterative process, where changes were made based on observed behavior and then validated through repeated trials.

---

Summary
--------

The combination of automatic sensor calibration and incremental testing allowed us to systematically improve Robert’s performance. While the robot did not complete the full course, this structured approach enabled consistent success in individual sections and helped identify the key limitations of the system.
