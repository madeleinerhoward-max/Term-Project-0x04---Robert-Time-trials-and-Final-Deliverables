Mechanical Design
=================

The mechanical design of Robert is based on a white Romi Chassis Kit, which includes the motors, encoders, wheels, and caster from Pololu Robotics and Electronics. This chassis provided a compact and reliable platform for differential drive motion and was used as the base structure for the entire robot. More information about the chassis can be found here:

https://www.pololu.com/category/203/romi-chassis-kits

For the hardware and electronics integration, we used a Nucleo L476RG microcontroller, a modified Shoe of Brian, and a BNO055 IMU breakout board. The structure of the robot was assembled using M2.5 hardware, including standoffs in 8 mm, 10 mm, and 30 mm lengths, along with M2.5 socket head cap screws, nylon lock nuts, and nylon washers. Screw lengths of 6 mm, 8 mm, and 10 mm were used throughout the assembly. Electrical connections were made using female-to-female jumper wires for flexibility and ease of modification during testing.

The electronics are mounted on a stacked platform above the chassis, which helps keep wiring compact while maintaining accessibility for debugging. This layered structure also improves organization and prevents interference with the wheels and sensors. The robot is powered by six rechargeable batteries, providing sufficient voltage and current for motors and control electronics.

For Robert-specific components, we used a Pololu QTR reflectance analog sensor with five channels for line detection:

https://www.pololu.com/category/123/pololu-qtr-reflectance-sensors

We also used a Romi T1-RSLK MAX bumper switch kit for obstacle detection and interaction:

https://www.pololu.com/product/3678

To properly position the line sensor, we designed a custom mount that attaches to the front of the chassis. This mount ensures that the sensor remains at a consistent height above the ground, which is critical for reliable line detection. The geometry of the mount also allows the sensor array to extend slightly forward of the robot, improving its ability to detect turns and transitions in the track.

The mount was designed in CAD and then 3D printed using black PLA on a Bambu A1 Mini printer, courtesy of the Cal Poly Student Machine Shops and Mustang Makerspace. The use of a custom mount allowed us to optimize sensor placement beyond the standard kit configuration and contributed significantly to improved tracking performance.

Sensor Mount Design
--------------------

.. image:: _static/sensor_iso.png
   :width: 500px
   :align: center

.. image:: _static/sensor_top.png
   :width: 500px
   :align: center


The CAD model and drawing of the sensor mount are included to allow the design to be replicated. The mount is simple, lightweight, and rigid, ensuring minimal vibration while maintaining consistent sensor alignment with the track.

Files
------

- CAD part file (STL/STEP)
- Engineering drawing (PDF)

These files are included in the repository so that the design can be reproduced or modified if needed.

---

Summary
--------

Overall, the mechanical design of Robert combines a standard chassis platform with custom components to improve performance and adaptability. The integration of a 3D-printed sensor mount and structured hardware layout resulted in a stable and effective system for line-following and obstacle interaction.