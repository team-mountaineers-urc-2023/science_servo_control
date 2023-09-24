# science_servo_control
ROS package responsible for controlling actuators on Wanderer's science payload
## Description
This package defines services for controlling the [Actuonix linear actuators](https://www.actuonix.com/l16-140-35-6-r), [DF Robot peristaltic pumps](https://www.dfrobot.com/product-1698.html), and [Dynamixel XL-430 servos](https://www.robotis.us/dynamixel-xl430-w250-t/) used on Wanderer's science payload. It uses the `payload_can` package to control the pumps and uses the `dynamixel_controller` package to configure and control the Dynamixels.