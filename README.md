# riptide_core
Riptide Core serves as a package group for all of the packages cor to the functionality of the vehicles. These packages cannot be easily interchanged, and thus are part of the "core" of the riptide software stack.

|            |              |
|------------|--------------|
| OS Distro  | Ubuntu 22.04 |
| ROS Distro | ROS2 Humble  |

## riptide_descriptions
Riptide Descriptions contains a common set of Xacro macros and mesh files for creating transform relationships for the vehicles as well as visualizations when loading the visual models. 

## riptide_hardware
Riptide Hardware contains and starts much of our device level drivers. This package also has the diganostic monitor nodes to make sure the vehicles are healthy during operation. Addtionally this package includes our state estimation system used onboard the vehicle while underway.

## riptide_msgs
Riptide Messages contains all of the custom messaging interfaces used across the vehicle stack. It is used to communicate everythings from hardware statuses to commanding onboard features of the vehicle.
