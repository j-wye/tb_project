# Turtlebot4 Project
### Initial Settings [🔗](https://blu-y.github.io/turtle/)
```bash
wget https://raw.githubusercontent.com/j-wye/tb_project/autoset.sh
bash autoset.sh
source ~/.bashrc
```

## 1. USAR Contest [🔗](./create_engineering/README.md)

- In an ***Unknown Environment***, Turtlebot has to drive autonomously until it recognizes a set number of *QR codes*.
    - Environment will be shown at the start of the contest
    - Get the exact number of QR Codes right before the contest starts

- During autonomous driving, if *QR codes* is recognized then it should be displayed as a marker on the RVIZ.

- Also should be displayed the robot's path on RVIZ.

- When the robot has recognized a certain number of *QR codes*, it returns to its starting point and stops.

- Measure the final time it took from the time of departure to the time it came to a starting point.

## 2. Detection Project [🔗](./detection/README.md)
Desktop informations and Operation : 
```
Ubuntu 22.04
CPU : i5-13600KF
GPU : RTX 4070 12GB

For communication, use rmw_fastrtps between Desktop and Turtlebot4
```
- Using YOLOv8 for Object Detection
    - Comparison with models [🔗](./detection/README.md)