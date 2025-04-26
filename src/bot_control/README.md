**Bot_Control Package**

To launch the filtered_scan node using launch file:
```bash
ros2 launch bot_control filter_scan.launch.py
```

To run the node of filtered lidar sensor readings:
```bash
ros2 run bot_control reading_laser
```
Scan data before filtering:

![Screenshot from 2025-04-26 21-31-16](https://github.com/user-attachments/assets/20bbe62a-09a5-4b34-95ec-edd88ad06db5)


Scan data after filtering:

![Screenshot from 2025-04-26 21-31-26](https://github.com/user-attachments/assets/7034d295-cf62-4673-a2c9-d47319780e53)
