# Contour Context: Abstract Structural Distribution for 3D LiDAR Loop Detection and Metric Pose Estimation(ICRA 2023)
- Note:this the evaluation by using pr curve for Scan context++
- paper code : https://github.com/lewisjiang/contour-context
# Code modification description
- We deleted the CPP files related to reading bag and added some Chinese annotations based on our understanding
- We modified the pr mop.py file and used the correlation parameter to draw the PR curve
# pr result
 - ## KITTI results (num_exclude_frames=300)
|                                              KITTI 00  |    revisit_thres=5m                             |
| -------------------------------------------------------|------------------------------------------------ | 
|![00](https://github.com/user-attachments/assets/8bf7ce9b-c084-4214-9e5c-9d7276451879) |![traj_00](https://github.com/user-attachments/assets/7983cad0-a9de-4de0-8c68-1c9e96297396)|
---------------------------------------------------------------------------------------------------
|                                              KITTI 02  |     revisit_thres=5m                            |
| -------------------------------------------------------|------------------------------------------------ | 
|![kitti_02](https://github.com/user-attachments/assets/7043f03f-8418-4143-abd6-db87b0cfe5d5)|![traj_02](https://github.com/user-attachments/assets/2d2d3c2c-9f5b-48ef-9f22-0a16f32532a4)|

|                                              KITTI 02  |     revisit_thres=10m                           |
| -------------------------------------------------------|------------------------------------------------ | 
|![02_10m](https://github.com/user-attachments/assets/e90eac2a-373a-4602-aac3-510e13714a1d)|![traj_02_10m](https://github.com/user-attachments/assets/6152fad2-2e76-460f-a0f5-acbba3559939)|
------------------------------------------------------------------------------------------------------------
|                                              KITTI 05  |     revisit_thres=5m                            |
| -------------------------------------------------------|------------------------------------------------ | 
|![05](https://github.com/user-attachments/assets/3c3ba09a-4f82-490e-9db9-25066f32d06b)|![traj_05](https://github.com/user-attachments/assets/53d1e8ed-ce10-4427-b49f-8d7a209f1097)|
------------------------------------------------------------------------------------------------------------

|                                              KITTI 08  |     revisit_thres=5m                            |
| -------------------------------------------------------|------------------------------------------------ | 
|![traj_08](https://github.com/user-attachments/assets/7960b6f9-5659-45a3-bf59-8da5dc66d3e9)|![traj_08](https://github.com/user-attachments/assets/ebbcd261-4010-4a03-86a8-0e505cc5765b)|

|                                              KITTI 08  |     revisit_thres=10m                           |
| -------------------------------------------------------|------------------------------------------------ | 
|![08_10m](https://github.com/user-attachments/assets/cb6635a2-0142-4e36-a49f-6acdd66fde0d)|![traj_08_10m](https://github.com/user-attachments/assets/b274cdad-6e37-46cc-9dd1-e0bb3d2a3fef)|
------------------------------------------------------------------------------------------------------------

# Results analysis
Different from the previous methods, this method has detailed depictions of different objects in the location and has excellent stability. This method can handle revisits with lateral translation and rotation relatively well. When a larger revisit distance threshold is set, this method can still maintain excellent performance.
# cite
```
@inproceedings{jiang2023contour,
address     = {London, United Kingdom},
title       = {Contour Context: Abstract Structural Distribution for 3D LiDAR Loop Detection and Metric Pose Estimation},
ISBN        = {9798350323658},
DOI         = {10.1109/ICRA48891.2023.10160337},
booktitle   = {2023 IEEE International Conference on Robotics and Automation (ICRA)},
author      = {Jiang, Binqian and Shen, Shaojie},
year        = {2023},
pages       = {8386–8392}
}

 ```
