# Localization using Fast LIO (For UROP path planning version)


## 1. Map Management

save PCD map in /map folder

map path is saved in /config/snu_localization.yaml

floor_1, floor_2, floor_3 PCD is saved

## 2. How to use

Command

    roslaunch snu_localization localization.launch

Use with Fast-LIO

If you start, this message will be showed. 

"Enter x, y, z (or press enter to skip, 'R' to reset): " 


If you know approximate initial pose, you can insert x,y,z to set intial pose. Localization will be start based on that pose. 

If you feel localization failed, you can insert 'R' to reset localization.

If you want localization pose to be rotated, you can insert  'F' to rotate pointcloud.



## 3. Floor change

Floor is changed by topic /multi_floor_planner/current_floor

if you want to change floor by rostopic, command this message

    rostopic pub /multi_floor_planner/current_floor std_msgs/Int32 "data: 2"


The number means floor.

## 4. Robot pose rostopic

This code is for UROP Path planner, UROP path planner also publish /multi_floor_planner/estimate_pose

if you want to change pose by rostopic pub command in terminal, command this message.

    rostopic pub /multi_floor_planner/estimate_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'map'
pose:
  position: {x: 1.0, y: 2.0, z: 3.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
    



## 4. Parameters

in the /config/snu_colorization.yaml, there are 4 parameters that you can modify.


    NDT_voxel_size: 0.5
    scan_voxel_size: 0.3
    map_voxel_size: 0.3
    map_entire_voxel_size: 0.2


NDT_voxel_size are used in global localization using ndt, which requires fast convergence. Too big voxel size gives fast but incorrect, too small gives accuracy but slow time. 0.5~1.0 are sufficient.

scan and map voxel size are used in GICP, which requires accuracy. 0.3~0.5 are sufficient.

map_entire_voxel_size is only for visualization in rviz.


