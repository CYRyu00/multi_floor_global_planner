@echo off

REM The front_laser configuration of Jackal is sufficient for
REM basic gmapping and navigation. It is mostly the default
REM config, but with a SICK LMS100 series LIDAR on the front,
REM pointing forward.

set JACKAL_LASER_3D=1
set JACKAL_LASER_3D_TOPIC="/cloud_in"
@REM set JACKAL_LASER_3D_TOPIC="/velodyne_points"
