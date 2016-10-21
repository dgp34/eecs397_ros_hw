# p6_block_info

This package addresses P6: Given a bagfile of a LIDAR scanning a block, modify a provided example "lidar_transformer" node in order to find the block's dimensions as well as its centroid with respect to the world frame.

## How to Run
Prior to running any of this program, make sure to set the use_sim_time as true.  Then, continue to starting Rviz.  The bagfile will then be have to run once or twice in order to get the fixed frame set to "world," the LaserScan topic field set to "/scan," and find the best visualization options for displayed points.
After this process has been carried out, run the transformer node manually by using 'rosrun p6_block_info find_block_info', followed immediately by running the bagfile once more.  The results of the file will appear in Rviz, while the continuously updated dimensions and centroid will appear in the terminal until the bagfile concludes.  The final values shown in the terminal can be treated as the most accurate estimate provided by that run of the program; running it again may potentially give values that are minutely different, but different nonetheless.

At last run prior to commit and submission, the values are as follows:
Dimensions (x-y-z):  0.405665 by 0.665641 by 0.179758.
Centroid (x,y,z) = (-0.007168, -0.332820, 0.089879).
