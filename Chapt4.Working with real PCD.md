### voxel_grid filtering
Voxel grid filtering will create a cubic grid and will filter the cloud by only leaving a single point per voxel cube, so the larger the cube length the lower the resolution of the point cloud.

http://pointclouds.org/documentation/tutorials/voxel_grid.php

### Region of Interest
A boxed region is defined and any points outside that box are removed.
http://docs.pointclouds.org/trunk/classpcl_1_1_crop_box.html

there is a 
```
setInputCloud
```
function here not listed in the doc above
