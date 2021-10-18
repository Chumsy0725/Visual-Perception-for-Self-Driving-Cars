# Visual-Perception-for-Self-Driving-Cars

## Baseline Enviroment Perception 

In Baseline Enviroment Perception, a set of Images, Depth Maps, outputs of a semantic segmentation model and object detection model gathered in the CARLA simulator enviroment is used to,
- Estimate drivable estimation in 3D using the output of a semantic segmentation model
- Estimate lanes using the output of a semantic segmentation model
- Filter out the errors in the output of high recall, low precision object detection model using semantic segmentation output
- Finally, to estimate the distance to obstacles using the filtered 2D object detection outputs.

<p align="center">
  <img width="1600" height="600" src="Baseline Enviroment Perception/Plots/data.png">
</p>

The output segmentation image contains mapping indices from every pixel to a road scene category as follows

|Category |Mapping Index| Visualization Color|
| --- | --- | --- |
| Background | 0 | Black |
| Buildings | 1 | Red |
| Pedestrians | 4 | Teal |
| Poles | 5 | White |
| Lane Markings | 6| Purple |
| Roads | 7 | Blue |
| Side Walks| 8 | Yellow |
| Vehicles| 10 | Green |

<p align="center">
  <img width="1200" height="300" src="Baseline Enviroment Perception/Plots/SG.png">
</p>

#### Drivable Space Estimation

Firstmost x,y,z coordinated of every pixel is estimated using the depth maps and the intrinsic parameters found in the camera calibration matrix. 

<p align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=\bg_black&space;\large&space;z&space;=&space;depth&space;\newline&space;\newline&space;x&space;=&space;\frac{\left&space;(&space;u-c_{u}&space;\right&space;)*depth}{f}&space;\newline&space;\newline&space;x&space;=&space;\frac{\left&space;(&space;v-c_{v}&space;\right&space;)*depth}{f}&space;\newline&space;\newline&space;k&space;=&space;\begin{pmatrix}&space;f&space;&&space;0&space;&&space;u_c&space;\\&space;0&space;&&space;f&space;&&space;u_v&space;\\&space;0&&space;0&space;&&space;1&space;\end{pmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\bg_black&space;\large&space;z&space;=&space;depth&space;\newline&space;\newline&space;x&space;=&space;\frac{\left&space;(&space;u-c_{u}&space;\right&space;)*depth}{f}&space;\newline&space;\newline&space;x&space;=&space;\frac{\left&space;(&space;v-c_{v}&space;\right&space;)*depth}{f}&space;\newline&space;\newline&space;k&space;=&space;\begin{pmatrix}&space;f&space;&&space;0&space;&&space;u_c&space;\\&space;0&space;&&space;f&space;&&space;u_v&space;\\&space;0&&space;0&space;&&space;1&space;\end{pmatrix}" title="\large z = depth \newline \newline x = \frac{\left ( u-c_{u} \right )*depth}{f} \newline \newline x = \frac{\left ( v-c_{v} \right )*depth}{f} \newline \newline k = \begin{pmatrix} f & 0 & u_c \\ 0 & f & u_v \\ 0& 0 & 1 \end{pmatrix}" /></a>
 </p>
 
 For an autonomous agent drivable space includes any space that the agent is physically capable of traversing in 3D. Estimating the drivable space is equivalent to estimating pixels belonging to the ground plane in the scene. RANSAC is used to estimate the ground plane in the 3D camera coordinate frame from the x,y, and z coordinates estimated from depth maps. As the first step, semantic segmentation output is used to extract the relevant pixels belonging to the class you want consider as ground.
 
 <p align="center">
  <img width="500" height="300" src="Baseline Enviroment Perception/Plots/RM.png">
</p>

Then the extracted x, y, and z coordinates of pixels belonging to the road is used to estimate the ground plane along with RANSAC for robust outlier rejection.

 <p align="center">
  <img width="500" height="300" src="Baseline Enviroment Perception/Plots/GM.png">
</p>

#### Lane Estimation

The output of semantic segmentation is used to estimate the lane boundaries of the current lane the agent is using. This task can be separated to two subtasks, lane line estimation, and post-processing through horizontal line filtering and similar line merging.

As the first step to estimate lane proposals, an image containing the semantic segmentation pixels belonging to categories relevant to the lane boundaries was craeted similar to what was done previously for the road plane. Then edge detction was carried out using the Canny Edge Detector on the derived lane boundry image followed by line estimation using Hough Transform on the output of edge detection.

 <p align="center">
  <img width="360" height="300" src="Baseline Enviroment Perception/Plots/LP.png">
</p>

The second step is to perform the estimation of the current lane boundary is to merge redundant lines, and filter out any horizontal lines apparent in the image. Merging redundant lines can be solved through grouping lines with similar slope and intercept. Horizontal lines can be filtered out through slope thresholding.
 <p align="center">
  <img width="360" height="300" src="Baseline Enviroment Perception/Plots/SP.png">
</p>

Finally, lanes are extrapolated to start at the beginning of the road, and end at the end of the road and to  determine the lane markings belong to the current lane, the closest to the lane midpoints from extrapolated lines are selected
 <p align="center">
  <img width="360" height="300" src="Baseline Enviroment Perception/Plots/ML.png">
</p>

#### Estimating Minimum Distance to Obstacles
 <p align="center">
  <img width="1000" height="200" src="Baseline Enviroment Perception/Plots/OD.png">
</p>
2D object detection output is used to determine the minimum distance to impact with obstacles in the scene. However, the 2D detections are from a high recall, low precision 2D object detector. To overcome this problem, the output of the semantic segmentation network is used to eliminate unreliable detections.Then the minimum distance to impact is computed using the remaining bounding boxes and the depth maps.
 <p align="center">
  <img width="360" height="300" src="Baseline Enviroment Perception/Plots/FD.png">
</p>
 <p align="center">
  <img width="360" height="300" src="Baseline Enviroment Perception/Plots/MD.png">
</p>


## Visual Odometry


In the Visual Odometry directory you may find the images taken with a monocular camera set up on the vehicle in the CARLA simulator enviroment. There are 52 data frames, each frame contains RGB images, Grayscale version of that image and a Depth Map. Those frames are
- Processed to extract the features
- Extracted features are used find matches between the features in different photographs
- Use the found matches to estimate the camera motion between subsequent photographs.
- Finally, Estimated camera motion is used to build the vehicle trajectory

<p align="center">
  <img width="1200" height="300" src="Visual Odometry/plots/data.png">
</p>

#### Feature Extraction
For feature extraction you may use different extractors and descriptors such as SIFT, ORB, SURF etc. The obtained features in an arbitary frame is displayed below

<p align="center">
  <img width="600" height="500" src="Visual Odometry/plots/features.png">
</p>

#### Fature Matching

Then the extracted features in each image is matched with the features from the subsequent frame. we can use Brute force matcher of a FLANN based matcher for this purpose. Feature matches are filtered using a threshold on distance between the matches. Matching done betwen two adjacent frames using FLANN based matcher is shown below.

<p align="center">
  <img width="1400" height="500" src="Visual Odometry/plots/matches.png">
</p>

#### Trajectory Estimation
Camera motion estimation from pair of images is done using Essential Matrix Decomposition. We can use Perspective n Ponint Algorithm for this purpose as well. RANSAC was adopted for outlier rejection.
<p align="center">
  <img width="600" height="500" src="Visual Odometry/plots/Camera.png">
</p>

<p align="center">
  <img width="600" height="500" src="Visual Odometry/plots/traj.png">
</p>


## Aplying Stereo Depth

In the Stereo Depth, an Image pair obtained by a stereo camera pair in the CARLA Simulator enviroment is used to,
- Obtain depth details from a pair of stereo images and their respective Projection matrix
- Find the Distance to collision with an obstacle

<p align="center">
  <img width="800" height="300" src="Stereo_Depth/Plots/Figure 2021-05-22 002611.png">
</p>

### Estimating Depth

The depth of a stereo scene is estimated using ,
1. Determine the disparity between the two images.
2. Decompose the projection matrices into the camera intrinsic matrix *K*, and extrinsics *R*, *t*.

#### Computing the Disparity
A disparity map from the perspective of the left camera is generated from the stereo pair

<p align="center">
  <img width="600" height="400" src="Stereo_Depth/Plots/Figure 2021-05-22 002611 (1).png">
</p>

#### Decompose the projection matrices
In Lesson 2 we touched on how to decompose a projection matrix **P**: 
1. Represent **P** as a combination of the intrinsic parameters **K** and the extrinsic rotation **R** and translation **t** as follows: 
<p align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=\bg_black&space;P&space;=&space;\left&space;[&space;R&space;\mid&space;t\right&space;]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\bg_black&space;P&space;=&space;\left&space;[&space;R&space;\mid&space;t\right&space;]" title="P = \left [ R \mid t\right ]" /></a>
</p>
2. Take the inverse of **KR** , which allows us to perform QR-decomposition to get the inberse of R and K: 
<p align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{120}&space;\bg_black&space;\large&space;\left&space;(&space;RK&space;\right&space;)^{-1}&space;=&space;R^{-1}&space;K^{-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;\bg_black&space;\large&space;\left&space;(&space;RK&space;\right&space;)^{-1}&space;=&space;R^{-1}&space;K^{-1}" title="\large \left ( RK \right )^{-1} = R^{-1} K^{-1}" /></a>
 </p>
  
3. From here it would seem as though we could easily determine K, R, and t.

Unfortunately, this isn't as simple as it seems due to the QR-decomposition isn't unique. This results in us having to check the signs of the diagonal of the K matrix and adjust R appropriately. assertions must be made about the directions of the camera and image x, y, and z axes.

After carrying out the matrix multiplication, the homogeneous component w_c will, in general, not be equal to 1. Therefore, to map back into the real plane we must perform the homogeneous divide or perspective divide by dividing each component by w_c

#### Generating the depth map

1. Get the focal length f from the K matrix
2. Compute the baseline $b$ using corresponding values from the translation vectors t
3. Compute depth map of the image:
<p align ="center">
  <a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{120}&space;\bg_black&space;\large&space;Z&space;=&space;\frac{fb}{x_{l}-x_{r}}&space;=&space;\frac{fb}{d}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;\bg_black&space;\large&space;Z&space;=&space;\frac{fb}{x_{l}-x_{r}}&space;=&space;\frac{fb}{d}" title="\large Z = \frac{fb}{x_{l}-x_{r}} = \frac{fb}{d}" /></a>
   </p>


<p align="center">
  <img width="600" height="400" src="Stereo_Depth/Plots/Figure 2021-05-22 002611 (2).png">
</p>

### Calculating Distance to an Obstacle

While we may have a map of the depths of each pixel in the scene, our system does not yet know which of these pixels are safe (like the road) or a potential obstacle (like a motorcycle).In this case,a template motorcycle image that was identified by the detector is used as an obstacke and calculate the distance to it.


<p align="center">
  <img width="300" height="200" src="Stereo_Depth/Plots/Figure 2021-05-22 002611 (3).png">
</p>

The cross corelation is used to find the location of the obstacle template in the image and extract the position of the obstacle. The cross correlation heatmap and the corresponding point of the obstacle can be seen here.

<p align="center">
  <img width="600" height="400" src="Stereo_Depth/Plots/Figure 2021-05-22 002611 (4).png">
</p>

once the location of the obstacle is obtained(bounding box reigon) we can determine the minimum distance using the depth map produced!

<p align="center">
  <img width="900" height="600" src="Stereo_Depth/Plots/Figure 2021-05-22 002611 (5).png">
</p>

## References 
> Visual Perception for Self Driving Cars by University of Toronto on Coursera
