# Visual-Perception-for-Self-Driving-Cars

## Aplying Stereo Depth

In this Stereo Depth an Image pair obtained by a stereo camera pair in the CARLA Simulator enviroment is used to,
- Obtain depth details from a pair of stereo images and their respective Projection matrix
- Find the Distance to collision with an obstacle

<p align="center">
  <img width="1200" height="400" src="Stereo_Depth/Plots/Figure 2021-05-22 002611.png">
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
