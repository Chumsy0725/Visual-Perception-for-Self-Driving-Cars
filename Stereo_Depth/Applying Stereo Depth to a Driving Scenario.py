import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib import patches
import files_management

def compute_left_disparity_map(img_left, img_right):
    """
    Returns the disparity map from the prespective of the left camera given the stereo pair
    """

    # Parameters
    num_disparities = 128
    block_size = 11
    min_disparity = 0
    window_size = 6
    
    # Stereo SGBM matcher
    left_matcher_SGBM = cv2.StereoSGBM_create(
        minDisparity=min_disparity,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * 3 * window_size ** 2,
        P2=32 * 3 * window_size ** 2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    # Compute the left disparity map
    disp_left = left_matcher_SGBM.compute(img_left, img_right).astype(np.float32)/16
    
    return disp_left


def decompose_projection_matrix(p):
    """
    Returns Intrinsic Parameters, Rotation Matrix and Translation Vector given the Projection matrix
    """
    
    k, r, t, _, _, _, _ = cv2.decomposeProjectionMatrix(p)
    t = t / t[3]
    #After carrying out the matrix multiplication, the homogeneous component  𝑤𝑐  will, in general, not be equal to 1. 
    #Therefore, to map back into the real plane we must perform the homogeneous divide or perspective divide by dividing
    #each component by  𝑤𝑐
    
    return k, r, t

def calc_depth_map(disp_left, k_left, t_left, t_right):
    """
    Returns the Depth map given the disparity map given the disparity map left camera. inytrinsics of left camera and translation vectors
    """
    
    # Get the focal length from the K matrix
    f = k_left[0, 0]

    # Get the distance between the cameras from the t matrices (baseline)
    b = t_left[1] - t_right[1]

    # Replace all instances of 0 and -1 disparity with a small minimum value (to avoid div by 0 or negatives)
    disp_left[disp_left == 0] = 0.1
    disp_left[disp_left == -1] = 0.1

    # Initialize the depth map to match the size of the disparity map
    depth_map = np.ones(disp_left.shape, np.single)

    # Calculate the depths 
    depth_map[:] = f * b / disp_left[:]

    return depth_map


def locate_obstacle_in_image(image, obstacle_image):
    """
    Returns the Cross Corelation Map and Location of the obstacle given an Image and an Obstacle Template
    """
    
    # Run the template matching from OpenCV
    cross_corr_map = cv2.matchTemplate(image, obstacle_image, method=cv2.TM_CCOEFF)
    
    # Locate the position of the obstacle using the minMaxLoc function from OpenCV
    _, _, _, obstacle_location = cv2.minMaxLoc(cross_corr_map)
    
    return cross_corr_map, obstacle_location


def calculate_nearest_point(depth_map, obstacle_location, obstacle_img):
    """
    Return the bounding box and the narest distance to the obstale given the obstacle template, obstacle location and the depth map
    """

    # Gather the relative parameters of the obstacle box
    obstacle_width = obstacle_img.shape[0]
    obstacle_height = obstacle_img.shape[1]
    obstacle_min_x_pos = obstacle_location[1]
    obstacle_max_x_pos = obstacle_location[1] + obstacle_width
    obstacle_min_y_pos = obstacle_location[0]
    obstacle_max_y_pos = obstacle_location[0] + obstacle_height

    # Get the depth of the pixels within the bounds of the obstacle image, find the closest point in this rectangle
    obstacle_depth = depth_map_left[obstacle_min_x_pos:obstacle_max_x_pos, obstacle_min_y_pos:obstacle_max_y_pos]
    closest_point_depth = obstacle_depth.min()
 
    # Create the obstacle bounding box 
    obstacle_bbox = patches.Rectangle((obstacle_min_y_pos, obstacle_min_x_pos), obstacle_height, obstacle_width, 
                                 linewidth=1, edgecolor='r', facecolor='none')
    
    return closest_point_depth, obstacle_bbox



# Read the stereo-pair of images
img_left = files_management.read_left_image()
img_right = files_management.read_right_image()

# Use matplotlib to display the two images
_, image_cells = plt.subplots(1, 2, figsize=(20, 20))
image_cells[0].imshow(img_left)
image_cells[0].set_title('left image')
image_cells[1].imshow(img_right)
image_cells[1].set_title('right image')
plt.savefig('Stereo_pair.png')


# Large plot of the left image
plt.figure(figsize=(16, 12), dpi=100)


# Read the calibration
p_left, p_right = files_management.get_projection_matrices()

# Use regular numpy notation instead of scientific one 
#np.set_printoptions(suppress=True)
#print("p_left \n", p_left)
#print("\np_right \n", p_right)

#############  Computing Disparity ############################################


# Compute the disparity map using the fuction above
disp_left = compute_left_disparity_map(img_left, img_right)

# Show the left disparity map
plt.figure(figsize=(12, 12))
plt.title("Disparity Map")
plt.axis('off')
plt.savefig('Disparity map.png')
plt.imshow(disp_left)


############# Decomposing the Projection Matrix ###############################

# Decompose each matrix
k_left, r_left, t_left = decompose_projection_matrix(p_left)
k_right, r_right, t_right = decompose_projection_matrix(p_right)

# Display the matrices
#print("k_left \n", k_left)
#print("\nr_left \n", r_left)
#print("\nt_left \n", t_left)
#print("\nk_right \n", k_right)
#print("\nr_right \n", r_right)
#print("\nt_right \n", t_right)

############ Depth Map Genertion ##############################################

# Get the depth map by calling the above function
depth_map_left = calc_depth_map(disp_left, k_left, t_left, t_right)

# Display the depth map
plt.figure(figsize=(12, 12), dpi=100)
plt.title("Depth Map")
plt.savefig('Depth Map.png')
plt.axis('off')
plt.imshow(depth_map_left, cmap='flag')


############# Distance to Collision ##########################################

# Get the image of the obstacle
obstacle_image = files_management.get_obstacle_image()

# Show the obstacle image
plt.figure(figsize=(6, 6))
plt.axis('off')
plt.title("Obstacle Image")
plt.imshow(obstacle_image)
plt.savefig("obstacle template.png")

# Gather the cross correlation map and the obstacle location in the image
cross_corr_map, obstacle_location = locate_obstacle_in_image(img_left, obstacle_image)

# Display the cross correlation heatmap 
plt.figure(figsize=(12, 12))
plt.title("Cross corelation Map")
plt.imshow(cross_corr_map)
plt.text(750,1000, 'Obstacle Location:'+str(obstacle_location), fontsize =15)
plt.savefig("Crosscorelation map.png")


# nearest point function to get the closest point depth and obstacle bounding box
closest_point_depth, obstacle_bbox = calculate_nearest_point(depth_map_left, obstacle_location, obstacle_image)

# Display the image with the bounding box displayed
fig, ax = plt.subplots(1, figsize=(12, 12))
ax.imshow(img_left)
ax.add_patch(obstacle_bbox)
plt.axis('off')
plt.text(750,1000, 'closest_point_depth:'+str(closest_point_depth), fontsize =15)
plt.savefig("obstacle.png")
plt.show()







"""
Reference : Visual Perception for Self Driving Cars by University of Toronto on Coursera 
"""