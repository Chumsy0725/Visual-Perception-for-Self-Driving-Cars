"""
Reference: Self Driving Cars Specialization by University of Toronto on Coursera
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
from utils import *


def xy_from_depth(depth, k):
    """
    Returns x, and y coordinates of every pixel in the image using the depth map and the calibration matrix.
    """

    sz = depth.shape

    f = k[0, 0]
    c_u = k[0, 2]
    c_v = k[1, 2]

    u, v = np.meshgrid(np.arange(1, sz[1] + 1, 1), np.arange(1, sz[0] + 1, 1))

    x = ((u - c_u) * depth) / f
    y = ((v - c_v) * depth) / f

    return x, y


def compute_plane(xyz):
    """
    Computes plane coefficients a,b,c,d of the plane in the form ax+by+cz+d = 0
    """
    ctr = xyz.mean(axis=1)
    normalized = xyz - ctr[:, np.newaxis]
    M = np.dot(normalized, normalized.T)

    p = np.linalg.svd(M)[0][:, -1]
    d = np.matmul(p, ctr)

    p = np.append(p, -d)

    return p


def dist_to_plane(plane, x, y, z):
    """
    Computes distance between points provided by their x, and y, z coordinates
    and a plane in the form ax+by+cz+d = 0
    """
    a, b, c, d = plane

    return (a * x + b * y + c * z + d) / np.sqrt(a ** 2 + b ** 2 + c ** 2)


def ransac_plane_fit(xyz_data):
    """
    Returns plane coefficients a,b,c,d of the plane in the form ax+by+cz+d = 0
    using ransac for outlier rejection.
    """

    num_itr = 100  # RANSAC maximum number of iterations
    min_num_inliers = 50000  # RANSAC minimum number of inliers
    distance_threshold = 0.05  # Maximum distance from point to plane for point to be considered inlier
    max_inliers = 0
    x = xyz_data[0, :]
    y = xyz_data[0, :]
    z = xyz_data[0, :]

    for i in range(num_itr):
        # Choose a minimum of 3 points from xyz_data at random.
        indexes = np.random.choice(xyz_data.shape[1], 25, replace=False)
        # Plane model
        p = compute_plane(xyz_data[:, indexes])
        # Number of inliers
        dist = np.abs(dist_to_plane(p, x, y, z))
        n_inliers = np.sum(dist < distance_threshold)
        # Check if the current number of inliers is greater than all previous iterations and keep the inlier set with the largest number of points.
        if max_inliers < n_inliers:
            max_inliers = n_inliers
            inlier_set = xyz_data[:, dist < distance_threshold]
        # Stopping criterion
        if max_inliers > min_num_inliers:
            break

    # Recompute the model parameters using largest inlier set.
    output_plane = compute_plane(inlier_set)

    return output_plane


def estimate_lane_lines(segmentation_output):
    """
    Returns lines belonging to lane boundaries. Multiple lines could correspond to a single lane.
    """
    # New Segm3ntation with pixels belonging to lane boundary categories
    new_segmentation = np.zeros(segmentation_output.shape)
    new_segmentation[segmentation_output == 6] = 255
    new_segmentation[segmentation_output == 8] = 255

    # Edge Detection using cv2.Canny()
    new_segmentation = cv2.GaussianBlur(new_segmentation, (5, 5), 1)
    edges = cv2.Canny(new_segmentation.astype(np.uint8), 100, 150, apertureSize=3)

    # Line estimation using Hough Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=80, maxLineGap=25)
    lines = np.squeeze(lines, axis=1)

    return lines


def get_slope_intecept(lines):
    slopes = (lines[:, 3] - lines[:, 1]) / (lines[:, 2] - lines[:, 0] + 0.001)
    intercepts = ((lines[:, 3] + lines[:, 1]) - slopes * (
            lines[:, 2] + lines[:, 0])) / 2
    return slopes, intercepts


def merge_lane_lines(lines):
    """
    Merges lane lines to output a single line per lane, using the slope and intercept as similarity measures.
    Also, filters horizontal lane lines based on a minimum slope threshold and returns the coordinates of two
    points in a line.
    """

    slope_similarity_threshold = 0.1
    intercept_similarity_threshold = 40
    min_slope_threshold = 0.3
    clusters = []
    current_inds = []
    itr = 0

    slopes, intercepts = get_slope_intecept(lines)

    horizontal_slopes = np.abs(slopes) > min_slope_threshold

    for slope, intercept in zip(slopes, intercepts):
        in_clusters = np.array([itr in current for current in current_inds])
        if not in_clusters.any():
            slope_cluster = np.logical_and(slopes < (slope + slope_similarity_threshold),
                                           slopes > (slope - slope_similarity_threshold))
            intercept_cluster = np.logical_and(intercepts < (intercept + intercept_similarity_threshold),
                                               intercepts > (intercept - intercept_similarity_threshold))
            inds = np.argwhere(slope_cluster & intercept_cluster & horizontal_slopes).T
            if inds.size:
                current_inds.append(inds.flatten())
                clusters.append(lines[inds])
        itr += 1

    merged_lines = [np.mean(cluster, axis=1) for cluster in clusters]
    merged_lines = np.array(merged_lines).reshape((-1, 4))

    return merged_lines


def filter_detections_by_segmentation(detections, segmentation_output):
    """
    Return filter 2D detection output based on a semantic segmentation map.
    """

    ratio_threshold = 0.3  # If 1/3 of the total pixels belong to the target category, the detection is correct.
    filtered_detections = []

    for detection in detections:

        # Step 1: Compute number of pixels belonging to the category for every detection.
        class_name, x_min, y_min, x_max, y_max, score = detection
        x_min = int(float(x_min))
        y_min = int(float(y_min))
        x_max = int(float(x_max))
        y_max = int(float(y_max))
        box_area = (x_max - x_min) * (y_max - y_min)
        if class_name == 'Car' or class_name == 'Cyclist':
            class_index = 10
        elif class_name == 'Pedestrian':
            class_index = 4
        correct_pixels = len(np.where(segmentation_output[y_min:y_max, x_min:x_max] == class_index)[0])

        # Devide the computed number of pixels by the area of the bounding box (total number of pixels).
        ratio = correct_pixels / box_area

        # If the ratio is greater than a threshold keep the detection. Else, remove the detection from the list of detections.
        if ratio > ratio_threshold:
            filtered_detections.append(detection)

    return filtered_detections


def find_min_distance_to_detection(detections, x, y, z):
    """
    Returns min_distances to impact with every object in the scene.
    """
    min_dist = []
    center_cam_dist = np.sqrt(x**2+y**2+z**2)
    for detection in detections:
        bounding_box = np.asfarray(detection[1:5])
        center_cam_dist_crop = center_cam_dist[int(bounding_box[1]):int(bounding_box[3]),
                                                int(bounding_box[0]):int(bounding_box[2])]
        min_dist.append(np.min(center_cam_dist_crop))

    return np.array(min_dist)


if __name__ == '__main__':
    dataset_handler = DatasetHandler()
    print('Camera Calibration Matrix: \n')
    k = dataset_handler.k
    print(k)

    i = 0  # Frame Number
    dataset_handler.set_frame(i)
    image = dataset_handler.image
    depth = dataset_handler.depth
    segmentation = dataset_handler.segmentation
    
    ########################## Drivable Space Estimation Using Semantic Segmentation Output ######################
    k = dataset_handler.k
    z = dataset_handler.depth
    x, y = xy_from_depth(z, k)

    road_mask = np.zeros(segmentation.shape)
    road_mask[segmentation == 7] = 1

    plt.imshow(road_mask)
    plt.axis('off')
    plt.title('Road Masked Sengemtation Image')
    plt.show()

    x_ground = x[road_mask == 1]
    y_ground = y[road_mask == 1]
    z_ground = dataset_handler.depth[road_mask == 1]
    xyz_ground = np.stack((x_ground, y_ground, z_ground))

    p_final = ransac_plane_fit(xyz_ground)
    print('Ground Plane: ' + str(p_final))

    dist = np.abs(dist_to_plane(p_final, x, y, z))

    ground_mask = np.zeros(dist.shape)
    t = 0.05
    ground_mask[dist < t] = 1
    ground_mask[dist > t] = 0

    plt.imshow(ground_mask)
    plt.axis('off')
    plt.title("Estimated Ground Plane")
    plt.show()

    #################### Lane Estimation Using The Semantic Segmentation Output ###############################

    lane_lines = estimate_lane_lines(segmentation)

    plt.imshow(dataset_handler.vis_lanes(lane_lines))
    plt.axis('off')
    plt.title('Line Proposals')
    plt.show()

    merged_lane_lines = merge_lane_lines(lane_lines)

    plt.imshow(dataset_handler.vis_lanes(merged_lane_lines))
    plt.axis('off')
    plt.title('Filtered Proposals')
    plt.show()

    max_y = dataset_handler.image.shape[0]
    min_y = np.min(np.argwhere(road_mask == 1)[:, 0])

    extrapolated_lanes = extrapolate_lines(merged_lane_lines, max_y, min_y)
    final_lanes = find_closest_lines(extrapolated_lanes, dataset_handler.lane_midpoint)
    plt.imshow(dataset_handler.vis_lanes(final_lanes))
    plt.axis('off')
    plt.title('Lane Boundries')
    plt.show()

    ###################################### Minimum Distance To Impact  #####################################

    detections = dataset_handler.object_detection

    plt.imshow(dataset_handler.vis_object_detection(detections))
    plt.axis("off")
    plt.title('Object Detection Output')
    plt.show()

    filtered_detections = filter_detections_by_segmentation(detections, segmentation)

    plt.imshow(dataset_handler.vis_object_detection(filtered_detections))
    plt.axis('off')
    plt.title('Filtered Object Detection Output')
    plt.show()

    min_distances = find_min_distance_to_detection(filtered_detections, x, y, z)

    print('Minimum distance to impact is: ' + str(min_distances))

    font = {'family': 'serif', 'color': 'red', 'weight': 'normal', 'size': 12}

    im_out = dataset_handler.vis_object_detection(filtered_detections)

    for detection, min_distance in zip(filtered_detections, min_distances):
        bounding_box = np.asfarray(detection[1:5])
        plt.text(bounding_box[0], bounding_box[1] - 20, 'Distance to Impact:' + str(np.round(min_distance, 2)) + ' m',
                 fontdict=font)

    plt.imshow(im_out)
    plt.axis('off')
    plt.title('Minimum Distance')
    plt.show()