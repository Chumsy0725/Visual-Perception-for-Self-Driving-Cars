"""
Reference: Self Driving Cars Specialization by University of Toronto on Coursera
"""

import cv2
from utils import *


def visualize_data(i=30):
    """Visualize Data of a given image frame"""

    plt.figure()
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(36, 10))
    image = dataset_handler.images[i]
    image_rgb = dataset_handler.images_rgb[i]
    depth = dataset_handler.depth_maps[i]

    ax1.imshow(image_rgb)
    ax1.set_title('RGB Image')
    ax1.axis('tight')
    ax1.axis('off')
    ax2.imshow(image, cmap='gray')
    ax2.set_title('Gray Scale Image')
    ax2.axis('off')
    ax2.axis('tight')
    ax3.imshow(depth, cmap='jet')
    ax3.set_title('Depth Map')
    ax3.axis('off')
    ax3.axis('tight')
    plt.subplots_adjust(wspace=0.005, hspace=0)
    plt.show()


def extract_features(image):
    """
    Return keypoints and descriptors for the image
    """
    surf = cv2.SIFT_create()
    kp, des = surf.detectAndCompute(image, None)
    # orb = cv.ORB_create(nfeatures=500)
    # kp = orb.detect(image, None)
    # kp, des = orb.compute(image, kp)

    return kp, des


def visualize_features(image, kp):
    """
    Visualize extracted features in the image
    """
    display = cv2.drawKeypoints(image, kp, None)
    plt.imshow(display)
    plt.show()


def visualize_features(image, kp):
    """
    Visualize extracted features in the image
    """
    display = cv2.drawKeypoints(image, kp, None)
    plt.figure(figsize=(8, 6), dpi=100)
    plt.axis('off')
    plt.imshow(display)
    plt.show()


def extract_features_dataset(images):
    """
    Returns keypoints and descriptors for each image in the dataset
    """
    kp_list = []
    des_list = []

    for img in images:
        kp, des = extract_features(img)
        kp_list.append(kp)
        des_list.append(des)

    return kp_list, des_list


def match_features(des1, des2):
    """
    Return Matched features from two images
    """

    # FLANN parameters
    FLANN_INDEX_KDTREE = 5
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)  # or pass empty dictionary

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    match = flann.knnMatch(des1, des2, k=2)

    # for ORB
    # FLANN_INDEX_LSH = 6
    # index_params= dict(algorithm = FLANN_INDEX_LSH,
    #               table_number = 6, # 12
    #               key_size = 12,     # 20
    #               multi_probe_level = 1) #2
    # search_params = dict(checks=50)
    # flann = cv.FlannBasedMatcher(index_params,search_params)
    # match = flann.knnMatch(des1,des2,k=2)

    return match


def filter_matches_distance(match, dist_threshold):
    """
    Filter matched features from two images by distance between the best matches
    dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0)
    """
    filtered_match = []

    for i, (m, n) in enumerate(match):
        if m.distance < dist_threshold * n.distance:
            filtered_match.append(m)

    return filtered_match


def visualize_matches(image1, kp1, image2, kp2, match):
    """
    Visualize corresponding matches in two images
    """

    image_matches = cv2.drawMatches(image1, kp1, image2, kp2, match, None, flags=2)
    plt.figure(figsize=(16, 6), dpi=100)
    plt.imshow(image_matches)
    plt.show()


def matches(n=100, filtering=True, i=0):
    """
    Visualize First n matches
    """
    image1 = dataset_handler.images[i]
    image2 = dataset_handler.images[i + 1]

    kp1 = kp_list[i]
    kp2 = kp_list[i + 1]

    des1 = des_list[i]
    des2 = des_list[i + 1]

    match = match_features(des1, des2)
    if filtering:
        dist_threshold = 0.6
        match = filter_matches_distance(match, dist_threshold)

    visualize_matches(image1, kp1, image2, kp2, match[:n])


def match_features_dataset(des_list, match_features):
    """
    Return Matched features for each subsequent image pair in the dataset
    """
    matches = []

    for i in range(len(des_list) - 1):
        descriptor1 = des_list[i]
        descriptor2 = des_list[i + 1]
        match = match_features(descriptor1, descriptor2)
        matches.append(match)

    return matches


def filter_matches_dataset(filter_matches_distance, matches, dist_threshold):
    """
    Return filtered matched features by distance for each subsequent image pair in the dataset
    dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0)
    """
    filtered_matches = []

    for m in matches:
        new_match = filter_matches_distance(m, dist_threshold)
        filtered_matches.append(new_match)

    return filtered_matches


def motion(i, matches, kp_list):
    """
    Visualize the motion between two adjacent frames
    """
    match = matches[i]
    kp1 = kp_list[i]
    kp2 = kp_list[i + 1]
    k = dataset_handler.k
    depth = dataset_handler.depth_maps[i]

    rmat, tvec, image1_points, image2_points = estimate_motion(match, kp1, kp2, k, depth1=depth)
    image1 = dataset_handler.images_rgb[i]
    image2 = dataset_handler.images_rgb[i + 1]

    image_move = visualize_camera_movement(image1, image1_points, image2, image2_points)
    plt.figure(figsize=(16, 12), dpi=100)
    plt.imshow(image_move)
    plt.show()


def estimate_motion(match, kp1, kp2, k, depth1=None):
    """
    Estimate camera motion from a pair of subsequent image frames
    Returns Rotation Matrix, Translation vector, a list of selected match coordinates in the first image
    and second image
    """

    image1_points = []
    image2_points = []

    for m in match:
        train_idx = m.trainIdx
        query_idx = m.queryIdx

        p1x, p1y = kp1[query_idx].pt
        image1_points.append([p1x, p1y])

        p2x, p2y = kp2[train_idx].pt
        image2_points.append([p2x, p2y])

    E, mask = cv2.findEssentialMat(np.array(image1_points), np.array(image2_points), k, method = cv.RANSAC, prob = 0.9)

    retval, rmat, tvec, mask = cv2.recoverPose(E, np.array(image1_points), np.array(image2_points), k)

    return rmat, tvec, image1_points, image2_points


def estimate_trajectory(estimate_motion, matches, kp_list, k, depth_maps=[]):
    """
    Estimate complete camera trajectory from subsequent image pairs
    """
    trajectory = [np.array([0, 0, 0])]

    R = np.diag([1, 1, 1])
    T = np.zeros([3, 1])
    RT = np.hstack([R, T])
    RT = np.vstack([RT, np.zeros([1, 4])])
    RT[-1, -1] = 1

    for i in range(len(matches)):
        match = matches[i]
        kp1 = kp_list[i]
        kp2 = kp_list[i + 1]
        depth = depth_maps[i]

        rmat, tvec, image1_points, image2_points = estimate_motion(match, kp1, kp2, k, depth)
        rt_mtx = np.hstack([rmat, tvec])
        rt_mtx = np.vstack([rt_mtx, np.zeros([1, 4])])
        rt_mtx[-1, -1] = 1

        rt_mtx_inv = np.linalg.inv(rt_mtx)

        RT = np.dot(RT, rt_mtx_inv)
        new_trajectory = RT[:3, 3]
        trajectory.append(new_trajectory)

    trajectory = np.array(trajectory).T

    return trajectory


if __name__ == '__main__':
    # Data visualisation and Loading
    dataset_handler = DatasetHandler()
    visualize_data()
    print('Camera Calibration Matrix: \n')
    k = dataset_handler.k
    print(k)
    print("Number of frames: ", dataset_handler.num_frames)

    # Feature Extraction
    images = dataset_handler.images
    kp_list, des_list = extract_features_dataset(images)

    # Feature Matching
    matches()  # For visualize matches between two frames

    matches = match_features_dataset(des_list, match_features)
    dist_threshold = 0.5
    filtered_matches = filter_matches_dataset(filter_matches_distance, matches, dist_threshold)
    matches = filtered_matches

    # Trajectory Estimation
    motion(30, matches, kp_list)  # Visualize the motion between two adjacent frames
    depth_maps = dataset_handler.depth_maps
    trajectory = estimate_trajectory(estimate_motion, matches, kp_list, k, depth_maps=depth_maps)

    visualize_trajectory(trajectory)
