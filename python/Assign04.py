# Sawyer Davis
# CS548 Assignment 1
# 4/28/2025


import numpy as np
from scipy.spatial import cKDTree
from General_Assign04 import create_transform_4x4

def get_matching_points(p_points, q_points):
    tree = cKDTree(q_points)
    distances, indices = tree.query(p_points)
    return indices

def get_centered_cloud(points):
    centered_points = np.copy(points)
    centroid = np.mean(centered_points, axis=0)
    centered_points -= centroid
    return centered_points, centroid

def compute_point_to_point_iteration(p_points, q_points):
    indices = get_matching_points(p_points, q_points)
    q_matches = q_points[indices]

    p_centered, p_centroid = get_centered_cloud(p_points)
    q_centered, q_centroid = get_centered_cloud(q_matches)

    cross_covariance = p_centered.T @ q_centered
    U, S, Vh = np.linalg.svd(cross_covariance)
    R = Vh.T @ U.T

    if np.linalg.det(R) < 0:
        Vh[-1, :] *= -1
        R = Vh.T @ U.T

    Tr = q_centroid - R @ p_centroid

    updated_p_points = (R @ p_points.T).T + Tr
    return updated_p_points, q_matches, R, Tr

def compute_RMSE(p_points, q_points):
    return np.sqrt(np.mean(np.linalg.norm(p_points - q_points, axis=1) ** 2))

def do_point_to_point_icp(p_points, q_points, max_iter=100, min_rmse=1e-6):
    current_rmse = np.inf
    iterations = 0
    total_transform = np.eye(4)

    while current_rmse > min_rmse:
        updated_p_points, q_matches, R, Tr = compute_point_to_point_iteration(p_points, q_points)
        transform_4x4 = create_transform_4x4(R, Tr)

        total_transform = transform_4x4 @ total_transform
        current_rmse = compute_RMSE(updated_p_points, q_matches)

        p_points = updated_p_points
        iterations += 1

        if iterations >= max_iter:
            break

    return p_points, total_transform, iterations, current_rmse
