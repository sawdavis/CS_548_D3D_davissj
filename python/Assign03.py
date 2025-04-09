import numpy as np
import open3d as o3d
import copy

def compute_distances(center, points):
    return np.linalg.norm(points - center, axis=1)

def compute_gaussian_weights(center, points, sigma):
    distances = compute_distances(center, points)
    return np.exp(-(distances**2) / (2 * sigma**2))

def compute_weighted_PCA(points, weights):
    weighted_sum = np.sum(weights)
    centroid = np.sum(points.T * weights, axis=1) / weighted_sum
    centered = points - centroid
    weighted_cov = (centered.T * weights) @ centered / weighted_sum
    eigvals, eigvecs = np.linalg.eigh(weighted_cov)
    U, V, W = eigvecs[:, 2], eigvecs[:, 1], eigvecs[:, 0]
    return centroid, U, V, W

def project_points_to_plane(points, centroid, U, V, W):
    basis = np.vstack((U, V, W)).T
    relative = points - centroid
    return relative @ basis

def reverse_plane_projection(projected, centroid, U, V, W):
    basis = np.vstack((U, V, W))
    return (projected @ basis) + centroid

def make_design_matrix_A(projected):
    u = projected[:, 0]
    v = projected[:, 1]
    return np.stack([np.ones_like(u), u, v, u**2, u*v, v**2], axis=1)

def make_vector_b(projected):
    return projected[:, 2].reshape(-1, 1)

def make_weight_matrix_G(weights):
    return np.diag(weights)

def compute_polynomial_coefficients(projected, weights):
    A = make_design_matrix_A(projected)
    b = make_vector_b(projected)
    G = make_weight_matrix_G(weights)
    AtGA = A.T @ G @ A
    AtGb = A.T @ G @ b
    a = np.linalg.inv(AtGA) @ AtGb
    return a

def project_points_to_polynomial(points, centroid, U, V, W, a):
    projected = project_points_to_plane(points, centroid, U, V, W)
    A = make_design_matrix_A(projected)
    w_pred = (A @ a).flatten()
    projected[:, 2] = w_pred
    return reverse_plane_projection(projected, centroid, U, V, W)

def fit_to_polynomial(center, points, sigma):
    weights = compute_gaussian_weights(center, points, sigma)
    centroid, U, V, W = compute_weighted_PCA(points, weights)
    projected = project_points_to_plane(points, centroid, U, V, W)
    a = compute_polynomial_coefficients(projected, weights)
    center_proj = project_points_to_plane(center.reshape(1, -1), centroid, U, V, W)
    A = make_design_matrix_A(center_proj)
    center_proj[:, 2] = (A @ a).flatten()
    new_center = reverse_plane_projection(center_proj, centroid, U, V, W)
    return new_center, W

def perform_moving_least_squares(cloud, radius, sigma):
    kdtree = o3d.geometry.KDTreeFlann(cloud)
    points = np.asarray(cloud.points)
    output_points = []
    output_normals = []
    output_colors = []

    for i, center in enumerate(points):
        [_, idxs, _] = kdtree.search_radius_vector_3d(center, radius)
        if len(idxs) < 6:
            output_points.append(center)
            output_normals.append([0, 0, 0])
            output_colors.append([0, 0, 0])
            continue
        neighbors = points[idxs]
        new_point, normal = fit_to_polynomial(center, neighbors, sigma)
        color = [np.linalg.norm(new_point - center), 0, 0]
        output_points.append(new_point)
        output_normals.append(normal)
        output_colors.append(color)

    smoothed = o3d.geometry.PointCloud()
    smoothed.points = o3d.utility.Vector3dVector(output_points)
    smoothed.normals = o3d.utility.Vector3dVector(output_normals)
    smoothed.colors = o3d.utility.Vector3dVector(output_colors)
    return smoothed

# Optional visualization function
def visualize_clouds(all_clouds, point_show_normal=False):
    adjusted_clouds = []
    x_inc = 20.0
    y_inc = 20.0
    for i in range(len(all_clouds)):
        one_set_clouds = all_clouds[i]
        for j in range(len(one_set_clouds)):
            center = (x_inc * j, y_inc * i, 0)
            adjusted_clouds.append(one_set_clouds[j].translate(center))
    o3d.visualization.draw_geometries(adjusted_clouds, point_show_normal=point_show_normal)

def main():
    cloud = o3d.io.read_point_cloud("data/assign03/input/noise_pervasive_large_bunny.pcd")
    radius = 1.0
    sigma = radius / 3.0
    output_cloud = perform_moving_least_squares(cloud, radius, sigma)

    output_points_only = copy.deepcopy(output_cloud)
    output_points_only.colors = o3d.utility.Vector3dVector([])
    output_points_only.normals = o3d.utility.Vector3dVector([])

    output_points_normals = copy.deepcopy(output_cloud)
    output_points_normals.colors = o3d.utility.Vector3dVector([])

    visualize_clouds([[cloud, output_points_only, output_points_normals, output_cloud]], point_show_normal=True)

if __name__ == "__main__":
    main()
