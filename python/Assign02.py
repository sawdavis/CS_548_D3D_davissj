# Sawyer Davis
# CS548 Assignment 2
# 2/10/2025

import open3d as o3d
import numpy as np
from collections import deque

class OctoNode:
    def __init__(self):
        self.indices = None
        self.children = None
        self.minP = None
        self.maxP = None

# Traverse octree
def traverse_octree(node):
    if node.indices is not None:
        return node.indices
    else:
        index_list = []
        for child in node.children:
            index_list.append(list(traverse_octree(child)))
        return index_list

# Get best axis-aligned bounding box
def get_best_AABB(cloud, margin=(0, 0, 0)):
    minP = np.min(cloud.points, axis=0) - np.array(margin)
    maxP = np.max(cloud.points, axis=0) + np.array(margin)
    return minP, maxP

# Split box into 8 octants
def split_box_octree(minP, maxP):
    center = (minP + maxP) / 2
    boxes = []
    for z in [0, 1]:
        for y in [0, 1]:
            for x in [0, 1]:
                box_min = minP + (center - minP) * np.array([x, y, z])
                box_max = center + (maxP - center) * np.array([x, y, z])
                boxes.append((box_min, box_max))
    return boxes

# Get distance from point to box
def get_distance_from_box(minP, maxP, point):
    clamped_point = np.maximum(minP, np.minimum(maxP, point))
    return np.linalg.norm(clamped_point - point)

# Get indices inside box
def get_indices_inside(points, indices, minP, maxP):
    inside, outside = [], []
    for idx in indices:
        if np.all(minP <= points[idx]) and np.all(points[idx] <= maxP):
            inside.append(idx)
        else:
            outside.append(idx)
    return inside, outside

# Build octree
def build_octree(cloud, margin, max_depth):
    root = OctoNode()
    points = np.asarray(cloud.points)
    root.minP, root.maxP = get_best_AABB(cloud, margin)
    indices = list(range(len(points)))
    queue = deque([(root, indices, 0)])
    
    while queue:
        node, node_indices, depth = queue.popleft()
        if depth == max_depth or len(node_indices) <= 1:
            node.indices = node_indices
        else:
            node.children = []
            for minP, maxP in split_box_octree(node.minP, node.maxP):
                child = OctoNode()
                child.minP, child.maxP = minP, maxP
                inside, node_indices = get_indices_inside(points, node_indices, minP, maxP)
                child.indices = inside if len(inside) <= 1 else None
                node.children.append(child)
                queue.append((child, inside, depth + 1))
    
    return root

# Radius search using octree
def do_radius_search_octree(cloud, tree, query_point, radius):
    points = np.asarray(cloud.points)
    queue = deque([tree])
    result_indices = []
    
    while queue:
        node = queue.popleft()
        if node.indices is not None:
            result_indices.extend([idx for idx in node.indices if np.linalg.norm(points[idx] - query_point) <= radius])
        else:
            for child in node.children:
                if get_distance_from_box(child.minP, child.maxP, query_point) <= radius:
                    queue.append(child)
    
    return result_indices

def main():
    # Set filename
    filename = "data/assign02/bunny.pcd"
    
    # Load cloud
    cloud = o3d.io.read_point_cloud(filename)
    
    # Get points and colors
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)
    
    # Build octree
    tree = build_octree(cloud, margin=(0.1, 0.1, 0.1), max_depth=4)
    
    # Do radius search
    query_point = np.array([6.79621601, 5.66879749, 8.07549858])
    radius = 1.7
    search_indices = do_radius_search_octree(cloud, tree, query_point, radius)
    
    # Set colors based on indices
    for i in range(len(colors)):
        colors[:] = (0, 0, 0)
    
    for index in search_indices:
        colors[index] = (255, 0, 0)
    
    # Visualize models
    o3d.visualization.draw_geometries([cloud])

if __name__ == "__main__":
    main()
