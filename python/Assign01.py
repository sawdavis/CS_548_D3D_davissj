# Sawyer Davis
# CS548 Assignment 1
# 2/10/2025

import sys
import numpy as np
import open3d as o3d

def load_pcd(filename):
    with open(filename, 'r') as file:
        header = []
        data_start = False
        fields = []
        num_points = 0
        
        for line in file:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("POINTS"):
                num_points = int(line.split()[1])
            elif line.startswith("DATA"):
                data_start = True
                break
        
        if not data_start:
            raise ValueError("DATA section not found in PCD file.")
        
        # Read the data section
        data = np.loadtxt(file, dtype=np.float32)
        
        # Extract XYZ fields
        xyz = np.zeros((num_points, 3), dtype=np.float32)
        normals = np.zeros((num_points, 3), dtype=np.float32)
        
        for i, field in enumerate(fields):
            if field == 'x':
                xyz[:, 0] = data[:, i]
            elif field == 'y':
                xyz[:, 1] = data[:, i]
            elif field == 'z':
                xyz[:, 2] = data[:, i]
            elif field == 'normal_x':
                normals[:, 0] = data[:, i]
            elif field == 'normal_y':
                normals[:, 1] = data[:, i]
            elif field == 'normal_z':
                normals[:, 2] = data[:, i]
        
        # Extract RGB if present
        if 'rgb' in fields:
            rgb_index = fields.index('rgb')
            rgb_f = data[:, rgb_index].astype(np.float32)
            rgb_i = rgb_f.view(np.uint32)
            r = ((rgb_i >> 16) & 255).astype(np.float32) / 255.0
            g = ((rgb_i >> 8) & 255).astype(np.float32) / 255.0
            b = (rgb_i & 255).astype(np.float32) / 255.0
            colors = np.stack((r, g, b), axis=-1)
        else:
            colors = None
        
        # Create Open3D PointCloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        if np.any(normals):
            pcd.normals = o3d.utility.Vector3dVector(normals)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd

def main():
    if len(sys.argv) < 2:
        print("Usage: python Assign01.py <path_to_pcd_file>")
        sys.exit(1)
    
    filename = sys.argv[1]
    pcd = load_pcd(filename)
    o3d.visualization.draw_geometries([pcd], point_show_normal=pcd.has_normals())

if __name__ == "__main__":
    main()

