// Sawyer Davis
// CS548 Assignment 1
// 2/10/2025

#include "PCD.hpp"
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_pcd_file>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    auto cloud = loadPCD(filename);
    if (!cloud) {
        std::cerr << "Failed to load PCD file." << std::endl;
        return 1;
    }

    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    viewer.setBackgroundColor(0.7, 0.7, 0.7);
    viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud, "cloud");
    
    viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud, 10, 0.01, "normals");
    
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    
    return 0;
}
