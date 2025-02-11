// Sawyer Davis
// CS548 Assignment 1
// 2/10/2025

#include "PCD.hpp"

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadPCD(std::string filename) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return nullptr;
    }
    
    std::string line;
    std::unordered_map<std::string, int> field_map;
    bool data_section = false;
    int num_points = 0;
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        
        if (token == "FIELDS") {
            int index = 0;
            while (iss >> token) {
                field_map[token] = index++;
            }
        } else if (token == "POINTS") {
            iss >> num_points;
            cloud->width = num_points;
            cloud->height = 1;
            cloud->points.resize(num_points);
        } else if (token == "DATA") {
            data_section = true;
            break;
        }
    }
    
    if (!data_section) {
        std::cerr << "Error: DATA section not found in PCD file." << std::endl;
        return nullptr;
    }
    
    int index = 0;
    while (std::getline(file, line) && index < num_points) {
        std::istringstream iss(line);
        pcl::PointXYZRGBNormal point;
        std::vector<float> values(field_map.size());
        
        for (float &val : values) {
            iss >> val;
        }
        
        if (field_map.count("x")) point.x = values[field_map["x"]];
        if (field_map.count("y")) point.y = values[field_map["y"]];
        if (field_map.count("z")) point.z = values[field_map["z"]];
        if (field_map.count("normal_x")) point.normal_x = values[field_map["normal_x"]];
        if (field_map.count("normal_y")) point.normal_y = values[field_map["normal_y"]];
        if (field_map.count("normal_z")) point.normal_z = values[field_map["normal_z"]];
        
        if (field_map.count("rgb")) {
            float rgb_f = values[field_map["rgb"]];
            uint32_t rgb_i = *reinterpret_cast<uint32_t*>(&rgb_f);
            point.r = (rgb_i >> 16) & 0xFF;
            point.g = (rgb_i >> 8) & 0xFF;
            point.b = rgb_i & 0xFF;
        }
        
        cloud->points[index++] = point;
    }
    
    file.close();
    return cloud;
}
