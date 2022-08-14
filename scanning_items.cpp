/***********************************************************************************************************************
* @file scanning_items.cpp
* @brief scanning overhead items given a point cloud data file
* @author khang nguyen
***********************************************************************************************************************/

#include "CloudVisualizer.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/io.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>

#define NUM_COMMAND_ARGS 2

using namespace std;
using namespace pcl;

// Point-Picking Call Back function 
void pointPickingCallback(const visualization::PointPickingEvent& event, void* cookie) {
    static int pickCount = 0;
    static PointXYZRGBA lastPoint;

    PointXYZRGBA p;
    event.getPoint(p.x, p.y, p.z);
    cout << "POINT CLICKED: " << p.x << " " << p.y << " " << p.z << endl;

    if(pickCount % 2 == 1) {
        double d = sqrt((p.x - lastPoint.x) * (p.x - lastPoint.x) + (p.y - lastPoint.y) * (p.y - lastPoint.y) + (p.z - lastPoint.z) * (p.z - lastPoint.z));
        cout << "DISTANCE BETWEEN THE POINTS: " << d << endl;
    }

    lastPoint.x = p.x;
    lastPoint.y = p.y;
    lastPoint.z = p.z;
    pickCount++;
}

// Keyboard Call Back Function
void keyboardCallback(const visualization::KeyboardEvent &event, void* viewer_void) {
    if(event.keyDown()) {
        switch(event.getKeyCode()) {
            case 'q':
                cout << "'Q' PRESSED. PROGRAM TERMINATED!" << endl;
                break;
            default:
                break;
        }
    }
}

// Function opening Point Cloud Data File
bool openCloud(PointCloud<PointXYZRGBA>::Ptr &cloudOut, const char* fileName) {
    string fileNameStr(fileName);
    string fileExtension = fileNameStr.substr(fileNameStr.find_last_of(".") + 1);

    if (fileExtension.compare("pcd") == 0) {
        if (io::loadPCDFile<PointXYZRGBA>(fileNameStr, *cloudOut) == -1) {
            PCL_ERROR("error while attempting to read pcd file: %s \n", fileNameStr.c_str());
            return false;
        } else return true;
    } else if (fileExtension.compare("ply") == 0) {
        if (io::loadPLYFile<PointXYZRGBA>(fileNameStr, *cloudOut) == -1) {
            PCL_ERROR("error while attempting to read pcl file: %s \n", fileNameStr.c_str());
            return false;
        } else return true;
    } else {
        PCL_ERROR("error while attempting to read unsupported file: %s \n", fileNameStr.c_str());
        return false;
    }
}

bool saveCloud(const PointCloud<PointXYZRGBA>::ConstPtr &cloudIn, string fileName, bool binaryMode=true) {
    // If the input cloud is empty, return
    if(cloudIn->points.size() == 0) {
        return false;
    }

    // Attempt to save the file
    if(io::savePCDFile<PointXYZRGBA>(fileName, *cloudIn, binaryMode) == -1) {
        PCL_ERROR("error while attempting to save pcd file: %s \n", fileName);
        return false;
    } else {
        printf("Saved 'output.pcd' to BUILD directory!\n");
    } return true;
}

// Function segmenting Planes
void segmentPlane(const PointCloud<PointXYZRGBA>::ConstPtr &cloudIn, PointIndices::Ptr &inliers, double distanceThreshold, int maxIterations) {
    // Store the model coefficients
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);

    // Create the segmentation object for the planar model and set the parameters
    SACSegmentation<PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    // seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);
}

// Function segmenting Upper Planes of Boxes
void segmentBoxPlane(const PointCloud<PointXYZRGBA>::ConstPtr &cloudIn, PointIndices::Ptr &inliers, double distanceThreshold, int maxIterations) {
    // Store the model coefficients
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);

    // Create the segmentation object for the planar model and set the parameters
    SACSegmentation<PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    // seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);
}

// Function segmenting Spherical Shapes of Balls
void segmentSphere(const PointCloud<PointXYZRGBA>::ConstPtr &cloudIn, PointIndices::Ptr &inliers1) {
    // Store the model coefficients
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);

    // Create the segmentation object for the planar model and set the parameters
    SACSegmentation<PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_SPHERE);
    seg.setMethodType(SAC_RANSAC);
    // seg.setMaxIterations(20000);
    // seg.setDistanceThreshold(0.02);
    // seg.setRadiusLimits(0, 0.6);

    // GOOD CONFIGURATION
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.017);
    seg.setRadiusLimits(0.0, 0.7);

    // seg.setMaxIterations(9500);
    // seg.setDistanceThreshold(0.015);
    // seg.setRadiusLimits(0.01, 0.9);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers1, *coefficients);
}

int main(int argc, char** argv) {
    static int num_plane = 0, num_sphere = 0;
    // Check input arguments
    if(argc != NUM_COMMAND_ARGS) {
        printf("USAGE: %s <file_name>\n", argv[0]);
        return 0;
    }

    char* fileName = argv[1];
    CloudVisualizer CV("Segmentation Window");

    // Open the point cloud
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>);
    openCloud(cloud, fileName);

    // Downsample the cloud using voxel grid filter
    const float voxelSize = 0.01;
    PointCloud<PointXYZRGBA>::Ptr cloudFiltered(new PointCloud<PointXYZRGBA>);
    VoxelGrid<PointXYZRGBA> voxFilter;
    voxFilter.setInputCloud(cloud);
    voxFilter.setLeafSize(static_cast<float>(voxelSize), static_cast<float>(voxelSize), static_cast<float>(voxelSize));
    voxFilter.filter(*cloudFiltered);

    // Segment box planes
    const float box_distanceThreshold = 0.065;
    const int box_maxIterations = 6000;
    PointIndices::Ptr box_inliers(new PointIndices);
    segmentPlane(cloud, box_inliers, box_distanceThreshold, box_maxIterations);
    num_plane = num_plane + 2;
    // cout << "Segmentation box points: " << box_inliers->indices.size() << endl;

    // Color the plane inliers green
    for(int i = 0; i < box_inliers->indices.size(); i++) {
        int index = box_inliers->indices.at(i);
        cloud->points.at(index).r = 0;
        cloud->points.at(index).g = 255;
        cloud->points.at(index).b = 0;
    }

    // Segment a sphere
    PointIndices::Ptr inliers1(new PointIndices);
    segmentSphere(cloud, inliers1);
    num_sphere = num_sphere + 1;
    // cout << "Segmentation sphere points: " << inliers1->indices.size() << endl;
    
    // Color the sphere inliers red
    for(int i = 0; i < inliers1->indices.size(); i++) {
        int index = inliers1->indices.at(i);
        cloud->points.at(index).r = 255;
        cloud->points.at(index).g = 0;
        cloud->points.at(index).b = 0;
    }

    // Segment a plane
    const float distanceThreshold = 0.0254;
    const int maxIterations = 1000;
    PointIndices::Ptr inliers(new PointIndices);
    segmentPlane(cloud, inliers, distanceThreshold, maxIterations);
    num_plane = num_plane + 1;
    // cout << "Segmentation plane points: " << inliers->indices.size() << endl;

    // Color the plane inliers blue
    for(int i = 0; i < inliers->indices.size(); i++) {
        int index = inliers->indices.at(i);
        cloud->points.at(index).r = 0;
        cloud->points.at(index).g = 0;
        cloud->points.at(index).b = 255;
    }

    // Update the scene
    CV.addCloud(cloud);
    CV.addCoordinateFrame(cloud->sensor_origin_, cloud->sensor_orientation_);

    // Mouse and events call backs 
    CV.registerPointPickingCallback(pointPickingCallback, cloud);
    CV.registerKeyboardCallback(keyboardCallback);

    // Print results
    cout << "Points before downsampling: " << cloud->points.size() << endl;
    cout << "Points after downsampling: " << cloudFiltered->points.size() << endl;
    cout << ">> BOX COUNT" << ": " << num_plane - 1 << endl;
    cout << ">> SPHERE COUNT" << ": " << num_sphere << endl;

    // Save segmented point cloud to "output.pcd" in "build" directory
    saveCloud(cloud, "output.pcd");

    // Create visualization loop
    while(CV.isRunning()) {
        CV.spin(100);
    }

    return 0;
}
