#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/flare.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>

using KdTreePtr = pcl::search::KdTree<pcl::PointXYZ>::Ptr;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

PointCloudPtr cloud;
KdTreePtr tree;

//sampled surface for the computation of tangent X axis
PointCloudPtr sampled_cloud;
KdTreePtr sampled_tree;

int
  main (int argc, char** argv)
{
  cloud.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  
  // Read input mesh
  std::ifstream file("../../../ModelNet10/chair/test/chair_0890.off");  // Change this path to where your input file is
  if (!file.is_open()) {
    std::cout << "Mesh file wasn't read successfully." << std::endl;
    return 1;
  }

  char string1[4];
  file >> string1;

  // Read header.
  unsigned int numV = 0;
  unsigned int numP = 0;
  unsigned int numE = 0;
  file >> numV >> numP >> numE;

  cloud->width = numV;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->resize (cloud->width * cloud->height);

  // Write vertices to point cloud
  for (unsigned int i = 0; i < numV; i++) {
    file >> cloud->at(i).x;
    file >> cloud->at(i).y;
    file >> cloud->at(i).z;
  }

  tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  tree->setInputCloud (cloud);

  const float sampling_perc = 0.2f;
  const float sampling_incr = 1.0f / sampling_perc;
  sampled_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> ());

  std::vector<int> sampled_indices;
  for (float sa = 0.0f; sa < (float)cloud->size (); sa += sampling_incr)
    sampled_indices.push_back (static_cast<int> (sa));
  copyPointCloud (*cloud, sampled_indices, *sampled_cloud);

  sampled_tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  sampled_tree->setInputCloud (sampled_cloud);

  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::ReferenceFrame> mesh_LRF;

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  const float xRange = maxPt.x - minPt.x;
  const float yRange = maxPt.y - minPt.y;
  const float zRange = maxPt.z - minPt.z;

  // The following scales 0.005f by how much the range of this mesh differs than the range of https://github.com/PointCloudLibrary/pcl/blob/master/test/bun0.pcd
  // For computation details, see https://github.com/PointCloudLibrary/pcl/blob/master/test/features/test_flare_estimation.cpp
  const float mesh_res = 0.005f * (std::max(std::max(xRange, yRange), zRange) / 0.15);

  // Compute normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  ne.setRadiusSearch (2.0f*mesh_res);
  ne.setViewPoint (1, 1, 10);
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  ne.compute (*normals);

  // Compute FLARE LRF
  pcl::FLARELocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> lrf_estimator;

  lrf_estimator.setRadiusSearch (5 * mesh_res);
  lrf_estimator.setTangentRadius (20 * mesh_res);

  lrf_estimator.setInputCloud (cloud);
  lrf_estimator.setSearchSurface (cloud);
  lrf_estimator.setInputNormals (normals);
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setSearchMethodForSampledSurface (sampled_tree);
  lrf_estimator.setSearchSampledSurface (sampled_cloud);

  lrf_estimator.compute (mesh_LRF);

  std::ofstream lrf_file;
  // Write LRF for each point
  lrf_file.open ("../../lrf_file.txt");
  lrf_file << "orig_x orig_y orig_z x_0 x_1 x_2 y_0 y_1 y_2 z_0 z_1 z_2\n";
  for (int i = 0 ; i < cloud->size() ; i++) {
    lrf_file << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " ";
    lrf_file << mesh_LRF.at (i).x_axis[0] << " " << mesh_LRF.at (i).x_axis[1] << " " << mesh_LRF.at (i).x_axis[2] << " ";
    lrf_file << mesh_LRF.at (i).y_axis[0] << " " << mesh_LRF.at (i).y_axis[1] << " " << mesh_LRF.at (i).y_axis[2] << " ";
    lrf_file << mesh_LRF.at (i).z_axis[0] << " " << mesh_LRF.at (i).z_axis[1] << " " << mesh_LRF.at (i).z_axis[2] << "\n";
  }
  lrf_file.close();
  return 0;
}