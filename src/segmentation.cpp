#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "sara_msgs/BoundingBoxes3D.h"
#include "visualization_msgs/Marker.h"

#include <dynamic_reconfigure/server.h>
#include "wm_object_segmentation/wm_object_segmentationConfig.h"

bool _PUBLISH_MARKERS{true};

ros::Publisher pub;
ros::Publisher pub_BB3D;
ros::Publisher pub_markers;
float lim_size_gripper;
float lim_max_size_object;
float lim_distance_object;

void callback(wm_object_segmentation::wm_object_segmentationConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f",
             config.lim_size_gripper,
             config.lim_max_size_object,
             config.lim_distance_object);

    lim_size_gripper = config.lim_size_gripper;
    lim_max_size_object = config.lim_max_size_object;
    lim_distance_object = config.lim_distance_object;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

   if(ros::param::has("/process_object_segmentation"))
   {
       ROS_INFO("process_object_segmentation EXIST");
       bool rosParamProcess;
       //ros::param::get("/process_object_segmentation", rosParamProcess);

       ros::param::get("/process_object_segmentation", rosParamProcess)

       ROS_INFO_STREAM("process_object_segmentation : " << rosParamProcess);
       if(!rosParamProcess)
       {
           ROS_INFO("process_object_segmentation IS FALSE");
           return;
       }
       else
           ROS_INFO("process_object_segmentation IS TRUE");
   }

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.02, 0.02, 0.02);
  sor.filter (cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2( cloud_filtered, point_cloud);
  pcl::copyPointCloud(point_cloud, *point_cloudPtr);

// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(point_cloudPtr);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.05); // 1cm
	ec.setMinClusterSize(50); //50
	ec.setMaxClusterSize(99000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(point_cloudPtr);
	ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

    //object to store the data to publish the BoundingBoxes3D
    sara_msgs::BoundingBoxes3D box_list;
    box_list.header.stamp = ros::Time::now();
    box_list.header.frame_id = "/base_link";
    sara_msgs::BoundingBox3D boundingBox;
    boundingBox.Class = "";
    boundingBox.probability = 1;
    bool firstValue = true;
    float xmin, xmax, ymin, ymax, zmin, zmax;
    float xmoy, ymoy, zmoy;

  int j= 0;
 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
          pcl::PointXYZRGB point;
          point.x = point_cloudPtr->points[*pit].x;
          point.y = point_cloudPtr->points[*pit].y;
          point.z = point_cloudPtr->points[*pit].z;

          if (j == 0) //Red	#FF0000	(255,0,0)
          {
              point.r = 0;
              point.g = 0;
              point.b = 255;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }

          } else if (j == 1) //Lime	#00FF00	(0,255,0)
          {
              point.r = 0;
              point.g = 255;
              point.b = 0;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else if (j == 2) // Blue	#0000FF	(0,0,255)
          {
              point.r = 255;
              point.g = 0;
              point.b = 0;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else if (j == 3) // Yellow	#FFFF00	(255,255,0)
          {
              point.r = 255;
              point.g = 255;
              point.b = 0;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else if (j == 4) //Cyan	#00FFFF	(0,255,255)
          {
              point.r = 0;
              point.g = 255;
              point.b = 255;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else if (j == 5) // Magenta	#FF00FF	(255,0,255)
          {
              point.r = 255;
              point.g = 0;
              point.b = 255;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else if (j == 6) // Olive	#808000	(128,128,0)
          {
              point.r = 128;
              point.g = 128;
              point.b = 0;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else if (j == 7) // Teal	#008080	(0,128,128)
          {
              point.r = 0;
              point.g = 128;
              point.b = 128;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else if (j == 8) // Purple	#800080	(128,0,128)
          {
              point.r = 128;
              point.g = 0;
              point.b = 128;

              if (firstValue) {
                  firstValue = false;
                  xmin = point.x;
                  xmax = point.x;
                  ymin = point.y;
                  ymax = point.y;
                  zmin = point.z;
                  zmax = point.z;
              } else {
                  if (xmin > point.x)
                      xmin = point.x;
                  if (xmax < point.x)
                      xmax = point.x;
                  if (ymin > point.y)
                      ymin = point.y;
                  if (ymax < point.y)
                      ymax = point.y;
                  if (zmin > point.z)
                      zmin = point.z;
                  if (zmax < point.z)
                      zmax = point.z;
              }
          } else {
              if (j % 2 == 0) {
                  point.r = 255 * j / (cluster_indices.size());
                  point.g = 128;
                  point.b = 50;

                  if (firstValue) {
                      firstValue = false;
                      xmin = point.x;
                      xmax = point.x;
                      ymin = point.y;
                      ymax = point.y;
                      zmin = point.z;
                      zmax = point.z;
                  } else {
                      if (xmin > point.x)
                          xmin = point.x;
                      if (xmax < point.x)
                          xmax = point.x;
                      if (ymin > point.y)
                          ymin = point.y;
                      if (ymax < point.y)
                          ymax = point.y;
                      if (zmin > point.z)
                          zmin = point.z;
                      if (zmax < point.z)
                          zmax = point.z;
                  }
              } else {
                  point.r = 0;
                  point.g = 255 * j / (cluster_indices.size());
                  point.b = 128;

                  if (firstValue) {
                      firstValue = false;
                      xmin = point.x;
                      xmax = point.x;
                      ymin = point.y;
                      ymax = point.y;
                      zmin = point.z;
                      zmax = point.z;
                  } else {
                      if (xmin > point.x)
                          xmin = point.x;
                      if (xmax < point.x)
                          xmax = point.x;
                      if (ymin > point.y)
                          ymin = point.y;
                      if (ymax < point.y)
                          ymax = point.y;
                      if (zmin > point.z)
                          zmin = point.z;
                      if (zmax < point.z)
                          zmax = point.z;
                  }
              }
          }
          point_cloud_segmented->push_back(point);
      }
      firstValue = true;
      xmoy = (xmin + xmax) / 2.0;
      ymoy = (ymin + ymax) / 2.0;
      zmoy = (zmin + zmax) / 2.0;


      boundingBox.Center.x = xmoy;
      boundingBox.Center.y = ymoy;
      boundingBox.Center.z = zmoy;
      boundingBox.Depth = xmax - xmin;
      boundingBox.Width = ymax - ymin;
      boundingBox.Height = zmax - zmin;
      
      if ( (boundingBox.Depth < lim_size_gripper || boundingBox.Width < lim_size_gripper) && boundingBox.Depth < lim_max_size_object && boundingBox.Width < lim_max_size_object && boundingBox.Center.x < lim_distance_object)
      {

        box_list.boundingBoxes.push_back(boundingBox);

         /*** Publish the boxes ***/
         if (_PUBLISH_MARKERS){  // Publish visual box
             visualization_msgs::Marker m;
             m.header.stamp = box_list.header.stamp;
             m.lifetime = ros::Duration(0.3);
             m.header.frame_id = box_list.header.frame_id;
             m.ns = boundingBox.Class;
             m.id = ros::Time::now().toNSec()+int(boundingBox.probability*1000);
             m.type = m.CUBE;
             m.pose.position.x = boundingBox.Center.x;
             m.pose.position.y = boundingBox.Center.y;
             m.pose.position.z = boundingBox.Center.z;
             m.scale.x = boundingBox.Depth;
             m.scale.y = boundingBox.Width;
             m.scale.z = boundingBox.Height;
             m.color.r = 0;
             m.color.g = 1;
             m.color.b = 0;
             m.color.a = 0.2;
             pub_markers.publish(m);
         }
      }

      j++;
    }
  pub_BB3D.publish(box_list);

  std::cerr<< "segemnted:  " << (int)point_cloud_segmented->size() << "\n";
  std::cerr<< "origin:     " << (int)point_cloudPtr->size() << "\n";
  // Convert to ROS data type
  point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;
  if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
  else pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "wm_object_segmentation");

  ros::NodeHandle nh;

   nh.param("publish_markers", _PUBLISH_MARKERS, true);

   // Configure the dynamic reconfigure thigny
   dynamic_reconfigure::Server<wm_object_segmentation::wm_object_segmentationConfig> server;
   dynamic_reconfigure::Server<wm_object_segmentation::wm_object_segmentationConfig>::CallbackType f;
   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/segment_table/nonplane", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  pub_BB3D = nh.advertise<sara_msgs::BoundingBoxes3D>("unknown_objects", 100);

  pub_markers = nh.advertise<visualization_msgs::Marker>("/boxes", 100);

  // Spin
  ros::spin ();
}
