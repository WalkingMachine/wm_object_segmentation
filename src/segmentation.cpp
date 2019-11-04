#include <ros/ros.h>
#include <math.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
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
#include "sara_msgs/PointClouds.h"
#include <limits>

bool _PUBLISH_MARKERS{true};

ros::Publisher pub_markers;
ros::Publisher pub_objects_pointclouds;
ros::Publisher pub_pointclouds;
float lim_size_gripper;
float lim_max_size_object;
float lim_distance_object;
geometry_msgs::Pose robot_pose;

void callback(wm_object_segmentation::wm_object_segmentationConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f",
             config.lim_size_gripper,
             config.lim_max_size_object,
             config.lim_distance_object);

    lim_size_gripper = config.lim_size_gripper;
    lim_max_size_object = config.lim_max_size_object;
    lim_distance_object = config.lim_distance_object;
}

void pose_cb(const geometry_msgs::Pose _robot_pose){
    robot_pose = _robot_pose;
}

float distance(geometry_msgs::Point a,geometry_msgs::Point b){
    return sqrt( pow(b.x-a.x, 2) + pow(b.y-a.y, 2) + pow(b.z-a.z, 2) );
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    if (ros::param::has("/process_object_segmentation")) {
        bool rosParamProcess;
        //ros::param::get("/process_object_segmentation", rosParamProcess);

        ros::param::get("/process_object_segmentation", rosParamProcess);

        ROS_INFO_STREAM("process_object_segmentation : " << rosParamProcess);
        if (!rosParamProcess) {
            ROS_INFO("process_object_segmentation IS FALSE");
            return;
        }
    }

    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.02, 0.02, 0.02);
    sor.filter(cloud_filtered);

    pcl::PointCloud <pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_filtered, point_cloud);
    pcl::copyPointCloud(point_cloud, *point_cloudPtr);

// Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
    tree->setInputCloud(point_cloudPtr);

    std::vector <pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction <pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05); // 1cm
    ec.setMinClusterSize(50); //50
    ec.setMaxClusterSize(99000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(point_cloudPtr);
    ec.extract(cluster_indices);


    //object to store the data to publish the BoundingBoxes3D
    sara_msgs::BoundingBoxes3D box_list;
    box_list.header = cloud_msg->header;
    sara_msgs::BoundingBox3D boundingBox;
    boundingBox.Class = "";
    boundingBox.probability = 1;


    int j = 0;

    sara_msgs::PointClouds msgPointClouds;
    msgPointClouds.header = cloud_msg->header;


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {

        pcl::PointCloud<pcl::PointXYZRGB> msgPointCloud;

        // Initialize limits
        float xmin{std::numeric_limits<float>::max()};
        float ymin{xmin};
        float zmin{ymin};

        float xmax{-std::numeric_limits<float>::max()};
        float ymax{xmax};
        float zmax{ymax};

        float xmoy, ymoy, zmoy;

        // Initiate the points
        pcl::PointXYZRGB point;

        // Calculate the points color
        float h{float(sin(j))};
        point.r = int(h*255);
        point.g = int((1-h)*255);
        point.b = 255-abs(int((h-0.5f)*510)) ;

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

            point.x = point_cloudPtr->points[*pit].x;
            point.y = point_cloudPtr->points[*pit].y;
            point.z = point_cloudPtr->points[*pit].z;

            // Adjust limits
            if (xmin > point.x) xmin = point.x;
            if (xmax < point.x) xmax = point.x;
            if (ymin > point.y) ymin = point.y;
            if (ymax < point.y) ymax = point.y;
            if (zmin > point.z) zmin = point.z;
            if (zmax < point.z) zmax = point.z;

            msgPointCloud.push_back(point);
        }

        // Calculate the dimentions of the box
        xmoy = (xmin + xmax) / 2.0;
        ymoy = (ymin + ymax) / 2.0;
        zmoy = (zmin + zmax) / 2.0;

        boundingBox.Center.x = xmoy;
        boundingBox.Center.y = ymoy;
        boundingBox.Center.z = zmoy;
        boundingBox.Depth = xmax - xmin;
        boundingBox.Width = ymax - ymin;
        boundingBox.Height = zmax - zmin;

        if ((boundingBox.Depth < lim_size_gripper || boundingBox.Width < lim_size_gripper) &&
            boundingBox.Depth < lim_max_size_object && boundingBox.Width < lim_max_size_object &&
            distance(robot_pose.position, boundingBox.Center) < lim_distance_object) {

            box_list.boundingBoxes.push_back(boundingBox);

            /*** Publish the visual box ***/
            if (_PUBLISH_MARKERS) {  // Publish visual box
                visualization_msgs::Marker m;
                m.header = cloud_msg->header;
                m.lifetime = ros::Duration(0.3);
                m.ns = boundingBox.Class;
                m.id = ros::Time::now().toNSec() + int(boundingBox.probability * 1000);
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


            // Publish the segmented pointcloud
            sara_msgs::PointCloud output;
            pcl::PCLPointCloud2 temp;

            pcl::toPCLPointCloud2(msgPointCloud, temp);
            pcl_conversions::fromPCL(temp, output.pointCloud);
            output.pointCloud.header = cloud_msg->header;
            pub_pointclouds.publish(output.pointCloud);
            output.boundingBox = boundingBox;

            msgPointClouds.pointClouds.push_back(output);

        }

        j++;
    }

    // Publish the list of boxes
    pub_objects_pointclouds.publish(msgPointClouds);
}

int
main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "wm_object_segmentation");

    ros::NodeHandle nh;

    nh.param("publish_markers", _PUBLISH_MARKERS, true);

    // Configure the dynamic reconfigure thigny
    dynamic_reconfigure::Server <wm_object_segmentation::wm_object_segmentationConfig> server;
    dynamic_reconfigure::Server<wm_object_segmentation::wm_object_segmentationConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    // Create a ROS subscriber for the input point cloud and robot_pose
    ros::Subscriber sub = nh.subscribe("/segment_table/nonplane", 1, cloud_cb);
    ros::Subscriber sub_pose = nh.subscribe("/robot_pose", 1, pose_cb);

    // Create the ROS publishers
    pub_markers = nh.advertise<visualization_msgs::Marker>("/boxes", 100);
    pub_objects_pointclouds = nh.advertise<sara_msgs::PointClouds>("/unknown_objects/segmented_pointclouds/listed", 100);
    pub_pointclouds = nh.advertise<sensor_msgs::PointCloud2>("/unknown_objects/segmented_pointclouds/individual", 100);

    // Spin
    ros::spin();
}
