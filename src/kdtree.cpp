#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>


static ros::Publisher PubOutput;
static int ExampleNumber;

void tf_broadcast(const std::string frame_id){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_depth_optical_frame";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = 2.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}


void kdTree(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // How to use a KdTree to search
    // Ref: http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // Use pcl::PointXYZRGB to visualize segmentation.
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    kdtree.setInputCloud (cloud.makeShared());

    pcl::PointXYZRGB searchPoint;

    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

    // K nearest neighbor search

    /*
    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }
    */

    // Neighbors within radius search

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

    float radius = 50;

    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
            cloud.points[ pointIdxRadiusSearch[i] ].r = 255;
            cloud.points[ pointIdxRadiusSearch[i] ].g = 0;
            cloud.points[ pointIdxRadiusSearch[i] ].b = 0;

        }
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = frame_id;
    // // Publish the data
    PubOutput.publish(output);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    const static std::string EXAMPLE_FRAME_ID = "example_frame";

    switch(ExampleNumber){
    case 0:
        kdTree(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    default:
        break;
    }

    // to shift positions of rendering point clouds
    tf_broadcast(EXAMPLE_FRAME_ID);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "example_kdtree");
    ros::NodeHandle nh("~");

    nh.param<int>("number", ExampleNumber, 0);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    PubOutput = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);

    ros::spin ();
}
