#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>


using namespace Eigen;
using namespace std;

typedef sensor_msgs::PointCloud2 PointCloud2;
typedef sensor_msgs::PointCloud PointCloud;
typedef geometry_msgs::PoseStamped PoseStamped;
typedef nav_msgs::Odometry Odometry;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

ros::Publisher pub_trans;
ros::Publisher pub_trans_after;
ros::Publisher pub_trans_outlier;



void callback_pc(const PointCloud::ConstPtr &msg)
{
  PointCloud2 vins_pc2;
  //Convert vins point cloud data to Pointcloud2 format
  sensor_msgs::convertPointCloudToPointCloud2(*msg, vins_pc2); 	
  //Convert vins point to pcl format
  PointCloudXYZ::Ptr vins_pcT (new PointCloudXYZ());
  pcl::fromROSMsg (vins_pc2, *vins_pcT);


  //http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
  //Creat a new cloud to fill the filtered data.
  PointCloudXYZ::Ptr vins_pcT_staf (new PointCloudXYZ());
  PointCloudXYZ::Ptr vins_pcT_sta_outliers (new PointCloudXYZ());
  //create statistics removal object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (vins_pcT);
  sor.setMeanK (150);
  sor.setStddevMulThresh (1.0);
  sor.filter (*vins_pcT_staf);
  sor.setNegative (true);
  sor.filter (*vins_pcT_sta_outliers);
  // pub_trans_after.publish(vins_pcT_f);
  // pub_trans_outlier.publish(vins_pcT_outliers);

  //http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers
  PointCloudXYZ::Ptr vins_pcT_radf (new PointCloudXYZ());
  PointCloudXYZ::Ptr vins_pcT_rad_outliers (new PointCloudXYZ());
  //qualified points can search at least n neighbors in Radius.
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter; // Initializing with true will allow us to extract the removed indices
  rorfilter.setInputCloud (vins_pcT_staf);
  rorfilter.setRadiusSearch (0.4);//Radius
  rorfilter.setMinNeighborsInRadius (1); //n neighbors
  rorfilter.filter (*vins_pcT_radf);
  rorfilter.setNegative (true);
  // rorfilter.filter (*vins_pcT_rad_outliers);
  // pub_trans_after.publish(vins_pcT_f);
  // pub_trans_outlier.publish(vins_pcT_outliers);

  //http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough
  PointCloudXYZ::Ptr vins_pcT_pass (new PointCloudXYZ());
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (vins_pcT_radf);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 4.0);
  // pass.setFilterLimitsNegative (true);
  pass.filter (*vins_pcT_pass);
  // pub_trans_after.publish(vins_pcT_f);
  // pub_trans_outlier.publish(vins_pcT_pass);


  //pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
  std::vector<int> inliers;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (vins_pcT_pass));
  pcl::PointCloud<pcl::PointXYZ>::Ptr vins_pcT_rs (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
  ransac.setDistanceThreshold (.10);
  ransac.computeModel();
  ransac.getInliers(inliers);

  pcl::copyPointCloud<pcl::PointXYZ>(*vins_pcT_pass, inliers, *vins_pcT_rs);
  // pub_trans_after.publish(vins_pcT_rs);

  // for (size_t i = 0; i < vins_pcT_pass->points.size (); ++i)
  // {
  //   vins_pcT_pass->points[i].z = 0;
  // }
  pub_trans_after.publish(vins_pcT_rs);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nsector");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("vins_estimator/point_cloud", 1, callback_pc);

  // ros::Subscriber sub_odom = nh.subscribe<Odometry>("mavros/local_position/odom", 1, callback_odom);

  pub_trans = nh.advertise<PointCloudXYZ>("nsector/est", 1);
  pub_trans_after = nh.advertise<PointCloudXYZ>("nsector/after", 1);
  pub_trans_outlier = nh.advertise<PointCloudXYZ>("nsector/outlier", 1);

  ros::spin();
}
