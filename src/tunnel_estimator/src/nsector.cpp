#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/distances.h>
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
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>

char LEFT_VAILD = 0;
char RIGHT_VAILD = 0;
char TUNNEL_VAILD = 0;

using namespace Eigen;
using namespace std;
#define PI 3.14159265

typedef sensor_msgs::PointCloud2 PointCloud2;
typedef sensor_msgs::PointCloud PointCloud;
typedef geometry_msgs::PoseStamped PoseStamped;
typedef nav_msgs::Odometry Odometry;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

ros::Publisher pub_trans;
ros::Publisher pub_trans_after;
ros::Publisher pub_trans_outlier;
ros::Publisher pub_trans_left;
ros::Publisher pub_trans_right;


double vins_pitch, vins_roll, vins_yaw;
Vector4f vins_xyz(0,0,0,1);
Vector4f vins_pcV(0,0,0,1);
pcl::PointXYZ vins_xyz_pt(0,0,0);
Vector3f normal_vec_left(0, 0, 0);
Vector3f normal_vec_right(0, 0, 0);
Vector3f drone_vector(0,0,0);
VectorXf coeff_right;
VectorXf coeff_left;

void callback_vins(const Odometry::ConstPtr &vins)
{
  tf::Quaternion vins_qn(vins->pose.pose.orientation.x, vins->pose.pose.orientation.y, vins->pose.pose.orientation.z, vins->pose.pose.orientation.w);
  tf::Matrix3x3 m(vins_qn);
  m.getRPY(vins_roll, vins_pitch, vins_yaw);
  vins_xyz(0) = vins->pose.pose.position.x;
  vins_xyz(1) = vins->pose.pose.position.y;
  vins_xyz(2) = vins->pose.pose.position.z;

  vins_xyz_pt.x = vins_xyz(0);
  vins_xyz_pt.y = vins_xyz(1);
  vins_xyz_pt.z = vins_xyz(2);
  //https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
  //both vins's and odom's yaw increase in counter-clockwise direction  
  drone_vector(0) = cos(vins_yaw);
  drone_vector(1) = sin(vins_yaw);  
  drone_vector(2) = 0;

}

void callback_pc(const PointCloud::ConstPtr &msg)
{
  PointCloud2 vins_pc2;
  //Convert vins point cloud data to Pointcloud2 format
  sensor_msgs::convertPointCloudToPointCloud2(*msg, vins_pc2); 	
  //Convert vins point to pcl format
  PointCloudXYZ::Ptr vins_pcT (new PointCloudXYZ());
  pcl::fromROSMsg (vins_pc2, *vins_pcT);


  //http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough
  PointCloudXYZ::Ptr vins_pcT_pass (new PointCloudXYZ());
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (vins_pcT);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 4.0);
  // pass.setFilterLimitsNegative (true);
  pass.filter (*vins_pcT_pass);

  //filter the front points.
  pcl::PointIndices::Ptr front_inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (size_t i = 0; i < vins_pcT_pass->points.size (); ++i)
  {
    vins_pcV(0) = vins_pcT_pass->points[i].x;
    vins_pcV(1) = vins_pcT_pass->points[i].y;
    vins_pcV(2) = vins_pcT_pass->points[i].z;
    pcl::PointXYZ pt(vins_pcT_pass->points[i].x, vins_pcT_pass->points[i].y, vins_pcT_pass->points[i].z);
    if (pcl::distances::l2Sqr(vins_xyz, vins_pcV) < 4.0 && vins_pcT_pass->points[i].z > 0)
    {
      front_inliers->indices.push_back(i);
      // vins_pcT_pass->points[i].z = 0;
    }
  }
  extract.setInputCloud(vins_pcT_pass);
  extract.setIndices(front_inliers);
  extract.setNegative(true);
  extract.filter(*vins_pcT_pass);
  // pub_trans.publish(vins_pcT_pass);
  // pub_trans_after.publish(vins_pcT_rs);

  //http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
  PointCloudXYZ::Ptr vins_pcT_staf (new PointCloudXYZ());
  PointCloudXYZ::Ptr vins_pcT_sta_outliers (new PointCloudXYZ());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (vins_pcT_pass);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*vins_pcT_staf);
  // sor.setNegative (true);
  // sor.filter (*vins_pcT_sta_outliers);
  // pub_trans_after.publish(vins_pcT_f);
  // pub_trans_outlier.publish(vins_pcT_outliers);

  //http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers
  PointCloudXYZ::Ptr vins_pcT_radf (new PointCloudXYZ());
  PointCloudXYZ::Ptr vins_pcT_rad_outliers (new PointCloudXYZ());
  //qualified points can search at least n neighbors in Radius.
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter; // Initializing with true will allow us to extract the removed indices
  rorfilter.setInputCloud (vins_pcT_staf);
  rorfilter.setRadiusSearch (0.5);//Radius
  rorfilter.setMinNeighborsInRadius (1); //n neighbors
  rorfilter.filter (*vins_pcT_radf);
  rorfilter.setNegative (true);
  // rorfilter.filter (*vins_pcT_rad_outliers);
  // pub_trans_after.publish(vins_pcT_f);
  // pub_trans_outlier.publish(vins_pcT_outliers);

  //seperate to left and right
  PointCloudXYZ::Ptr vins_pcT_pass_left (new PointCloudXYZ());  //Y<0
  PointCloudXYZ::Ptr vins_pcT_pass_right (new PointCloudXYZ()); //Y>0
  pass.setInputCloud(vins_pcT_radf);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0, 10.0); 
  pass.filter (*vins_pcT_pass_right);

  pass.setInputCloud (vins_pcT_radf);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits(-10, 0);
  pass.filter(*vins_pcT_pass_left);

  // pub_trans_after.publish(vins_pcT_pass_left);
  // pub_trans_outlier.publish(vins_pcT_pass_right);

  // ROS_INFO_STREAM("we have %d points in left cloud" << vins_pcT_pass_left->points.size ());
  // ROS_INFO_STREAM("we have %d points in right cloud" << vins_pcT_pass_right->points.size ());

  //pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
  //left
  if(vins_pcT_pass_left->points.size() > 50)
  {
    LEFT_VAILD = 1;
    vector<int> inliers_left;
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr model_p_left (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (vins_pcT_pass_left));
    pcl::PointCloud<pcl::PointXYZ>::Ptr vins_pcT_rs_left (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_left (model_p_left);
    model_p_left->setAxis (Vector3f (0.0, 1.0, 0.0));
    model_p_left->setEpsAngle (0.15);
    ransac_left.setDistanceThreshold (.10);
    ransac_left.computeModel();
    ransac_left.getInliers(inliers_left);
    ransac_left.getModelCoefficients (coeff_left);
    pcl::copyPointCloud<pcl::PointXYZ>(*vins_pcT_pass_left, inliers_left, *vins_pcT_rs_left);
    // ROS_INFO_STREAM(coeff_left);
    normal_vec_left(0) = coeff_left(0);
    normal_vec_left(1) = coeff_left(1);
    normal_vec_left(2) = coeff_left(2);
  }
  else
  {
    LEFT_VAILD = 0;
  }
  //right
  if(vins_pcT_pass_right->points.size() > 50)
  {
    RIGHT_VAILD = 1;
    vector<int> inliers_right;
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr model_p_right (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (vins_pcT_pass_right));
    pcl::PointCloud<pcl::PointXYZ>::Ptr vins_pcT_rs_right (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_right (model_p_right);
    model_p_right->setAxis (Vector3f (0.0, 1.0, 0.0));
    model_p_right->setEpsAngle (0.15);
    ransac_right.setDistanceThreshold (.10);
    ransac_right.computeModel();
    ransac_right.getInliers(inliers_right);
    ransac_right.getModelCoefficients (coeff_right);
    pcl::copyPointCloud<pcl::PointXYZ>(*vins_pcT_pass_right, inliers_right, *vins_pcT_rs_right);
    // ROS_INFO_STREAM(coeff_right);
    normal_vec_right(0) = coeff_right(0);
    normal_vec_right(1) = coeff_right(1);
    normal_vec_right(2) = coeff_right(2);
  }
  else
  {
    RIGHT_VAILD = 0;
  }

  if(LEFT_VAILD && RIGHT_VAILD)
  {
    TUNNEL_VAILD = 1;
  }
  else
  {
    TUNNEL_VAILD = 0;
  }
  // pub_trans_left.publish(vins_pcT_rs_left);
  // pub_trans_right.publish(vins_pcT_rs_right);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nsector");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("vins_estimator/point_cloud", 1, callback_pc);

  ros::Subscriber sub_vins = nh.subscribe<Odometry>("vins_estimator/odometry", 1, callback_vins);

  pub_trans = nh.advertise<PointCloudXYZ>("nsector/est", 1);
  pub_trans_after = nh.advertise<PointCloudXYZ>("nsector/after", 1);
  pub_trans_outlier = nh.advertise<PointCloudXYZ>("nsector/outlier", 1);
  pub_trans_left = nh.advertise<PointCloudXYZ>("nsector/left", 1);
  pub_trans_right = nh.advertise<PointCloudXYZ>("nsector/right", 1);


  ros::Rate loop_rate(20);
  while(ros::ok())
  {

    if(TUNNEL_VAILD)
    {
      //https://www.vitutor.com/geometry/distance/line_plane.html
      abs(drone_vector.array() * normal_vec_left.array()) / abs(drone_vector.norm() * normal_vec_left.norm());
      abs(drone_vector.array() * normal_vec_right.array()) / abs(drone_vector.norm() * normal_vec_right.norm());
      pcl::pointToPlaneDistance(vins_xyz_pt, coeff_right);
      pcl::pointToPlaneDistance(vins_xyz_pt, coeff_left);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
