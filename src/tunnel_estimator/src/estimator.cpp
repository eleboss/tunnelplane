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
#include <pcl/common/distances.h>
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
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"

float x_trans, y_trans, z_trans;

char LEFT_VAILD = 0;
char RIGHT_VAILD = 0;
char TUNNEL_VAILD = 0;

double PASSTHROUGH_ZMIN = 0.2;
double PASSTHROUGH_ZMAX = 4;
double FRONTFILTER_RAD = 4;
double STA_MEANK = 50;
double STA_STD_THRESH = 1.0;
double ROR_RAD = 0.5;
double ROS_NER = 1;
double TUNNEL_WIDTH = 4.2;
double RANSAC_PLANE_ANG_TOR = 0.05;
double RANSAC_DIS_THR = 0.05;
double RANSAC_VAILD_NUM =30;
double VAILD_ANG_TOR = 0.05;
double VAILD_DIS_TOR = 999;
double VIZ = 0;

float vaild_angle = 0;
float centric_distance = 0;
float last_vaild_distance = TUNNEL_WIDTH;


using namespace Eigen;
using namespace std;
#define PI 3.14159265

typedef sensor_msgs::PointCloud2 PointCloud2;
typedef sensor_msgs::PointCloud PointCloud;
typedef sensor_msgs::LaserScan LaserScan;
typedef geometry_msgs::PoseStamped PoseStamped;
typedef nav_msgs::Odometry Odometry;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

ros::Publisher pub_trans;
ros::Publisher pub_trans_after;
ros::Publisher pub_trans_outlier;
ros::Publisher pub_trans_left;
ros::Publisher pub_trans_right;
ros::Publisher pub_estimation;
laser_geometry::LaserProjection projector_;

double angle_left, angle_right, distance_left, distance_right;

double vins_pitch, vins_roll, vins_yaw;
Vector4f vins_xyz(0,0,0,1);
Vector4f vins_pcV(0,0,0,1);
pcl::PointXYZ vins_xyz_pt(0,0,0);
Vector4f line_vec_left(0, 0, 0, 1);
Vector4f line_vec_right(0, 0, 0, 1);
Vector3f drone_vector(0,0,0);
VectorXf coeff_right;
VectorXf coeff_left;
Vector4f pointinline_left(0,0,0,1);
Vector4f pointinline_right(0,0,0,1);

double line_plane_angle(Eigen::Vector3f line_vector, Eigen::Vector3f plane_norm)
{
  return asin((line_vector.array() * plane_norm.array()).sum() / line_vector.norm() * plane_norm.norm());
}
double two_vector_angle(Eigen::Vector3f vector1, Eigen::Vector3f vector2)
{
  return acos((vector1.array() * vector2.array()).sum() / vector1.norm() * vector2.norm());
}

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

void callback_scan(const LaserScan::ConstPtr &scan)
{

  PointCloud2 scan_pc2;
  //convert laserscan to pointcloud
  projector_.projectLaser(*scan, scan_pc2);
  Odometry tunnel_est;
  //Convert scan point to pcl format
  PointCloudXYZ::Ptr scan_pcT (new PointCloudXYZ());
  pcl::fromROSMsg (scan_pc2, *scan_pcT);
  // pub_trans_after.publish(scan_pcT);

  //创建旋转矩阵，这里的旋转的角度根据IMU给出
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // Define a translation
  transform.translation() << x_trans, y_trans, z_trans;
  // The same rotation matrix as before; theta radians around Z axis
  transform.rotate(Eigen::AngleAxisf(vins_roll, Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(vins_pitch, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(vins_yaw, Eigen::Vector3f::UnitZ()));
  // Print the transformation
  // printf("\nMethod #2: using an Affine3f\n");
  // std::cout << transform_2.matrix() << std::endl;
  // Executing the transformation
  pcl::transformPointCloud(*scan_pcT, *scan_pcT, transform);

  //http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
  PointCloudXYZ::Ptr scan_pcT_staf (new PointCloudXYZ());
  PointCloudXYZ::Ptr scan_pcT_sta_outliers (new PointCloudXYZ());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (scan_pcT);
  sor.setMeanK (STA_MEANK);
  sor.setStddevMulThresh (STA_STD_THRESH);
  sor.filter (*scan_pcT_staf);
  // sor.setNegative (true);
  // sor.filter (*scan_pcT_sta_outliers);
  // pub_trans_after.publish(scan_pcT_f);
  // pub_trans_outlier.publish(scan_pcT_outliers);


  //seperate to left and right
  pcl::PassThrough<pcl::PointXYZ> pass;
  PointCloudXYZ::Ptr scan_pcT_pass_left (new PointCloudXYZ());  //Y<0
  PointCloudXYZ::Ptr scan_pcT_pass_right (new PointCloudXYZ()); //Y>0
  pass.setInputCloud(scan_pcT_staf);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-TUNNEL_WIDTH, 0.0); 
  pass.filter (*scan_pcT_pass_right);

  pass.setInputCloud (scan_pcT_staf);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits(0.0, TUNNEL_WIDTH);
  pass.filter(*scan_pcT_pass_left);


  // ROS_INFO_STREAM("we have %d points in left cloud" << scan_pcT_pass_left->points.size ());
  // ROS_INFO_STREAM("we have %d points in right cloud" << scan_pcT_pass_right->points.size ());

  //pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
  //left
  if(scan_pcT_pass_left->points.size() > RANSAC_VAILD_NUM)
  {
    LEFT_VAILD = 1;
    vector<int> inliers_left;
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l_left (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (scan_pcT_pass_left));
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pcT_rs_left (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_left (model_l_left);

    ransac_left.setDistanceThreshold (RANSAC_DIS_THR);
    ransac_left.computeModel();
    ransac_left.getInliers(inliers_left);
    ransac_left.getModelCoefficients (coeff_left);
    pcl::copyPointCloud<pcl::PointXYZ>(*scan_pcT_pass_left, inliers_left, *scan_pcT_rs_left);
    ROS_INFO_STREAM(coeff_left);
    line_vec_left(0) = coeff_left(3);
    line_vec_left(1) = coeff_left(4);
    line_vec_left(2) = coeff_left(5);


    //if the angle between vector(scan to point) and normal of plane are large than PI/2, we re direct the normal to point inside of the tunnel
    if(two_vector_angle(vins_xyz.head(3), line_vec_left.head(3)) > 1.5707963)
    {
      line_vec_left = - line_vec_left;
    }
    if(VIZ)
      pub_trans_left.publish(scan_pcT_rs_left);
  }
  else
  {
    LEFT_VAILD = 0;
  }

  if(scan_pcT_pass_right->points.size() > RANSAC_VAILD_NUM)
  {
    RIGHT_VAILD = 1;
    vector<int> inliers_right;
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l_right (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (scan_pcT_pass_right));
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pcT_rs_right (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_right (model_l_right);

    ransac_right.setDistanceThreshold (RANSAC_DIS_THR);
    ransac_right.computeModel();
    ransac_right.getInliers(inliers_right);
    ransac_right.getModelCoefficients (coeff_right);
    pcl::copyPointCloud<pcl::PointXYZ>(*scan_pcT_pass_right, inliers_right, *scan_pcT_rs_right);
    ROS_INFO_STREAM(coeff_right);
    line_vec_right(0) = coeff_right(3);
    line_vec_right(1) = coeff_right(4);
    line_vec_right(2) = coeff_right(5);


    //if the angle between vector(scan to point) and normal of plane are large than PI/2, we re direct the normal to point inside of the tunnel
    if(two_vector_angle(vins_xyz.head(3), line_vec_right.head(3)) > 1.5707963)
    {
      line_vec_right.head(3) = - line_vec_right.head(3);
    }
    if(VIZ)
      pub_trans_right.publish(scan_pcT_rs_right);
  }
  else
  {
    RIGHT_VAILD = 0;
  }


    if(LEFT_VAILD)
    {

      pointinline_left(0) = coeff_left(0);
      pointinline_left(1) = coeff_left(1);
      pointinline_left(2) = coeff_left(2);
      //https://www.vitutor.com/geometry/distance/line_plane.html
      angle_left = line_plane_angle(drone_vector, line_vec_left.head(3));
      distance_left = pcl::sqrPointToLineDistance(vins_xyz,pointinline_left , line_vec_left);
      // ROS_INFO_STREAM("nor left"<<line_vec_left);
    }
    if(RIGHT_VAILD)
    {
      pointinline_right(0) = coeff_right(0);
      pointinline_right(1) = coeff_right(1);
      pointinline_right(2) = coeff_right(2);
      //https://www.vitutor.com/geometry/distance/line_plane.html
      angle_right = line_plane_angle(drone_vector, line_vec_right.head(3));
      distance_right = pcl::sqrPointToLineDistance(vins_xyz,pointinline_left , line_vec_right);
      // ROS_INFO_STREAM("nor right"<<line_vec_right);
    }
    if(LEFT_VAILD && RIGHT_VAILD)
    {
      if (abs(angle_left - angle_right) <= VAILD_ANG_TOR)
      {
        if(abs((distance_left + distance_right) - TUNNEL_WIDTH) <= VAILD_DIS_TOR)
        {
          TUNNEL_VAILD = 1;
          vaild_angle = (angle_left + angle_right)/2;
          centric_distance = distance_left - distance_right;
          // ROS_INFO_STREAM(vaild_angle<<" "<< centric_distance);
          ROS_INFO_STREAM("angle "<<vaild_angle<<" distance "<<centric_distance<<"left angle:"<<angle_left<<"right angle"<<angle_right);
        }
        else
        {
          TUNNEL_VAILD = 0;
        }
      }
      else
      {
        TUNNEL_VAILD = 0;
      }
    }
    else
    {
      TUNNEL_VAILD = 0;
    }
    tunnel_est.pose.pose.position.x = vaild_angle;
    tunnel_est.pose.pose.position.y = centric_distance;
    tunnel_est.pose.pose.position.z = TUNNEL_VAILD;
    tunnel_est.pose.pose.orientation.x = LEFT_VAILD;
    tunnel_est.pose.pose.orientation.y = angle_left;
    tunnel_est.pose.pose.orientation.z = RIGHT_VAILD;
    tunnel_est.pose.pose.orientation.w = angle_right;
    
    pub_estimation.publish(tunnel_est);
}



void callback_pc(const PointCloud::ConstPtr &msg)
{
  ;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~"); // to get the private params

  _nh.param("PASSTHROUGH_ZMIN", PASSTHROUGH_ZMIN, 0.2);
  ROS_INFO_STREAM("Min height of pass through filter, used to filter the ground points to find the wall " << PASSTHROUGH_ZMIN);

  _nh.param("PASSTHROUGH_ZMAX", PASSTHROUGH_ZMAX, 4.0);
  ROS_INFO_STREAM("Max height of pass through filter, used to filter the ground points to find the wall " << PASSTHROUGH_ZMAX);

  _nh.param("FRONTFILTER_RAD", FRONTFILTER_RAD, 4.0);
  ROS_INFO_STREAM("Filter the front points, radius:" << FRONTFILTER_RAD<<" meter");

  _nh.param("STA_MEANK", STA_MEANK, 50.0);
  ROS_INFO_STREAM("statistics filter, compare with " << STA_MEANK<<" points");

  _nh.param("STA_STD_THRESH", STA_STD_THRESH, 50.0);
  ROS_INFO_STREAM("std of statistics filter" << STA_STD_THRESH);

  _nh.param("ROR_RAD", ROR_RAD, 0.5);
  ROS_INFO_STREAM("RAD filter, Find points in " << ROR_RAD <<" meters");

  _nh.param("ROS_NER", ROS_NER, 1.0);
  ROS_INFO_STREAM("RAF filter vaild point can find " << ROS_NER <<" neighboor in "<<ROR_RAD<<"meter");

  _nh.param("TUNNEL_WIDTH", TUNNEL_WIDTH, 10.0);
  ROS_INFO_STREAM("We assume the tunnel is wider than " << TUNNEL_WIDTH <<"meter");

  _nh.param("RANSAC_PLANE_ANG_TOR", RANSAC_PLANE_ANG_TOR, 0.05);
  ROS_INFO_STREAM("Angle tor of ransac:" << RANSAC_PLANE_ANG_TOR <<"rad");

  _nh.param("RANSAC_DIS_THR", RANSAC_DIS_THR, 0.15);
  ROS_INFO_STREAM("Dis tor of ransac:" << RANSAC_DIS_THR <<"meter");

  _nh.param("RANSAC_VAILD_NUM", RANSAC_VAILD_NUM, 30.0);
  ROS_INFO_STREAM("Vaild tunnel direction estimation needs at least " << RANSAC_VAILD_NUM <<"points");

  _nh.param("VIZ", VIZ, 0.0);
  ROS_INFO_STREAM("Visualize "<< VIZ);

  ros::Subscriber sub = nh.subscribe<PointCloud>("vins_estimator/point_cloud", 1, callback_pc);
  ros::Subscriber sub_vins = nh.subscribe<Odometry>("vins_estimator/odometry", 1, callback_vins);
  ros::Subscriber sub_scan = nh.subscribe<LaserScan>("scan_filtered", 1, callback_scan);

  // pub_trans = nh.advertise<PointCloudXYZ>("estimator/est", 1);
  pub_trans_after = nh.advertise<PointCloudXYZ>("estimator/filtered", 1);
  // pub_trans_outlier = nh.advertise<PointCloudXYZ>("estimator/outlier", 1);
  pub_trans_left = nh.advertise<PointCloudXYZ>("estimator/left", 1);
  pub_trans_right = nh.advertise<PointCloudXYZ>("estimator/right", 1);

  pub_estimation = nh.advertise<Odometry> ("estimator/estimator", 1);


  ros::Rate loop_rate(20);
  while(ros::ok())
  {



    // ROS_INFO_STREAM(angle_left<<" "<<angle_right<<" "<<distance_right<<" "<<distance_left);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
