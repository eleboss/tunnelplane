#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


using namespace Eigen;
using namespace std;

typedef sensor_msgs::PointCloud2 PointCloud;
typedef geometry_msgs::PoseStamped PoseStamped;
typedef nav_msgs::Odometry Odometry;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

char PC_VIZ = 0; //enable visualization
//点云和transfromation的参数,根据相机安装位置来
float x_trans = 1;
float y_trans = 0;
float z_trans = 0;

//u = image x, v = image y
// Convert from u (column / width), v (row/height) to position in array
float central_u = 0;
float central_v = 0;
int u = 0;
int v = 0;

//bounding box
int box_height = 1;
int box_width = 1;
float box_theta = 0.74;
float grab_theta = 0; // if width>height then grab_theta = box_theta + 1.57
int box_x = 1;
int box_y = 1;
int box_point_level = 6;
char detection_avaliable = 0;//瓶子检测结果
float bottle_x, bottle_y, bottle_z;
float bottle_width = 0;
float focal_length = 921.959961; //if width>height then 物理长度d= box_width*depthofbottle/focal_length

float distance_percentage = 0.1; //均匀取点取中心点周围10%散开点八个点
int medium_num = 0;              //最终中值点: medium_num = medium_percentage * vaild_counter
float medium_percentage = 0.38;  //取有效点内中点点百分比
int depth_point_level = 4;       //采样点层数，3代表采样3X3=9个点，4代表采样4X4=16个点

float depth_samples[128];         //点云采样点数，点数=(depth_point_level * depth_point_level)*8，
float final_depth = 0;

double odom_pitch, odom_roll, odom_yaw;
double init_odom_roll, init_odom_pitch, init_odom_yaw;
char ODOM_INIT = 1;

ros::Publisher pub_trans;
ros::Publisher pub_trans_bottle;
ros::Publisher pub_final;

void pixelTo3DPoint(PointCloud &pCloud, int x, int y, float array[])
{

  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // where X,Y,Z data starts
  int arrayPosition = y * pCloud.row_step + x * pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  // put data into the point p
  array[0] = X;
  array[1] = Y;
  array[2] = Z;
}

// 将9点理解成为一张九个像素点图像，像素单位是pix_unit然后通过如下计算在循环中遍历三个大点
//每个大点中选择九个小点，两个循环分别遍历行列
void PushPoints(PointCloud &inputCloud, PointCloudXYZ::Ptr &outputpointcloud, int u, int v, int level)
{
  float pointXYZ[3];
  for (int i = level; i > 0; i--)
  {
    for (int j = level; j > 0; j--)
    {
      pixelTo3DPoint(inputCloud , u - int(level / 2) + i , v - int(level / 2) + j, pointXYZ);
      outputpointcloud->points.push_back(pcl::PointXYZ(pointXYZ[0], pointXYZ[1], pointXYZ[2]));
    }
  }
}

void quickSort(float s[], int l, int r)
{
  if (l < r)
  {
    int i = l, j = r;
    float x = s[l];
    while (i < j)
    {
      while (i < j && s[j] >= x) // 从右向左找第一个小于x的数
        j--;
      if (i < j)
        s[i++] = s[j];
      while (i < j && s[i] < x) // 从左向右找第一个大于等于x的数
        i++;
      if (i < j)
        s[j--] = s[i];
    }
    s[i] = x;
    quickSort(s, l, i - 1); // 递归调用
    quickSort(s, i + 1, r);
  }
}

void callback_odom(const Odometry::ConstPtr &odom)
{
  // ROS_INFO("I heard odom");
  if(ODOM_INIT)
  {
    tf::Quaternion odom_qn(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(odom_qn);
    m.getRPY(init_odom_roll, init_odom_pitch, init_odom_yaw);
    ODOM_INIT = 0;
  }
  else
  {
    tf::Quaternion odom_qn(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(odom_qn);
    m.getRPY(odom_roll, odom_pitch, odom_yaw);

    odom_roll = odom_roll - init_odom_roll;
    odom_pitch = odom_pitch - init_odom_pitch;
    odom_yaw = odom_yaw - init_odom_yaw;

    if (odom_roll < -M_PI)
        odom_roll = odom_roll + 2 * M_PI;
    if (odom_roll > M_PI)
        odom_roll = odom_roll - 2 * M_PI;

    if (odom_pitch < -M_PI)
        odom_pitch = odom_pitch + 2 * M_PI;
    if (odom_pitch > M_PI)
        odom_pitch = odom_pitch - 2 * M_PI;

    if (odom_yaw < -M_PI)
        odom_yaw = odom_yaw + 2 * M_PI;
    if (odom_yaw > M_PI)
        odom_yaw = odom_yaw - 2 * M_PI;

    // std::cout << "odom_roll: " << odom_roll << "odom_pitch: " << odom_pitch << "odom_yaw: " << odom_yaw;
  }

}

void callback_pc(const PointCloud::ConstPtr &msg)
{
  int iter;
  int len;
  float sum_of_samples = 0;
  int depth_vaild_counter, depth_invaild_counter, bottle_vaild_counter;
  if (PC_VIZ ==1)
  {
    printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    ROS_INFO("I Heard pc");
    ROS_INFO_STREAM("Cloud: width = " << msg->width << " height =" << msg->height);
  }
  PointCloudXYZ::Ptr depthcloud(new PointCloudXYZ());
  PointCloudXYZ::Ptr bottlecloud(new PointCloudXYZ());
  PointCloudXYZ::Ptr transformed_depth_cloud(new PointCloudXYZ());
  PointCloudXYZ::Ptr transformed_bottle_cloud(new PointCloudXYZ());
  PointCloud receivedPointCloud = *msg;

  //八点定高法，选择中心点周围的八个点，除了中心点以外，八个点的排列方式为下图所示
  // 1   2   3
  // 4  中心  5
  // 6   7   8
  //计算像素中心点
  central_u = int(msg->width / 2);
  central_v = int(msg->height / 2);
  //因为图像一般都是比较宽，因此选用短边（高）作为取点单位，从中心按照百分比散开取点
  int pix_unit = distance_percentage * msg->height;
  //上方三个点的处理，使用函数封装会造成不必要点计算量的浪费，因此直接展开写
  for (int k = 2; k >= 0; k--)
  {
    PushPoints(receivedPointCloud, depthcloud,central_u - pix_unit + pix_unit * k, central_v - pix_unit, depth_point_level);
    // 将9点理解成为一张九个像素点图像，像素单位是pix_unit然后通过如下计算在循环中遍历三个大点
  }
  //处理下方的三个点
  for (int k = 2; k >= 0; k--)
  {
    PushPoints(receivedPointCloud, depthcloud,central_u - pix_unit + pix_unit * k, central_v + pix_unit, depth_point_level);
  }
  //middle left
  PushPoints(receivedPointCloud, depthcloud, central_u - pix_unit, central_v, depth_point_level);
  //middle right
  PushPoints(receivedPointCloud, depthcloud, central_u + pix_unit, central_v, depth_point_level);

  //Instruction of realsense coordinate
  //Each stream of images provided by this SDK is also associated with a separate 3D coordinate space, specified in meters, with the coordinate [0,0,0] 
  //referring to the center of the physical imager. Within this space, the positive x-axis points to the right, the positive y-axis points down, and the 
  //positive z-axis points forward. Coordinates within this space are referred to as "points", and are used to describe locations within 3D space that 
  //might be visible within a particular image.

  //创建旋转矩阵，这里的旋转的角度根据IMU给出
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << x_trans, y_trans, z_trans;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate(Eigen::AngleAxisf(odom_roll, Eigen::Vector3f::UnitX()));
  transform_2.rotate(Eigen::AngleAxisf(odom_pitch, Eigen::Vector3f::UnitY()));
  transform_2.rotate(Eigen::AngleAxisf(-odom_yaw, Eigen::Vector3f::UnitZ()));


  // Print the transformation
  // printf("\nMethod #2: using an Affine3f\n");
  // std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation


  pcl::transformPointCloud(*depthcloud, *transformed_depth_cloud, transform_2);


  detection_avaliable = 0;//由王金旺的检测程序给出，置位后进行瓶子3d数据的提取
  //检测结果有效时进入点云处理
  if (detection_avaliable)
  {
    if (box_point_level > box_height)
    {
      box_point_level = 4;
    }
    else if(box_point_level > box_width)
    {
      box_point_level = 4;
    }
    PushPoints(receivedPointCloud, bottlecloud, box_x, box_y, box_point_level);

    //为了计算box窄边的宽度，判断那边比较段
    if (box_height < box_width)
    {
      grab_theta = box_theta + float(M_PI/2);
    }

    
    pcl::transformPointCloud(*bottlecloud, *transformed_bottle_cloud, transform_2);

    bottle_vaild_counter = 0; 
    bottle_x = 0;
    bottle_y = 0;
    bottle_z = 0;
    BOOST_FOREACH (const pcl::PointXYZ &pt, transformed_bottle_cloud->points)
    {
      if (pt.x < 20 && pt.x > -20 && pt.y < 20 && pt.y > -20 && pt.z < 20 && pt.z > -20 )
      {
        bottle_x = bottle_x + pt.x;
        bottle_y = bottle_y + pt.y;
        bottle_z = bottle_z + pt.z;
        bottle_vaild_counter = bottle_vaild_counter + 1;
        // ROS_INFO_STREAM("x"<<bottle_x<<" y"<<bottle_y<<" z"<<bottle_z);
      }
    }
    //失效处理
    if(bottle_vaild_counter != 0)
    {
      bottle_x = bottle_x / bottle_vaild_counter;
      bottle_y = bottle_y / bottle_vaild_counter;
      bottle_z = bottle_z / bottle_vaild_counter;

      //计算短边宽度
      bottle_width = bottle_z * min(box_height , box_width) / focal_length;
 

    }
    else
    {
      bottle_x = 999;
      bottle_y = 999;
      bottle_z = 999;
    }
      // ROS_INFO_STREAM("final x"<<bottle_x<<" final y"<<bottle_y<<" final z"<<bottle_z<<"final width"<<bottle_width); 
    //标志位置0
    detection_avaliable = 0;
  }

  //点云数据可视化
  if(PC_VIZ)
  {
    pcl_conversions::toPCL(ros::Time::now(), depthcloud->header.stamp);
    depthcloud->header.frame_id = "camera_color_optical_frame";
    pub_trans.publish(depthcloud);


    pcl_conversions::toPCL(ros::Time::now(), transformed_bottle_cloud->header.stamp);
    transformed_bottle_cloud->header.frame_id = "camera_color_optical_frame";
    pub_trans_bottle.publish(transformed_bottle_cloud);
  }

  iter = 0;
  depth_invaild_counter = 0;
  depth_vaild_counter = 0;
  len = sizeof(depth_samples) / sizeof(float);

  //easy to cause core dump!!! Don't forget to modify the depth_samples array number.
  BOOST_FOREACH (const pcl::PointXYZ &pt, transformed_depth_cloud->points)
  {
    // ROS_INFO_STREAM(iter);
    //如果数据有效则记录，否则作为无效数据处理（给-1），无效数据在排序之后就能被滤除
    //TODO 无效数据直接不存下来，增加快排效率
    if (pt.x < 10 && pt.x > 0)
    {
      depth_samples[iter] = pt.z;
      depth_vaild_counter = depth_vaild_counter + 1;
    }
    else
    {
      depth_samples[iter] = -1;
      depth_invaild_counter = depth_invaild_counter + 1;
    }
    iter = iter + 1;
  }

  
  // printf("排序前\n");
  // printf("%d \n",depth_invaild_counter);
  // for (int i = 0; i<sizeof(depth_samples) / sizeof(depth_samples[0]); i++)
  // {
  //     printf("%f ", depth_samples[i]);
  // }

  //快速排序，排序后数据从小到大排列，invaild_counter的数值在数组中表示第一个有效数据
  quickSort(depth_samples, 0, len - 1);

  // printf("排序后\n");
  // for (int i = 0; i<sizeof(depth_samples) / sizeof(depth_samples[0]); i++)
  // {
  //     printf("%f ", depth_samples[i]);
  // }

  //中值滤波，按照百分比取中值点
  medium_num = int(depth_vaild_counter * medium_percentage);
  //如果有效点实在太少，算无效处理
  if (medium_num == 0)
  {
    depth_vaild_counter = 0;
  }
  //失效处理
  if(depth_vaild_counter > 0)
  {
    for (int i = 0; i < medium_num; i++)
    {
      //从invaild_counter开始为有效数据，中间数据用全长/2，然后从中间向后找medium_num/2开始遍历相加
      sum_of_samples = sum_of_samples + depth_samples[depth_invaild_counter + i + int(depth_vaild_counter / 2) - int(medium_num / 2)];
      // printf("looping namber %f",depth_samples[invaild_counter + i + int(vaild_counter / 2) - int(medium_num/2)]);
    }
    final_depth = sum_of_samples / medium_num;
  }
  else
  {
    //深度为-1代表无效
    final_depth = -1;
  }
  // realsense要求在0.2m以上才能成像
  if(final_depth <0.1)
  {
    final_depth = -1;
  }
  if(PC_VIZ)
  {
    printf(" 最终高度结果 %f \n", final_depth);
  }

  Odometry depth_msg;
  depth_msg.header.stamp = ros::Time::now();
  depth_msg.header.frame_id = "world";
  depth_msg.child_frame_id = "base_link";
  depth_msg.pose.pose.position.z = final_depth;
  pub_final.publish(depth_msg);


}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("camera/depth/color/points", 1, callback_pc);

  ros::Subscriber sub_odom = nh.subscribe<Odometry>("mavros/local_position/odom", 1, callback_odom);

  pub_final = nh.advertise<Odometry>("depth", 1);
  pub_trans = nh.advertise<PointCloudXYZ>("pointdepth", 1);
  pub_trans_bottle = nh.advertise<PointCloudXYZ>("pointbottle", 1);

  ros::spin();
}

//检测程序运行时间专用代码
// #include <iostream>
// #include<time.h>
// clock_t start,finish,lasttime;
// double totaltime,totaltime2;
// finish=clock();
// lasttime=clock();
// totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
// totaltime2 =(double)(lasttime-finish)/CLOCKS_PER_SEC;
// cout<<"\n1程序的运行时间为"<<totaltime<<"秒！"<<endl;
// cout<<"\n2程序的运行时间为"<<totaltime2<<"秒！"<<endl;






