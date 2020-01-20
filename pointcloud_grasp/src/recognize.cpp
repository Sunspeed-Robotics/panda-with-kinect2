#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_result_pc,pub_result_point;
typedef pcl::PointXYZRGB points;
typedef pcl::PointCloud<points> pointscloud;


void tfPub(const geometry_msgs::Point point,const std::string source,const std::string target)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(point.x, point.y, point.z) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  ros::Time transform_time = ros::Time::now();
  br.sendTransform(tf::StampedTransform(transform, transform_time, source.c_str(), target.c_str()));
}

bool inrange (const float h, const float s, const float v)
{
  bool result = true;

  if (h >34  || h <26) result = false;
  if (s < 43) result = false;
  if (v < 46) result = false;

  return result;
}

void rgb2hsv(const int r, const int g, const int b, float& h, float& s, float& v)
{
  float R = r/255.0;
  float G = g/255.0;
  float B = b/255.0;
  float min, max, delta, tmp, H, S, V;

  tmp = R>G?G:R;
  min = tmp>B?B:tmp;
  tmp = R>G?R:G;
  max = tmp>B?tmp:B;
  delta = max - min;

  V = max;

  if (V == 0) S = 0;
  else S = delta/V;

  if (delta == 0) H = 0;
  else if (V == G) H = (B - R)/delta + 2;
  else if (V == B) H = (R - G)/delta + 4;
  else H = (G - B)/delta;

  if (H < 0) H += 6;

  h = 30 * H;
  s = 255 * S;
  v = 255 * V;
}

pointscloud::Ptr object_dect (const pointscloud::Ptr cloud)
{
  pointscloud temp = *cloud;
  temp.erase(temp.begin(), temp.end());
  pointscloud::Ptr result = temp.makeShared();

  for (int i = 0; i < cloud -> points.size(); i++)
  {
    float h, s, v;
    int r, g, b;
    r = cloud -> points[i].r;
    g = cloud -> points[i].g;
    b = cloud -> points[i].b;

    rgb2hsv(r, g, b, h, s, v);

    if (inrange(h,s,v))
      result -> push_back(cloud ->points[i]);
  }

  return result;
}

pointscloud::Ptr filter(const pointscloud::Ptr cloud)
{
  pointscloud::Ptr cloud_filtered (new pointscloud);
  pcl::RadiusOutlierRemoval<points> seg;
  seg.setInputCloud(cloud);
  seg.setRadiusSearch(0.02);
  seg.setMinNeighborsInRadius(100);
  seg.filter(*cloud_filtered);

//  pcl::StatisticalOutlierRemoval<points> sor;
//  sor.setInputCloud (cloud);
//  sor.setMeanK (70);
//  sor.setStddevMulThresh (0.01);
//  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

void comp(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pointscloud::Ptr cloud (new pointscloud);
  pcl::fromROSMsg (*cloud_msg, *cloud);
  std::vector<int> mapping;

  //pointscloud::Ptr cloud_fil = cloud_filter(cloud);
  pointscloud::Ptr object = object_dect(cloud);

  pcl::removeNaNFromPointCloud(*object, *object, mapping);
  pointscloud::Ptr cloud_filtered = filter(object);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_filtered, centroid);

  sensor_msgs::PointCloud2 output_pc;
  pcl::toROSMsg (*cloud_filtered, output_pc);
  pub_result_pc.publish(output_pc);

  geometry_msgs::Point output;
  output.x = centroid[0];
  output.y = centroid[1];
  output.z = centroid[2];
  pub_result_point.publish(output);
  // Publish the data
  tfPub(output,"kinect2_rgb_optical_frame","object");
//  std::cout<<output.x<<" "<<output.y<<" "<<output.z<<std::endl;
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"grasp");
  ros::MultiThreadedSpinner spinner(2);
//  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string source,result;
  private_nh.param("source",source,std::string("/kinect2/sd/points"));
  private_nh.param("source",result,std::string("/grasp/result"));


  pub_result_pc = nh.advertise<sensor_msgs::PointCloud2>(result.c_str(),1);
  ros::Subscriber recv = nh.subscribe(source.c_str(),1,comp);
  pub_result_point = nh.advertise<geometry_msgs::Point> ("/grasp/result/point", 1);
//  ros::spin();
//  delete loop;
  spinner.spin();
  return 0;
}
