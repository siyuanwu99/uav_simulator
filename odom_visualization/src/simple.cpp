#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <string>
#include "armadillo"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "pose_utils.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

static string mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale;

int  drone_id     = 0;
bool cross_config = false;
bool tf45         = false;
bool cov_pos      = false;
bool cov_vel      = false;
bool cov_color    = false;
bool origin       = false;
bool isOriginSet  = false;

std::string _frame_id;
int         _drone_id;

visualization_msgs::Marker trajROS;
visualization_msgs::Marker meshROS;

ros::Publisher meshPub;
ros::Publisher trajPub;

colvec poseOrigin(6);

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (msg->header.frame_id == string("null")) return;

  colvec pose(6);
  pose(0) = msg->pose.position.x;
  pose(1) = msg->pose.position.y;
  pose(2) = msg->pose.position.z;
  colvec q(4);

  q(0)            = msg->pose.orientation.w;
  q(1)            = msg->pose.orientation.x;
  q(2)            = msg->pose.orientation.y;
  q(3)            = msg->pose.orientation.z;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  if (origin && !isOriginSet) {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin) {
    vel  = trans(ypr_to_R(pose.rows(3, 5))) * vel;
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3, 5)) * vel;
  }

  // Mesh model
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp    = msg->header.stamp;
  meshROS.ns              = "mesh";
  meshROS.id              = 0;
  meshROS.type            = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action          = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.position.x;
  meshROS.pose.position.y = msg->pose.position.y;
  meshROS.pose.position.z = msg->pose.position.z;
  q(0)                    = msg->pose.orientation.w;
  q(1)                    = msg->pose.orientation.x;
  q(2)                    = msg->pose.orientation.y;
  q(3)                    = msg->pose.orientation.z;
  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x            = scale;
  meshROS.scale.y            = scale;
  meshROS.scale.z            = scale;
  meshROS.color.a            = color_a;
  meshROS.color.r            = color_r;
  meshROS.color.g            = color_g;
  meshROS.color.b            = color_b;
  meshROS.mesh_resource      = mesh_resource;
  meshPub.publish(meshROS);

  // Color Coded Trajectory
  static colvec    ppose = pose;
  static ros::Time pt    = msg->header.stamp;
  ros::Time        t     = msg->header.stamp;
  if ((t - pt).toSec() > 0.5) {
    trajROS.header.frame_id    = string("world");
    trajROS.header.stamp       = ros::Time::now();
    trajROS.ns                 = string("trajectory");
    trajROS.type               = visualization_msgs::Marker::LINE_LIST;
    trajROS.action             = visualization_msgs::Marker::ADD;
    trajROS.pose.position.x    = 0;
    trajROS.pose.position.y    = 0;
    trajROS.pose.position.z    = 0;
    trajROS.pose.orientation.w = 1;
    trajROS.pose.orientation.x = 0;
    trajROS.pose.orientation.y = 0;
    trajROS.pose.orientation.z = 0;
    trajROS.scale.x            = 0.1;
    trajROS.scale.y            = 0;
    trajROS.scale.z            = 0;
    trajROS.color.r            = 0.0;
    trajROS.color.g            = 1.0;
    trajROS.color.b            = 0.0;
    trajROS.color.a            = 0.8;
    geometry_msgs::Point p;
    p.x = ppose(0);
    p.y = ppose(1);
    p.z = ppose(2);
    trajROS.points.push_back(p);
    p.x = pose(0);
    p.y = pose(1);
    p.z = pose(2);
    trajROS.points.push_back(p);
    std_msgs::ColorRGBA color;
    color.r = color_r;
    color.g = color_g;
    color.b = color_b;
    color.a = 1;
    trajROS.colors.push_back(color);
    trajROS.colors.push_back(color);
    ppose = pose;
    pt    = t;
    trajPub.publish(trajROS);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_odom");
  ros::NodeHandle n("~");

  n.param("mesh_resource", mesh_resource,
          std::string("package://odom_visualization/meshes/hummingbird.mesh"));
  n.param("color/r", color_r, 1.0);
  n.param("color/g", color_g, 0.0);
  n.param("color/b", color_b, 0.0);
  n.param("color/a", color_a, 1.0);
  n.param("origin", origin, false);
  n.param("robot_scale", scale, 2.0);
  n.param("frame_id", _frame_id, string("world"));

  n.param("cross_config", cross_config, false);
  n.param("tf45", tf45, false);
  n.param("covariance_scale", cov_scale, 100.0);
  n.param("covariance_position", cov_pos, false);
  n.param("covariance_velocity", cov_vel, false);
  n.param("covariance_color", cov_color, false);
  n.param("drone_id", _drone_id, -1);

  ros::Subscriber sub_odom = n.subscribe("pose", 100, poseCallback);
  trajPub                  = n.advertise<visualization_msgs::MarkerArray>("trajectory", 100, true);
  meshPub                  = n.advertise<visualization_msgs::Marker>("robot", 100, true);

  ros::spin();

  return 0;
}
