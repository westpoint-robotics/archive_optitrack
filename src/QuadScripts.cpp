#include "mocap_optitrack/Quad.h"
#include "mocap_optitrack/QuadScripts.h"

// Base class - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Publisher initialized in specific script class
QuadScript::QuadScript()
 : data(NULL), needsWandCheck(false) {}

void QuadScript::give_data(QuadData* data_in) {
  assert(data_in != NULL);
  data = data_in;
  init();
}

bool QuadScript::standardWandCheckPasses() const {
  bool rot_check = std::abs(data->wand_pose.pose.orientation.y) < 0.1;
  bool z_check = data->wand_pose.pose.position.z > 0.8;
  return rot_check && z_check;
}

void QuadScript::set_needsWandCheck(bool input) {
  needsWandCheck = input;
}

// TAKEOFF - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Takeoff::Takeoff(double height_in) : tkoff_height(height_in), locked(false) {}

void Takeoff::init() {
  ROS_INFO("Takeoff initialized");
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
}

bool Takeoff::completed() const {
  return data->local_pose.pose.position.z > tkoff_height;
}

// TODO fix this it sucks
void Takeoff::publish_topic() {
  ROS_INFO_ONCE("Starting Takeoff...");
  if (!locked) {
    dest_pose = data->local_pose;
    dest_pose.pose.position.z = 2.0 * tkoff_height;
    locked = data->state.armed && data->state.mode == "OFFBOARD";
  }
  pub.publish(dest_pose);
}


// SETPOSE - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
SetPose::SetPose(double px, double py, double pz,
                 double ox, double oy, double oz, double ow) {
  dest_pose.pose.position.x = px;
  dest_pose.pose.position.y = py;
  dest_pose.pose.position.z = pz;

  dest_pose.pose.orientation.x = ox;
  dest_pose.pose.orientation.y = oy;
  dest_pose.pose.orientation.z = oz;
  dest_pose.pose.orientation.w = ow;
}

void SetPose::init() {
  dest_pose.header = data->local_pose.header;

  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
  ROS_INFO("SetPose initialized");
}

bool SetPose::completed() const {
  double dist = 0.4;
  double rot = 0.1;

  bool wand_check = needsWandCheck ? standardWandCheckPasses() : 1;
  ROS_INFO(wand_check ? "wand pass" : "wand fail");
  ROS_INFO(pose_dist_check(dest_pose.pose,
                                       data->local_pose.pose,
                                       dist, rot) ?
            "dist pass" : "dist fail");
  return wand_check && pose_dist_check(dest_pose.pose,
                                       data->local_pose.pose,
                                       dist, rot);
}

void SetPose::publish_topic() {
  ROS_INFO_ONCE("Starting SetPose");
  dest_pose.header.stamp = ros::Time::now();
  pub.publish(dest_pose);
}

// FOLLOWOFFSET - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
FollowOffset::FollowOffset(double x, double y, double z, int quad_num)
 : mode(0), quad_to_follow(quad_num) {
  offset.x = x;
  offset.y = y;
  offset.z = z;
}

FollowOffset::FollowOffset(int mode_in, int quad_num)
 : mode(mode_in), quad_to_follow(quad_num) {}

void FollowOffset::init() {
  dest_pose.header = data->local_pose.header;
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
  ROS_INFO("FollowOffset initialized");
}

bool FollowOffset::completed() const {
  double dist = 0.2;
  double rot = 0.2;
  bool wand_check = needsWandCheck ? standardWandCheckPasses() : 1;
  bool dist_check = pose_dist_check(dest_pose.pose,
                                    data->local_pose.pose,
                                    dist, rot);
  // ROS_INFO(wand_check ? "wand pass" : "wand fail");
  // ROS_INFO(dist_check ? "dist pass" : "dist fail");
  return wand_check && dist_check;
}

void FollowOffset::publish_topic() {
  ROS_INFO_ONCE("Starting FollowOffset");
  dest_pose = data->other_quads.at(quad_to_follow)->get_local_pose();
  dest_pose.header.stamp = ros::Time::now();
  if (mode == 0) {
    dest_pose.pose.position.x += offset.x;
    dest_pose.pose.position.y += offset.y;
    dest_pose.pose.position.z += offset.z;
  }
  else {
    tf::Quaternion q;
    tf::quaternionMsgToTF(data->wand_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (mode == WAND_ROTATE_TOP) {
      dest_pose.pose.position.y -= sin(roll) * 0.5;
      dest_pose.pose.position.x += cos(roll) * 0.5;
    }
    else if (mode == WAND_ROTATE_BOTTOM) {
      dest_pose.pose.position.y += sin(roll) * 0.5;
      dest_pose.pose.position.x -= cos(roll) * 0.5;
    }
  }
  pub.publish(dest_pose);
}

// CatchBall - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// TODO this is placeholder
CatchBall::CatchBall() : catching(false) {}

void CatchBall::init() {
  // TODO This is placeholder
  dest_pose.header = data->local_pose.header;
  dest_pose.pose = data->local_pose.pose;
  dest_pose.pose.position.z = 0.5;
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
}

bool CatchBall::completed() const {
  // TODO placeholder
  return false;
}

void CatchBall::publish_topic() {
  ROS_INFO_ONCE("CatchBall started");
  double catch_z = 0.5;

  double x = data->ball_pose.pose.position.x;
  double y = data->ball_pose.pose.position.y;
  double z = data->ball_pose.pose.position.z;

  double a = -9.8;

  double vx = data->ball_vel.twist.linear.x;
  double vy = data->ball_vel.twist.linear.y;
  double vz = data->ball_vel.twist.linear.z;

  double t_plus = (-vz + sqrt(vz*vz - 2*a*(z - catch_z))) / a;
  double t_minus = (-vz - sqrt(vz*vz - 2*a*(z - catch_z))) / a;
  double t = t_plus > t_minus ? t_plus : t_minus;

  // Overcompensate for bad/delayed velocity calculations
  double x_proj = x + 4 * (vx * t);
  double y_proj = y + 4 * (vy * t);

  if (!catching) {
    dest_pose.header = data->local_pose.header;
    dest_pose.pose = data->local_pose.pose;
    dest_pose.pose.position.z = 0.5;
    catching = vz > 0.3 && z > 1;
  }


  // Overcompensate for speed
  geometry_msgs::PoseStamped comp_pose = dest_pose;

  if (z > catch_z + 0.1 && catching) {
    ROS_INFO_ONCE("catching!");
    dest_pose.pose.position.x = x_proj;
    dest_pose.pose.position.y = y_proj;
    dest_pose.pose.position.z = catch_z;
    double dx = dest_pose.pose.position.x - data->local_pose.pose.position.x;
    double dy = dest_pose.pose.position.y - data->local_pose.pose.position.y;
    comp_pose.pose.position.x += 2 * dx;
    comp_pose.pose.position.y += 2 * dy;

  }

  pub.publish(comp_pose);
}




// HELPER FUNCTIONS - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool pose_dist_check(geometry_msgs::Pose pose1,
                            geometry_msgs::Pose pose2,
                            double max_dist, double max_rot) {
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  // ROS_INFO("dist: %.2f", std::sqrt(dx*dx + dy*dy + dz*dz));
  bool dist_check = std::sqrt(dx*dx + dy*dy + dz*dz) < max_dist;

  double dox = pose1.orientation.x -
               pose2.orientation.x;
  double doy = pose1.orientation.y -
               pose2.orientation.y;
  double doz = pose1.orientation.z -
               pose2.orientation.z;
  double dow = pose1.orientation.w -
               pose2.orientation.w;
  bool rot_check = std::sqrt(dox*dox + doy*doy + doz*doz + dow*dow) < max_rot;

  return dist_check;
}
