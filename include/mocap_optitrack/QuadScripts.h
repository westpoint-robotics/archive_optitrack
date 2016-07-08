#ifndef __QUADSCRIPTS_H__
#define __QUADSCRIPTS_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>

#include "mocap_optitrack/constants_config.h"
#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>

namespace {
  const int WAND_ROTATE_TOP = 51;
  const int WAND_ROTATE_BOTTOM = 52;
}

// Forward declare to prevent circular dependencies
struct QuadData;
class Quad;

// Base class - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Control flow is done from the Quad class, no internal links
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class QuadScript {
public:
  // Subclasses have to initialize any new variables they have
  QuadScript();

  void give_data(QuadData* data_in);

  // Initializes publisher with correct type and topic
  virtual void init() = 0;

  // Returns true when terminating conditions for this script are met, possibly
  // including wand checks.
  virtual bool completed() const = 0;

  // Calculates and publishes specific script topic
  virtual void publish_topic() = 0;

  // Sets whether or not this script needs a wand check
  void set_needsWandCheck(bool input);

protected:
  // Generic publisher, type and topic published changes per script
  ros::Publisher pub;

  // Associated quad's data struct
  QuadData* data;

  // True when wand is rotated up with z > 0.8m
  bool standardWandCheckPasses() const;

  // Whether or not needs to wait for stdwandCheck to pass, default false
  bool needsWandCheck;
};

// DERIVED CLASSES - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class Takeoff : public QuadScript {
public:
  Takeoff(double height_in);

  virtual void init();

  virtual bool completed() const;

  virtual void publish_topic();

protected:
  // Height at end of takeoff
  double tkoff_height;

  bool locked;
  geometry_msgs::PoseStamped dest_pose;
};

class SetPose : public QuadScript {
public:
  SetPose(double px, double py, double pz,
          double ox, double oy, double oz, double ow);

  virtual void init();

  virtual bool completed() const;

  virtual void publish_topic();

protected:
  // Pose to set position to
  geometry_msgs::PoseStamped dest_pose;
};

class FollowOffset : public QuadScript {
public:
  // Follows quad with offset x,y,z
  FollowOffset(double x, double y, double z, int quad_num);

  // Follows quad with special mode
  FollowOffset(int mode_in, int quad_num);

  virtual void init();

  virtual bool completed() const;

  virtual void publish_topic();

protected:
  int quad_to_follow;
  int mode;

  geometry_msgs::PoseStamped dest_pose;
  geometry_msgs::Point offset;
};

class CatchBall : public QuadScript {
public:
  CatchBall();

  virtual void init();

  virtual bool completed() const;

  virtual void publish_topic();

protected:
  bool catching;
  geometry_msgs::PoseStamped dest_pose;

  int timer;
  bool dipping;
  int dip_timer;

  double last_z;
  double vz;
  bool hitPeak;
};

class Drift : public QuadScript {
public:
  Drift();

  virtual void init();
  virtual bool completed() const;
  virtual void publish_topic();

protected:
  geometry_msgs::PoseStamped locked_pose;
  geometry_msgs::PoseStamped dest_pose;

  bool init_locked;
  bool locked;
};

// Helper functions - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool pose_dist_check(geometry_msgs::Pose pose1,
                     geometry_msgs::Pose pose2,
                     double max_dist, double max_rot);

bool pose_xy_check(geometry_msgs::Pose pose1,
                   geometry_msgs::Pose pose2,
                   double max_dist);


#endif
