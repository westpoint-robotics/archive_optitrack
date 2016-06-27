#include "mocap_optitrack/Quad.h"
#include "mocap_optitrack/QuadScripts.h"

// PUBLIC - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Constructor and Destructor
Quad::Quad(std::string ns_in) {
  // This means this is not a real quad and most data must be fabricated
  if (ns_in[0] == '_') {
    data.node = ros::NodeHandle("fabr" + ns_in);
    quad_type = FABR_QUAD;
    if (ns_in == "_wand_proj") {
      quad_type = WAND_PROJECTION;
    }
    ROS_INFO("fake quad created");
    wand_pose_sub = data.node.subscribe("/wand_pose",
                                        FRAMES_PER_SEC,
                                        &Quad::wand_pose_callback,
                                        this);
    plat_pose_sub = data.node.subscribe("/object_pose",
                                        FRAMES_PER_SEC,
                                        &Quad::plat_pose_callback,
                                        this);
    ball_pose_sub = data.node.subscribe("/ball_pose",
                                        FRAMES_PER_SEC,
                                        &Quad::ball_pose_callback,
                                        this);
  }
  else {
    quad_type = 0;
    data.node = ros::NodeHandle(ns_in);
    // Initialize vel queues to correct size, data cycled through in pose subs
    geometry_msgs::PoseStamped init_pose;
    for (int i = 0; i < FRAMES_PER_SEC / VEL_T_DENOM; ++i) {
      past_wand_pose.push(init_pose);
      past_plat_pose.push(init_pose);
      past_ball_pose.push(init_pose);
    }

    // Init subs
    state_sub = data.node.subscribe("mavros/state",
                                    FRAMES_PER_SEC,
                                    &Quad::state_callback,
                                    this);
    local_pose_sub = data.node.subscribe("mavros/local_position/pose",
                                         FRAMES_PER_SEC,
                                         &Quad::local_pose_callback,
                                         this);
    wand_pose_sub = data.node.subscribe("/wand_pose",
                                        FRAMES_PER_SEC,
                                        &Quad::wand_pose_callback,
                                        this);
    plat_pose_sub = data.node.subscribe("/object_pose",
                                        FRAMES_PER_SEC,
                                        &Quad::plat_pose_callback,
                                        this);
    ball_pose_sub = data.node.subscribe("/ball_pose",
                                        FRAMES_PER_SEC,
                                        &Quad::ball_pose_callback,
                                        this);
    local_vel_sub = data.node.subscribe("mavros/local_position/velocity",
                                        FRAMES_PER_SEC,
                                        &Quad::local_vel_callback,
                                        this);

    // Init disarming client
    disarm_client = data.node.serviceClient<mavros_msgs::CommandBool>
                                           ("mavros/cmd/arming");
  }
}

Quad::~Quad() {
  ROS_INFO_ONCE("Cleaning up remaining quad memory");
  while (!script_queue.empty()) {
    delete script_queue.front();
    script_queue.pop();
  }
}

void Quad::run() {

  check_for_disarm_cmd();

  if (!script_queue.empty()) {
    int size = script_queue.size();
    // Delete script when moving to next one
    if (script_queue.front()->completed() && script_queue.size() > 1) {
      ROS_INFO("Script completed, starting next one.");
      delete script_queue.front();
      script_queue.pop();
    }
    script_queue.front()->publish_topic();
    ros::spinOnce();
  }
}

// Getters
const geometry_msgs::PoseStamped& Quad::get_local_pose() const {
  return data.local_pose;
}

const geometry_msgs::TwistStamped& Quad::get_local_vel() const {
  return data.local_vel;
}

QuadScript* Quad::front() {
  return script_queue.front();
}

QuadScript* Quad::back() {
  return script_queue.back();
}

bool Quad::isReal() {
  return quad_type == REAL_QUAD;
}

// Adders
void Quad::add_quad(Quad* other_quad) {
  data.other_quads.push_back(other_quad);
}

void Quad::add_script(QuadScript* script_in) {
  script_in->give_data(&data);
  script_queue.push(script_in);
}

// Disarm
void Quad::disarm() {
  if (data.state.armed) {
    mavros_msgs::CommandBool cmd;
    cmd.request.value = false;
    disarm_client.call(cmd);
    // ROS_INFO("Quad disarmed.");
  }
}


// PRIVATE - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Callbacks
void Quad::state_callback(const mavros_msgs::State::ConstPtr& msg) {
  data.state.header = msg->header;
  data.state.connected = msg->connected;
  data.state.armed = msg->armed;
  data.state.guided = msg->guided;
  data.state.mode = msg->mode;
}

void Quad::local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.local_pose.header = msg->header;
  data.local_pose.pose = msg->pose;
}

void Quad::wand_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.wand_pose.header = msg->header;
  data.wand_pose.pose = msg->pose;

  past_wand_pose.push(data.wand_pose);
  past_wand_pose.pop();
  set_wand_vel();

  if (quad_type == WAND_PROJECTION) {
    // // Project out from end of wand
    // tf::Quaternion q;
    // tf::quaternionMsgToTF(data.wand_pose.pose.orientation, q);
    // double roll, pitch, yaw;
    // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //
    //
    // ROS_INFO(" ");
    // ROS_INFO("roll: %.2f", roll);
    // ROS_INFO("pitch: %.2f", pitch);
    // ROS_INFO("yaw: %.2f", yaw);
    data.local_pose.pose.position.x = -1;
    data.local_pose.pose.position.y = 0;
    data.local_pose.pose.position.z = 0.5;
  }
}

void Quad::plat_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.plat_pose.header = msg->header;
  data.plat_pose.pose = msg->pose;

  past_plat_pose.push(data.plat_pose);
  past_plat_pose.pop();
  set_plat_vel();
}

void Quad::ball_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.ball_pose.header = msg->header;
  data.ball_pose.pose = msg->pose;

  past_ball_pose.push(data.ball_pose);
  past_ball_pose.pop();
  set_ball_vel();
}

void Quad::local_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  data.local_vel.header = msg->header;
  data.local_vel.twist = msg->twist;
}

// Vel setters
void Quad::set_wand_vel() {
  // Angular velocity fields are all zero and should not be used
  data.wand_vel.header = past_wand_pose.back().header;
  double dx = past_wand_pose.back().pose.position.x -
              past_wand_pose.front().pose.position.x;
  double dy = past_wand_pose.back().pose.position.y -
              past_wand_pose.front().pose.position.y;
  double dz = past_wand_pose.back().pose.position.z -
              past_wand_pose.front().pose.position.z;
  double dt = 1 / VEL_T_DENOM;
  data.wand_vel.twist.linear.x = dx / dt;
  data.wand_vel.twist.linear.y = dy / dt;
  data.wand_vel.twist.linear.z = dz / dt;
}

void Quad::set_plat_vel() {
  data.plat_vel.header = past_plat_pose.back().header;
  double dx = past_plat_pose.back().pose.position.x -
              past_plat_pose.front().pose.position.x;
  double dy = past_plat_pose.back().pose.position.y -
              past_plat_pose.front().pose.position.y;
  double dz = past_plat_pose.back().pose.position.z -
              past_plat_pose.front().pose.position.z;
  double dt = 1 / VEL_T_DENOM;
  data.plat_vel.twist.linear.x = dx / dt;
  data.plat_vel.twist.linear.y = dy / dt;
  data.plat_vel.twist.linear.z = dz / dt;
}

void Quad::set_ball_vel() {
  data.ball_vel.header = past_ball_pose.back().header;
  double dx = past_ball_pose.back().pose.position.x -
              past_ball_pose.front().pose.position.x;
  double dy = past_ball_pose.back().pose.position.y -
              past_ball_pose.front().pose.position.y;
  double dz = past_ball_pose.back().pose.position.z -
              past_ball_pose.front().pose.position.z;
  double dt = 1 / VEL_T_DENOM;
  data.ball_vel.twist.linear.x = dx / dt;
  data.ball_vel.twist.linear.y = dy / dt;
  data.ball_vel.twist.linear.z = dz / dt;
}

void Quad::check_for_disarm_cmd() {
  if (data.wand_vel.twist.linear.z < -3.5 &&
      std::abs(data.wand_pose.pose.orientation.y) > 0.9) {
    disarm();
  }
}

// FORMATION - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Formation::Formation() : initialized(false) {}

void Formation::run() {
  for (int i = 0; i < quad_list.size(); ++i) {
    quad_list.at(i)->run();
  }
  if (!initialized) {
    sleep(1.0);
    initialized = true;
  }
  else {
    check_for_collisions();
  }
}

void Formation::add_quad(Quad& quad) {
  for (int i = 0; i < quad_list.size(); ++i) {
    quad_list.at(i)->add_quad(&quad);
    quad.add_quad(quad_list.at(i));
  }
  quad.add_quad(&quad);
  quad_list.push_back(&quad);
}

void Formation::print_count() {
  int size = quad_list.size();
  ROS_INFO("num of quads: %d", size);
}

void Formation::check_for_collisions() {
  for (int i = 0; i < quad_list.size(); ++i) {
    if (quad_list.at(i)->isReal()) {

      // Check if i is near another quad
      for (int j = i + 1; j < quad_list.size(); ++j) {
        if (pose_dist_check(quad_list.at(i)->get_local_pose().pose,
                            quad_list.at(j)->get_local_pose().pose,
                            0.7, 1.0)) {
          quad_list.at(i)->disarm();
          quad_list.at(j)->disarm();
          // ROS_INFO("Too close");
        }
      }

      // Check if i is near the boundaries
      if (!insideBoundaries(quad_list.at(i)->get_local_pose().pose)) {
        quad_list.at(i)->disarm();
        // ROS_INFO("Outside boundaries");
      }

    }
  }
}

bool insideBoundaries(geometry_msgs::Pose pose) {
  return pose.position.x > -2.05 && pose.position.x < 0.00 &&
         pose.position.y > -1.30 && pose.position.y < 1.40 &&
         pose.position.z < 2.5;
}
