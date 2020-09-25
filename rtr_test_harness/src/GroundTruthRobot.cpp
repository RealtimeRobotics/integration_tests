#include "rtr_test_harness/GroundTruthRobot.hpp"

namespace rtr {

namespace perception {

GroundTruthRobot::StampedJointConfiguration::StampedJointConfiguration(
    const SensorTime time, const std::vector<double>& joints)
    : timestamp(time), config(joints.size()) {
  for (size_t i = 0; i < joints.size(); ++i) {
    config[i] = joints[i];
  }
}

GroundTruthRobot::GroundTruthRobot(ros::NodeHandle& nh, const std::string& topic_name)
    : nh_(nh), topic_name_(topic_name) {
  joint_buffer_ = BufferQueueT<StampedJointConfiguration>::MakePtr();
  joint_sub_ = nh.subscribe(topic_name, 10, &GroundTruthRobot::JointCallback, this,
                            ros::TransportHints().reliable().tcpNoDelay());
}

GroundTruthRobot::~GroundTruthRobot() {
  if (robot_) {
    robot_.reset();
  }

  if (tf_manager_) {
    tf_manager_.reset();
  }
}

bool GroundTruthRobot::Init(const std::string& urdf) {
  robot_ = Robot::MakePtr();
  std::string urdf_path = RapidSenseInterface::GetPackagePath(urdf);
  if (!robot_->Init(urdf_path, Robot::GetDefaultROSPackagePath(urdf_path))) {
    RTR_ERROR("Robot failed to initialize with {} and package path {}", urdf_path,
              Robot::GetDefaultROSPackagePath(urdf_path));
  }

  tf_manager_ = TransformManager::MakePtr();

  return true;
}

void GroundTruthRobot::JointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  joint_buffer_->add(
      StampedJointConfiguration(RosTimeToSystemTime(msg->header.stamp), msg->position));
}

bool GroundTruthRobot::UpdateJointStates(const SensorTime timestamp) {
  if (joint_buffer_->empty()) {
    std::lock_guard<std::mutex> lk(topic_mutex_);
    JointCallback(ros::topic::waitForMessage<sensor_msgs::JointState>(topic_name_, nh_));
  }

  StampedJointConfiguration joints;
  auto get_time = [](const StampedJointConfiguration& config) { return config.timestamp; };
  GetClosestToTimestamp<StampedJointConfiguration>(joint_buffer_, get_time, timestamp, joints);

  if (std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - joints.timestamp)
                   .count())
      > 20) {
    RTR_WARN("Mismatching timestamps ({}ms), skipping frame",
             std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - joints.timestamp)
                 .count());
    return false;
  }

  // update the robot model
  std::vector<std::string> joint_names;
  robot_->GetJointNames(joint_names);
  robot_->SetConfig(joint_names, joints.config);

  std::vector<std::string> link_names;
  robot_->LinkNames(link_names);

  std::vector<Pose> poses(link_names.size());
  for (size_t i = 0; i < link_names.size(); ++i) {
    robot_->LinkToWorldTransform(link_names[i], poses[i]);
    poses[i].SetParent(TransformManager::World_Transform);
  }

  // update it in the transform manager
  tf_manager_->UpdateTransforms(link_names, poses);

  return true;
}

void GroundTruthRobot::SetJointStateTopic(const std::string& joint_topic) {
  std::lock_guard<std::mutex> lk(topic_mutex_);
  joint_buffer_->Clear();
  topic_name_ = joint_topic;
  joint_sub_.shutdown();
  joint_sub_ = nh_.subscribe(joint_topic, 10, &GroundTruthRobot::JointCallback, this,
                             ros::TransportHints().reliable().tcpNoDelay());
}

Robot::Ptr GroundTruthRobot::GetRobot() const {
  return robot_;
}

TransformManager::Ptr GroundTruthRobot::GetTransformManager() const {
  return tf_manager_;
}

}  // namespace perception

}  // namespace rtr
