#ifndef RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_GROUNDTRUTHROBOT_HPP_
#define RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_GROUNDTRUTHROBOT_HPP_

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <rtr_core/TransformManager.hpp>
#include <rtr_math/Random.hpp>
#include <rtr_perc_api/BufferUtils.hpp>
#include <rtr_perc_api/RapidSenseMessage.hpp>
#include <rtr_perc_rapidsense/RapidSenseInterface.hpp>
#include <rtr_perc_sensors_ros/StampedMessage.hpp>

namespace rtr {
namespace perception {

/***********************************************************************************************
 * GroundTruthRobot                                                                             *
 *                                                                                              *
 * Subscribes to joint states, buffers joint states with a timestamp. When given a timestamp,   *
 * updates robot and TransformManager with joint states matching the provided timestamp.        *
 * Similar to the RosRobotProxy, but matches specific timestamps and does not run in a loop.    *
 ************************************************************************************************/
class GroundTruthRobot {
  DEFINE_SMART_PTR(GroundTruthRobot)

 public:
  GroundTruthRobot(ros::NodeHandle& nh, const std::string& topic_name);
  ~GroundTruthRobot();

  //! @brief Initialize GroundTruthRobot
  bool Init(const std::string& urdf);

  //! @brief Update joint states with closest state in joint buffer
  bool UpdateJointStates(const SensorTime timestamp);

  //! @brief Get Robot
  Robot::Ptr GetRobot() const;

  //! @brief Get TransformManager
  TransformManager::Ptr GetTransformManager() const;

  //! @brief Set joint state topic to subscribe to
  void SetJointStateTopic(const std::string& joint_topic);

 private:
  struct StampedJointConfiguration {
    StampedJointConfiguration() {}
    StampedJointConfiguration(const SensorTime time, const std::vector<double>& joints);

    SensorTime timestamp;
    JointConfiguration config;
  };

  //! @brief Buffers joint states
  void JointCallback(const sensor_msgs::JointState::ConstPtr& msg);

  ros::NodeHandle nh_;

  std::mutex topic_mutex_;
  std::string topic_name_;

  ros::Subscriber joint_sub_;

  Robot::Ptr robot_;
  TransformManager::Ptr tf_manager_;
  BufferInterface<StampedJointConfiguration>::Ptr joint_buffer_;
};

}  // namespace perception
}  // namespace rtr

#endif  // RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_GROUNDTRUTHROBOT_HPP_
