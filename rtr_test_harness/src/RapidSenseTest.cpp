#include "rtr_test_harness/RapidSenseTest.hpp"

#include <rtr_app_layer/RapidPlanProject.hpp>
#include <rtr_control_ros/RosController.hpp>
#include <rtr_msgs/GetGroupInfo.h>
#include <rtr_msgs/GetProjectROSInfo.h>
#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
#include <rtr_perc_rapidsense_ros/RosRobotConnection.hpp>
#include <rtr_roadmapgen/roadmap_generator.hpp>

using ExtCodeSeqPair = rtr::ApplianceCommander::ExtCodeSeqPair;

namespace rtr {
namespace perception {

RapidSenseTest::RapidSenseTest()
    : nh_(""), app_commander_("127.0.0.1"), proxy_(RapidSenseFrontEndProxy::MakePtr()) {}

bool RapidSenseTest::Init(const std::string& dir) {
  test_dir_ = dir;

  if (!SetIgnoreVisionEnabledOnServer(true)) {
    return false;
  }

  if (!config_.Deserialize(GetParameterFilename())) {
    return false;
  }

  if (config_.sim_mode) {
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("restart_sim");
    std_srvs::Trigger trigger;
    if (!client.call(trigger) || !trigger.response.success) {
      RTR_ERROR("Failed to refresh server");
      return false;
    }
  } else if (!proxy_->SetOperationMode()) {
    RTR_ERROR("Failed to refresh server");
    return false;
  }

  if (!proxy_->GetConfiguration(rapidsense_config_).IsSuccess()) {
    RTR_ERROR("Failed to get config from server");
    return false;
  }
  observers_ = proxy_->GetObservers();
  region_desc_ =
      VoxelRegionDescription(proxy_->GetVoxelRegionNumVoxels(config_.voxel_stream_name),
                             proxy_->GetVoxelRegionDimensions(config_.voxel_stream_name) / 2.0,
                             proxy_->GetVoxelRegionPose(config_.voxel_stream_name).Inverse());

  if (observers_.empty()) {
    RTR_ERROR("Cannot initialize with no robots");
    return false;
  }

  if (!app_commander_.IsConnected()) {
    RTR_ERROR("App commander is not connected");
    return false;
  }

  for (const auto& observer : observers_) {
    RTR_INFO("Decon group {} {} {}", config_.decon_group, observer->GetName(),
             observer->GetStateSpaceName());
    ExtCode res = app_commander_.InitGroup(config_.decon_group, observer->GetName(),
                                           observer->GetStateSpaceName());
    if (res != ExtCode::SUCCESS) {
      RTR_ERROR("Failed to InitGroup with error: {}", Convert<std::string>(ExtCode(res)));
      return false;
    }

    res = app_commander_.SetInterruptBehavior(observer->GetName(), 5, 10);
    if (res != ExtCode::SUCCESS) {
      RTR_ERROR("Failed to SetInterruptBehavior with error: {}",
                Convert<std::string>(ExtCode(res)));
      return false;
    }
  }

  benchmark_manager_ = RosBenchmarkManager::MakePtr(nh_, config_.voxel_stream_name, region_desc_);

  return benchmark_manager_->Init(observers_);
}

bool RapidSenseTest::CheckRapidSenseServerState_() {
  // check RapidSense
  ros::ServiceClient get_config_client =
      nh_.serviceClient<rtr_msgs::GetSchemaMessage>(RS::Topic("get_configuration"));
  if (!ros::service::waitForService(get_config_client.getService(), ros::Duration(10.0))) {
    RTR_ERROR("Timed out waiting for configuration from RapidSenseServer");
    return false;
  }

  rtr_msgs::GetSchemaMessage srv;
  if (!get_config_client.call(srv)
      || !FromSchemaMessageResponse(srv.response, rapidsense_config_)) {
    RTR_ERROR("Failed to get configuration from RapidSenseServer");
    return false;
  }

  bool config_ok = true;
  for (auto& stream : rapidsense_config_.streams) {
    if (stream.enable_robotfilter != config_.test_robot_filter) {
      stream.enable_robotfilter = config_.test_robot_filter;
      config_ok = false;
    }
  }

  if (!config_ok) {
    RTR_ERROR("RapidSenseServer configuration does not match test requirements");
    return false;
  }

  RTR_WARN("Waiting for RapidSense health on topic {}", RS::Topic("health"));
  rtr_msgs::SchemaMessage::ConstPtr health_msg =
      ros::topic::waitForMessage<rtr_msgs::SchemaMessage>(RS::Topic("health"), ros::Duration(10.0));
  if (!health_msg) {
    RTR_ERROR("Timed out waiting for health message from RapidSenseServer");
    return false;
  }

  RapidSenseHealth health;
  try {
    health = FromSchemaMessage<RapidSenseHealth>(*health_msg);
  } catch (std::exception& e) {
    RTR_ERROR("Could not get RapidSense health: {}", e.what());
    return false;
  }

  if (health.active_deconfliction_group != config_.decon_group) {
    return false;
  }

  return health.active_deconfliction_group == config_.decon_group
         && health.current_status.state == RapidSenseState::OPERATION;
}

std::string RapidSenseTest::GetParameterFilename() const {
  return test_dir_ + "/test.xml";
}

bool RapidSenseTest::Run(const bool use_live_data, const bool record_data) {
  test_result_ = RapidSenseTestResult::MakePtr(config_.GetName());
  ExtCode res;
  size_t seq_num;

  if (use_live_data) {
    res = app_commander_.BeginOperationMode();
    if (res != ExtCode::SUCCESS) {
      RTR_ERROR("Failed to BeginOperationMode with error: {}", Convert<std::string>(ExtCode(res)));
      test_result_->result_ = RapidSenseTestResult::OPERATION_FAILURE;
      return false;
    }
  }

  // attempt to move to first hub
  // TODO: extend this to multirobot
  RapidSenseTestHubConfig hub_config = config_.hub_sequence.front();
  ExtCodeSeqPair pair = app_commander_.MoveToHub(hub_config.active_robot, hub_config.state_space,
                                                 hub_config.hub_name, config_.speed);
  res = pair.first;
  seq_num = pair.second;
  if (res == ExtCode::START_CONFIG_NOT_WITHIN_TOL_TO_ROADMAP) {
    // offroad to hub (y/n)
    std::string input;
    std::cout << "Starting configuration of robot {} is not within tolerance "
                 "to roadmap. Offroad "
                 "to hub? (y/n)";
    std::getline(std::cin, input);
    if (input == "y") {
      pair = app_commander_.OffroadToHub(hub_config.active_robot, hub_config.state_space,
                                         hub_config.hub_name);
    }
  }

  if (res != ExtCode::SUCCESS) {
    RTR_ERROR("Failed to move to initial hub with error: {}", Convert<std::string>(ExtCode(res)));
    test_result_->result_ = RapidSenseTestResult::OPERATION_FAILURE;
    return false;
  }

  res = app_commander_.WaitForMove(seq_num);
  if (res != ExtCode::SUCCESS) {
    RTR_ERROR("Failed to WaitForMove with error: {}", Convert<std::string>(ExtCode(res)));
    test_result_->result_ = RapidSenseTestResult::OPERATION_FAILURE;
    return false;
  }

  // start recording if necessary
  if (record_data) {
    if (!proxy_->StartRecording(config_.GetName(), 0)) {
      RTR_ERROR("Failed to start recording");
      test_result_->result_ = RapidSenseTestResult::OPERATION_FAILURE;
      return false;
    }
  }

  // start benchmarking
  auto on_data = [this](const BenchmarkManager::MetricFrame& metric) {
    this->test_result_->Update(metric);
  };
  benchmark_manager_->SetRobotBenchmarkEnable(!config_.test_robot_filter);
  benchmark_manager_->ResetFrameBuffer();

  std::string conn_id = benchmark_manager_->SubscribeToMetrics(on_data);

  // begin moving to hubs
  std::vector<RapidSenseTestHubConfig> curr_hub_sequence;
  MoveToHubs_(curr_hub_sequence);

  // end benchmarking
  benchmark_manager_->UnsubscribeFromMetrics(conn_id, true);

  // stop recording if necessary
  if (record_data) {
    proxy_->StopRecording(config_.GetName());
  }

  if (use_live_data) {
    res = app_commander_.EndOperationMode();
    if (res != ExtCode::SUCCESS) {
      RTR_ERROR("Failed to EndOperationMode with error: {}", Convert<std::string>(ExtCode(res)));
    }
  }

  if (test_result_->result_ != RapidSenseTestResult::SUCCESS) {
    return false;
  }

  // compare or rewrite test configs
  // for now, always pass tests if they run. in the future, check if they
  // followed the same sequence
#if 0
  auto& last_hub_sequence = config_.hub_sequence;
  for (std::size_t h_idx = 0; h_idx < curr_hub_sequence.size(); ++h_idx) {
    auto& curr_joint_configs = curr_hub_sequence[h_idx].joint_configs;
    auto& last_joint_configs = last_hub_sequence[h_idx].joint_configs;

    if (curr_joint_configs.size() != last_joint_configs.size()) {
      // if we don't have a recorded set of configs yet, just create one
      if (last_joint_configs.empty()) {
        last_hub_sequence = curr_hub_sequence;
        config_.Serialize(GetParameterFilename());
        break;

        // otherwise, we have deviated from the previous path, fail test
      } else {
        test_result_->result_ = RapidSenseTestResult::PATH_FAILURE;
        RTR_ERROR("Test resulted in inconsistent path");
        return false;
      }
    } else {
      for (std::size_t j_idx = 0; j_idx < curr_joint_configs.size(); ++j_idx) {
        if (!curr_joint_configs[j_idx].FuzzyEquals(last_joint_configs[j_idx],
                                                   0.075)) {  // within ~5 degreeds
          test_result_->result_ = RapidSenseTestResult::PATH_FAILURE;
          RTR_ERROR("Test resulted in inconsistent path");
          return false;
        }
      }
    }
  }
#endif

  // compute, write and print results
  test_result_->ComputeFinalResults(config_);
  test_result_->Print();

  return true;
}

bool RapidSenseTest::MoveToHubs_(std::vector<RapidSenseTestHubConfig>& hub_sequence) {
  bool no_interrupts = true;
  std::atomic_int hub_idx(1);

  ExtCodeSeqPair pair;
  ExtCode res;
  size_t seq_num;

  hub_sequence = config_.hub_sequence;
  for (auto& hub_config : hub_sequence) {
    hub_config.joint_configs.clear();
  }

  // subscribe to path and joint states - this is ugly, maybe we can find a
  // better way to do this
  std::vector<ros::Subscriber> js_subs, result_subs, feedback_subs;
  std::vector<uint32_t> edge_indices;

  for (const auto& observer : observers_) {
    RosController::TopicNames names = RosController::GetTopicNames(observer->GetName());

    auto buffer = BufferLastT<sensor_msgs::JointState>::MakePtr();

    auto buffer_latest_js = [buffer](const sensor_msgs::JointState::ConstPtr& msg) {
      buffer->add(*msg);
    };
    js_subs.push_back(
        nh_.subscribe<sensor_msgs::JointState>(names.joint_states, 10, buffer_latest_js));

    auto check_for_interrupts =
        [&no_interrupts](const rtr_control_ros::FollowJointPathActionResult::ConstPtr& msg) {
          if (msg->result.error_code != 0) {
            no_interrupts = false;
          }
        };
    result_subs.push_back(nh_.subscribe<rtr_control_ros::FollowJointPathActionResult>(
        names.follow_joint_path + "/result", 10, check_for_interrupts));

    // store approximation of key points in path
    edge_indices.push_back(0);
    uint32_t& old_edge_idx = edge_indices.back();
    auto store_key_points =
        [&old_edge_idx, &hub_idx, &hub_sequence,
         buffer](const rtr_control_ros::FollowJointPathActionFeedback::ConstPtr& msg) {
          if (msg->feedback.current_edgeid_idx != old_edge_idx) {
            sensor_msgs::JointState js;
            if (buffer->front(js)) {
              JointConfiguration config(js.position.size());
              for (size_t i = 0; i < config.Size(); ++i) {
                config[i] = js.position[i];
              }
              hub_sequence[hub_idx].joint_configs.push_back(config);
            }
          }
          old_edge_idx = msg->feedback.current_edgeid_idx;
        };
    feedback_subs.push_back(nh_.subscribe<rtr_control_ros::FollowJointPathActionFeedback>(
        names.follow_joint_path + "/feedback", 1, store_key_points));
  }

  for (auto it = hub_sequence.begin() + 1; it != hub_sequence.end(); ++it) {
    pair = app_commander_.MoveToHub(it->active_robot, it->state_space, it->hub_name, config_.speed);
    res = pair.first;
    seq_num = pair.second;
    if (res != ExtCode::SUCCESS) {
      RTR_ERROR("Failed to MoveToHub with error: {}", Convert<std::string>(ExtCode(res)));
      test_result_->result_ = RapidSenseTestResult::OPERATION_FAILURE;
      return false;
    }

    res = app_commander_.WaitForMove(seq_num);
    if (res != ExtCode::SUCCESS) {
      RTR_ERROR("Failed to WaitForMove with error: {}", Convert<std::string>(ExtCode(res)));
      test_result_->result_ = RapidSenseTestResult::OPERATION_FAILURE;
      return false;
    }

    hub_idx.fetch_add(1);
  }

  feedback_subs.clear();
  js_subs.clear();
  result_subs.clear();

  if (!no_interrupts) {
    RTR_ERROR("Test resulted in inconsistent path");
    test_result_->result_ = RapidSenseTestResult::PATH_FAILURE;
    return false;
  }

  return true;
}

RapidSenseTestResult::Ptr RapidSenseTest::GetResult() const {
  return test_result_;
}

std::string GetActiveDeconGroup() {
  rtr_msgs::DeconGroupInfo group_info;
  if (GetLoadedDeconGroup(group_info)) {
    return group_info.GroupName;
  }

  RTR_ERROR("Could not find loaded deconfliction group");
  return "";
}

bool GetLoadedDeconGroup(rtr_msgs::DeconGroupInfo& loaded_group) {
  ros::NodeHandle nh("");
  ros::ServiceClient decon_groups_client =
      nh.serviceClient<rtr_msgs::GetGroupInfo>("/GetDeconGroupInfo");
  ros::service::waitForService(decon_groups_client.getService());

  rtr_msgs::GetGroupInfo srv;
  if (!decon_groups_client.call(srv)) {
    RTR_ERROR("Failed to get deconfliction groups");
    return false;
  }

  for (const auto& group : srv.response.groups) {
    if (group.loaded && group.vision_enabled) {
      loaded_group = group;
      return true;
    }
  }

  return false;
}

bool LoadRapidPlanProjects(std::vector<RobotObserver::Ptr>& observers,
                           VoxelRegionDescription& region_desc) {
  rtr_msgs::DeconGroupInfo group_info;
  if (!GetLoadedDeconGroup(group_info)) {
    return false;
  }

  ros::NodeHandle nh("");
  ros::ServiceClient prj_info_client =
      nh.serviceClient<rtr_msgs::GetProjectROSInfo>("/GetProjectROSInfo");
  ros::service::waitForService(prj_info_client.getService());

  for (const auto& project_name : group_info.projects) {
    rtr_msgs::GetProjectROSInfo prj_info;
    prj_info.request.project_name = project_name;
    if (!prj_info_client.call(prj_info) || !prj_info.response.valid) {
      RTR_ERROR("Cannot get project info for {}", project_name);
      return false;
    }

    RapidPlanProject::Ptr project = RapidPlanProject::MakePtr();
    if (!project->Load(prj_info.response.project_path)) {
      RTR_ERROR("Failed to load project {}", project_name);
      return false;
    }

    VoxelRegion region = project->GetVoxelRegion();
    region_desc = VoxelRegionDescription(
        {DEFAULT_MPA_RESOLUTION, DEFAULT_MPA_RESOLUTION, DEFAULT_MPA_RESOLUTION},
        region.GetDimensions() / 2.0, region.GetPose().Inverse());

    RapidSenseRobotConfig config;
    config.name = project_name;
    config.project_path = prj_info.response.project_path;
    config.joint_topic = prj_info.response.joint_state_topic;
    config.status_topic = prj_info.response.robot_status_topic;
    config.joint_path_topic = prj_info.response.joint_path_topic;

    RobotConnection::Ptr connection = RosRobotConnection::MakePtr(nh, config);
    RobotObserver::Ptr observer = RobotObserver::MakePtr();
    if (!observer->Init(project, connection)) {
      return false;
    }
    observers.push_back(observer);
  }
  return true;
}

bool SetIgnoreVisionEnabledOnServer(const bool enable) {
  ros::NodeHandle nh("");
  rtr_msgs::GetSchemaMessage srv;
  ros::ServiceClient vision_client =
      nh.serviceClient<rtr_msgs::GetSchemaMessage>(RS::Topic("set_ignore_vision_enabled"));
  if (!ToSchemaMessageRequest(enable, srv.request) || !vision_client.call(srv)
      || !srv.response.is_success) {
    RTR_ERROR("Failed to set ignore vision enabled on RapidSense server");
    return false;
  }
  return true;
}

}  // namespace perception
}  // namespace rtr
