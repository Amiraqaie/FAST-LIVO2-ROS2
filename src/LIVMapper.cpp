/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <vikit/camera_loader.h>
#include <filesystem>

using namespace Sophus;
LIVMapper::LIVMapper(const rclcpp::Node::SharedPtr &mapper_node)
    : node(mapper_node),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);
  
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(this->node);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  path.header.stamp = this->node->now();
  path.header.frame_id = "odom";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  // declare parameters
  this->node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  this->node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  this->node->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  this->node->declare_parameter<int>("common.img_en", 1);
  this->node->declare_parameter<int>("common.lidar_en", 1);
  this->node->declare_parameter<std::string>("common.img_topic", "/left_camera/image");

  this->node->declare_parameter<bool>("vio.normal_en", true);
  this->node->declare_parameter<bool>("vio.inverse_composition_en", false);
  this->node->declare_parameter<int>("vio.max_iterations", 5);
  this->node->declare_parameter<int>("vio.img_point_cov", 100);
  this->node->declare_parameter<bool>("vio.raycast_en", false);
  this->node->declare_parameter<bool>("vio.exposure_estimate_en", true);
  this->node->declare_parameter<double>("vio.inv_expo_cov", 0.1);
  this->node->declare_parameter<int>("vio.grid_size", 5);
  this->node->declare_parameter<int>("vio.grid_n_height", 17);
  this->node->declare_parameter<int>("vio.patch_pyrimid_level", 4);
  this->node->declare_parameter<int>("vio.patch_size", 8);
  this->node->declare_parameter<int>("vio.outlier_threshold", 100);
  this->node->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  this->node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  this->node->declare_parameter<bool>("rolling_shutter.enable", false);
  this->node->declare_parameter<double>("rolling_shutter.tr_readout", 0.0);
  this->node->declare_parameter<double>("rolling_shutter.anchor_factor", 0.5);
  this->node->declare_parameter<bool>("rolling_shutter.debug_print", false);
  this->node->declare_parameter<bool>("rolling_shutter.td_estimate_enable", false);
  this->node->declare_parameter<double>("rolling_shutter.td_step_alpha", 0.1);
  this->node->declare_parameter<double>("rolling_shutter.td_step_cap", 0.0005);
  this->node->declare_parameter<double>("rolling_shutter.td_lambda", 100.0);
  this->node->declare_parameter<double>("rolling_shutter.td_min_jac_sq", 1.0);

  this->node->declare_parameter<std::string>("evo.seq_name", "01");
  this->node->declare_parameter<bool>("evo.pose_output_en", false);
  this->node->declare_parameter<double>("imu.gyr_cov", 1.0);
  this->node->declare_parameter<double>("imu.acc_cov", 1.0);
  this->node->declare_parameter<int>("imu.imu_int_frame", 10);
  this->node->declare_parameter<bool>("imu.imu_en", true);
  this->node->declare_parameter<bool>("imu.gravity_est_en", true);
  this->node->declare_parameter<bool>("imu.ba_bg_est_en", true);

  this->node->declare_parameter<double>("preprocess.blind", 0.01);
    this->node->declare_parameter<bool>("preprocess.hilti_en", false);
  this->node->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  this->node->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->node->declare_parameter<int>("preprocess.scan_line",6);
  this->node->declare_parameter<int>("preprocess.point_filter_num", 3);
  this->node->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  this->node->declare_parameter<int>("pcd_save.interval", -1);
  this->node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->node->declare_parameter<bool>("image_save.img_save_en", false);
  this->node->declare_parameter<int>("image_save.interval", 1);

  this->node->declare_parameter<int>("pcd_save.type", 0);
  this->node->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  this->node->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Pcl", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Rcl", vector<double>{});
  this->node->declare_parameter<double>("debug.plot_time", -10);
  this->node->declare_parameter<int>("debug.frame_cnt", 6);

  this->node->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  this->node->declare_parameter<bool>("publish.pub_effect_point_en", true);
  this->node->declare_parameter<bool>("publish.dense_map_en", false);

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  this->node->get_parameter("vio.img_point_cov", IMG_POINT_COV);
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.exposure_estimate_en", exposure_estimate_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  this->node->get_parameter("vio.outlier_threshold", outlier_threshold);
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  this->node->get_parameter("time_offset.lidar_time_offset", lidar_time_offset);
  this->node->get_parameter("rolling_shutter.enable", rolling_shutter_enable);
  this->node->get_parameter("rolling_shutter.tr_readout", rs_tr_readout);
  this->node->get_parameter("rolling_shutter.anchor_factor", rs_anchor_factor);
  this->node->get_parameter("rolling_shutter.debug_print", rs_debug_print);
  this->node->get_parameter("rolling_shutter.td_estimate_enable", td_estimate_enable);
  this->node->get_parameter("rolling_shutter.td_step_alpha", td_step_alpha);
  this->node->get_parameter("rolling_shutter.td_step_cap", td_step_cap);
  this->node->get_parameter("rolling_shutter.td_lambda", td_lambda);
  this->node->get_parameter("rolling_shutter.td_min_jac_sq", td_min_jac_sq);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node) 
{
  if (!(filter_size_surf_min > 0)) throw std::invalid_argument("preprocess.filter_size_surf must be positive");
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  
  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (img_en)
  {
    // vikit loads pinhole parameters by bare name and the other models by namespace.
    const auto &overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    for (const char *key : {"cam_model", "cam_width", "cam_height", "scale", "cam_fx", "cam_fy",
                            "cam_cx", "cam_cy", "cam_d0", "cam_d1", "cam_d2", "cam_d3",
                            "k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2"})
    {
      auto it = overrides.find(key);
      if (it == overrides.end()) continue;
      node->declare_parameter(key, it->second);
      node->declare_parameter(std::string("parameter_blackboard/") + key, it->second);
    }
    if (!vk::camera_loader::loadFromRosNs(this->node, "parameter_blackboard", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

    vio_manager->grid_size = grid_size;
    vio_manager->patch_size = patch_size;
    vio_manager->outlier_threshold = outlier_threshold;
    vio_manager->setImuToLidarExtrinsic(extT, extR);
    vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
    vio_manager->state = &_state;
    vio_manager->state_propagat = &state_propagat;
    vio_manager->max_iterations = max_iterations;
    vio_manager->img_point_cov = IMG_POINT_COV;
    vio_manager->normal_en = normal_en;
    vio_manager->inverse_composition_en = inverse_composition_en;
    vio_manager->raycast_en = raycast_en;
    vio_manager->grid_n_width = grid_n_width;
    vio_manager->grid_n_height = grid_n_height;
    vio_manager->patch_pyrimid_level = patch_pyrimid_level;
    vio_manager->exposure_estimate_en = exposure_estimate_en;
    vio_manager->colmap_output_en = colmap_output_en;
    vio_manager->rs_enable = rolling_shutter_enable;
    vio_manager->rs_tr_readout = rs_tr_readout;
    vio_manager->rs_anchor_factor = rs_anchor_factor;
    vio_manager->rs_debug_print = rs_debug_print;
    vio_manager->td_estimate_enable = td_estimate_enable;
    vio_manager->td_step_alpha = td_step_alpha;
    vio_manager->td_step_cap = td_step_cap;
    vio_manager->td_lambda = td_lambda;
    vio_manager->td_min_jac_sq = td_min_jac_sq;
    vio_manager->initializeVIO();
  }

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D::Constant(node->declare_parameter<double>("imu.b_gyr_cov", 0.0001)));
  p_imu->set_acc_bias_cov(V3D::Constant(node->declare_parameter<double>("imu.b_acc_cov", 0.0001)));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en) std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/pcd");
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers()
{
  if (p_pre->lidar_type == AVIA) {
#if defined(USE_LIVOX_ROS_DRIVER2) && defined(USE_LIVOX_INTERFACES)
    // Both livox msg packages available — use livox_msg_type param to select
    // Options: "livox_ros_driver2" (default), "livox_interfaces" (legacy driver)
    std::string livox_msg_type = "livox_ros_driver2";
    this->node->declare_parameter<std::string>("common.livox_msg_type", "livox_ros_driver2");
    this->node->get_parameter("common.livox_msg_type", livox_msg_type);

    if (livox_msg_type == "livox_interfaces") {
      sub_pcl = this->node->create_subscription<livox_interfaces::msg::CustomMsg>(lid_topic, 200000, std::bind(&LIVMapper::livox_interfaces_pcl_cbk, this, std::placeholders::_1));
      RCLCPP_INFO(this->node->get_logger(), "Using livox_interfaces::msg::CustomMsg for '%s'", lid_topic.c_str());
    } else {
      sub_pcl = this->node->create_subscription<livox_custom_msg::CustomMsg>(lid_topic, 200000, std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
      RCLCPP_INFO(this->node->get_logger(), "Using livox_ros_driver2::msg::CustomMsg for '%s'", lid_topic.c_str());
    }
#else
    sub_pcl = this->node->create_subscription<livox_custom_msg::CustomMsg>(lid_topic, 200000, std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
#endif
  } else {
    sub_pcl = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 200000, std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  sub_imu = this->node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(img_topic, 200000, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
  
  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubBody = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", rclcpp::QoS(1).transient_local());
  map_timer = node->create_wall_timer(std::chrono::seconds(1), std::bind(&LIVMapper::publish_map, this));
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  pubState = node->create_publisher<fast_livo::msg::LioState>("/lio_state", 20);
  pubScaledImu = node->create_publisher<sensor_msgs::msg::Imu>("~/imu_si", rclcpp::SensorDataQoS());
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (!pcl_w_wait_pub || pcl_w_wait_pub->empty())
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  if (vio_manager->rs_enable)
  {
    const double img_t = LidarMeasures.measures.back().vio_time;
    V3D omg_b = V3D::Zero();
    V3D acc_b = V3D::Zero();
    double t_used = 0.0;
    const char *src = "none";

    if (rs_imu_prev_t > 0.0 && rs_imu_next_t > rs_imu_prev_t + 1e-9)
    {
      // proper bracketing interpolation
      const double alpha = (img_t - rs_imu_prev_t) / (rs_imu_next_t - rs_imu_prev_t);
      omg_b << rs_imu_prev.angular_velocity.x + alpha * (rs_imu_next.angular_velocity.x - rs_imu_prev.angular_velocity.x),
               rs_imu_prev.angular_velocity.y + alpha * (rs_imu_next.angular_velocity.y - rs_imu_prev.angular_velocity.y),
               rs_imu_prev.angular_velocity.z + alpha * (rs_imu_next.angular_velocity.z - rs_imu_prev.angular_velocity.z);
      acc_b << rs_imu_prev.linear_acceleration.x + alpha * (rs_imu_next.linear_acceleration.x - rs_imu_prev.linear_acceleration.x),
               rs_imu_prev.linear_acceleration.y + alpha * (rs_imu_next.linear_acceleration.y - rs_imu_prev.linear_acceleration.y),
               rs_imu_prev.linear_acceleration.z + alpha * (rs_imu_next.linear_acceleration.z - rs_imu_prev.linear_acceleration.z);
      t_used = img_t;
      src = "interp";
    }
    else if (rs_imu_prev_t > 0.0)
    {
      // one-sided: only prev available
      omg_b << rs_imu_prev.angular_velocity.x, rs_imu_prev.angular_velocity.y, rs_imu_prev.angular_velocity.z;
      acc_b << rs_imu_prev.linear_acceleration.x, rs_imu_prev.linear_acceleration.y, rs_imu_prev.linear_acceleration.z;
      t_used = rs_imu_prev_t;
      src = "prev";
    }
    else
    {
      // fallback to newest_imu
      omg_b << newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z;
      acc_b << newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z;
      t_used = stamp2Sec(newest_imu.header.stamp);
      src = "newest";
    }

    vio_manager->rs_omega_b = omg_b - _state.bias_g;
    vio_manager->rs_accel_w = _state.rot_end * (acc_b - _state.bias_a) + _state.gravity;

    if (rs_debug_print)
    {
      const double t_newest = stamp2Sec(newest_imu.header.stamp);
      RCLCPP_INFO(this->node->get_logger(),
        "[ RS ] img_t=%.6f  src=%s  imu_used_t=%.6f (Δ=%+.4f ms)  prev_t=%.6f next_t=%.6f  newest_t=%.6f (Δ=%+.4f ms)  |ω|=%.4f  |a_w|=%.4f",
        img_t, src, t_used, (t_used - img_t) * 1e3,
        rs_imu_prev_t, rs_imu_next_t,
        t_newest, (t_newest - img_t) * 1e3,
        vio_manager->rs_omega_b.norm(), vio_manager->rs_accel_w.norm());
    }
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (vio_manager->td_estimate_enable && vio_manager->td_step_pending != 0.0)
  {
    const double old_offset = img_time_offset;
    img_time_offset += vio_manager->td_step_pending;
    if (rs_debug_print)
    {
      RCLCPP_INFO(this->node->get_logger(),
        "[ TD ] step=%+.4f ms  img_time_offset: %.6f -> %.6f  (grad=%.4f  jac^2=%.4f)",
        vio_manager->td_step_pending * 1e3,
        old_offset, img_time_offset,
        vio_manager->td_last_grad_sum, vio_manager->td_last_jac_sq_sum);
    }
    vio_manager->td_step_pending = 0.0;
  }

  publish_lio_state();

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (!feats_undistort || feats_undistort->empty())
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  publish_lio_state();

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  const Eigen::Quaterniond orientation(_state.rot_end);
  geoQuat.x = orientation.x();
  geoQuat.y = orientation.y();
  geoQuat.z = orientation.z();
  geoQuat.w = orientation.w();
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;
  if (pubBody->get_subscription_count())
  {
    PointCloudXYZI body(size, 1);
    for (int i = 0; i < size; ++i)
      RGBpointBodyLidarToIMU(&laserCloudFullRes->points[i], &body[i]);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(body, msg);
    msg.header = odomAftMapped.header;
    msg.header.frame_id = "livox_frame";
    pubBody->publish(msg);
  }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  publish_path(pubPath);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run()
{
  rclcpp::Rate rate(5000);
  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(this->node);
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = stamp2Sec(msg->header.stamp) + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(),"lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_custom_msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_custom_msg::CustomMsg::SharedPtr msg(new livox_custom_msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  RCLCPP_INFO(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

#if defined(USE_LIVOX_ROS_DRIVER2) && defined(USE_LIVOX_INTERFACES)
void LIVMapper::livox_interfaces_pcl_cbk(const livox_interfaces::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  // Convert livox_interfaces::msg::CustomMsg to livox_ros_driver2::msg::CustomMsg
  auto converted = std::make_shared<livox_custom_msg::CustomMsg>();
  converted->header = msg_in->header;
  converted->timebase = msg_in->timebase;
  converted->point_num = msg_in->point_num;
  converted->lidar_id = msg_in->lidar_id;
  converted->rsvd = msg_in->rsvd;
  converted->points.resize(msg_in->points.size());
  for (size_t i = 0; i < msg_in->points.size(); ++i) {
    converted->points[i].offset_time = msg_in->points[i].offset_time;
    converted->points[i].x = msg_in->points[i].x;
    converted->points[i].y = msg_in->points[i].y;
    converted->points[i].z = msg_in->points[i].z;
    converted->points[i].reflectivity = msg_in->points[i].reflectivity;
    converted->points[i].tag = msg_in->points[i].tag;
    converted->points[i].line = msg_in->points[i].line;
  }
  livox_pcl_cbk(converted);
}
#endif

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  RCLCPP_INFO(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->node->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  {
    RCLCPP_WARN(this->node->get_logger(), "imu time stamp Jumps %0.4lf seconds, resetting IMU timeline\n", timestamp - last_timestamp_imu);
    imu_buffer.clear();
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  newest_imu = *msg;
  if (!p_imu->imu_need_init && pubScaledImu->get_subscription_count())
  {
    auto scaled = *msg;
    const double scale = G_m_s2 / p_imu->IMU_mean_acc_norm;
    scaled.linear_acceleration.x *= scale;
    scaled.linear_acceleration.y *= scale;
    scaled.linear_acceleration.z *= scale;
    if (scaled.linear_acceleration_covariance[0] >= 0)
      for (auto &entry : scaled.linear_acceleration_covariance) entry *= scale * scale;
    pubScaledImu->publish(scaled);
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

// static int i = 0;
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  RCLCPP_INFO(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    RCLCPP_WARN(this->node->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      const double rs_bulk_shift = rolling_shutter_enable ? (0.5 - rs_anchor_factor) * rs_tr_readout : 0.0;
      double img_capture_time = img_time_buffer.front() + exposure_time_init + rs_bulk_shift;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_ERROR(this->node->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // RCLCPP_ERROR(this->node->get_logger(), "lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      rs_imu_prev_t = -1.0;
      rs_imu_next_t = -1.0;
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time)
        {
          rs_imu_next = *imu_buffer.front();
          rs_imu_next_t = stamp2Sec(imu_buffer.front()->header.stamp);
          break;
        }

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        rs_imu_prev = *imu_buffer.front();
        rs_imu_prev_t = stamp2Sec(imu_buffer.front()->header.stamp);

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      const double rs_bulk_shift = rolling_shutter_enable ? (0.5 - rs_anchor_factor) * rs_tr_readout : 0.0;
      double img_capture_time = img_time_buffer.front() + exposure_time_init + rs_bulk_shift;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = stamp2Sec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = stamp2Sec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   stamp2Sec(imu_buffer.front()->header.stamp));
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
}

void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  const bool camera_update = LidarMeasures.lio_vio_flg == VIO;
  const bool finished_scan = camera_update || slam_mode_ != LIVO;
  pcl::PointCloud<MapPoint> scan;
  scan.reserve(pcl_w_wait_pub->size());
  // Only the nearest LiDAR return in each camera pixel can sample its color.
  struct Projection { size_t index; V2D uv; double depth; };
  std::vector<Projection> projections;
  std::vector<uint32_t> colors;
  constexpr uint32_t no_color = 0xffffffffu;
  if (camera_update && vio_manager->new_frame_ && vio_manager->img_rgb.type() == CV_8UC3 &&
      !vio_manager->img_rgb.empty())
  {
    const auto &img = vio_manager->img_rgb;
    cv::Mat1d depth(img.rows, img.cols, std::numeric_limits<double>::infinity());
    colors.assign(pcl_w_wait_pub->size(), no_color);
    for (size_t i = 0; i < pcl_w_wait_pub->size(); ++i)
    {
      const auto &p = (*pcl_w_wait_pub)[i];
      const V3D world(p.x, p.y, p.z);
      const V3D camera = vio_manager->new_frame_->w2f(world);
      if (camera.z() <= 0 || camera.norm() <= blind_rgb_points) continue;
      const V2D uv = vio_manager->new_frame_->w2c(world);
      if (!uv.allFinite() || uv.x() < 1 || uv.y() < 1 ||
          uv.x() >= img.cols - 1 || uv.y() >= img.rows - 1) continue;
      double &nearest = depth(int(uv.y()), int(uv.x()));
      nearest = std::min(nearest, camera.z());
      projections.push_back({i, uv, camera.z()});
    }
    for (const auto &projection : projections)
    {
      const V2D &uv = projection.uv;
      if (projection.depth != depth(int(uv.y()), int(uv.x()))) continue;
      const V3F bgr = vio_manager->getInterpolatedPixel(img, uv);
      colors[projection.index] = (uint32_t(bgr[2]) << 16) | (uint32_t(bgr[1]) << 8) | uint32_t(bgr[0]);
    }
  }
  for (size_t i = 0; i < pcl_w_wait_pub->size(); ++i)
  {
    const auto &p = (*pcl_w_wait_pub)[i];
    if (!p.getVector3fMap().allFinite()) continue;
    MapPoint point{p.x, p.y, p.z, p.intensity, p.normal_x, p.normal_y, p.normal_z,
                   p.curvature, 0x808080, 0};
    const VOXEL_LOCATION key(std::floor(p.x / filter_size_surf_min),
                             std::floor(p.y / filter_size_surf_min),
                             std::floor(p.z / filter_size_surf_min));
    auto &stored = display_map.try_emplace(key, point).first->second;
    if (!colors.empty() && colors[i] != no_color)
    {
      point.rgb = colors[i];
      point.has_color = 1;
      stored = point;
    }
    if (finished_scan)
    {
      point.rgb = stored.rgb;
      point.has_color = stored.has_color;
      scan.push_back(point);
      if (pcd_save_en && pcd_save_type == 0 && slam_mode_ == LIVO)
      {
        PointTypeRGB saved;
        saved.x = p.x;
        saved.y = p.y;
        saved.z = p.z;
        saved.rgba = point.rgb | 0xff000000u;
        laserCloudWorldRGB->push_back(saved);
      }
    }
  }
  map_dirty = true;
  if (finished_scan)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(scan, msg);
    msg.header = odomAftMapped.header;
    pubLaserCloudFullRes->publish(msg);
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if (finished_scan) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_map()
{
  if (!map_dirty || !pubLaserCloudMap->get_subscription_count()) return;
  pcl::PointCloud<MapPoint> map;
  map.reserve(display_map.size());
  for (const auto &entry : display_map) map.push_back(entry.second);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(map, msg);
  msg.header = odomAftMapped.header;
  pubLaserCloudMap->publish(msg);
  map_dirty = false;
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = odomAftMapped.header.stamp;
  laserCloudFullRes3.header.frame_id = "odom";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "odom";
  odomAftMapped.child_frame_id = "livox_frame";
  odomAftMapped.header.stamp = sec2Stamp(LidarMeasures.measures.back().lio_time);
  set_posestamp(odomAftMapped.pose.pose);
  const V3D velocity = _state.rot_end.transpose() * _state.vel_end;
  auto &twist = odomAftMapped.twist.twist;
  twist.linear.x = velocity.x();
  twist.linear.y = velocity.y();
  twist.linear.z = velocity.z();
  twist.angular.x = p_imu->unbiased_gyr.x();
  twist.angular.y = p_imu->unbiased_gyr.y();
  twist.angular.z = p_imu->unbiased_gyr.z();
  const M3D pv = _state.rot_end.transpose() * _state.cov.block<3,3>(7,7) * _state.rot_end;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      odomAftMapped.pose.covariance[6*i+j] = _state.cov((i+3)%6, (j+3)%6);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      odomAftMapped.twist.covariance[6*i+j] = pv(i,j);
      odomAftMapped.twist.covariance[6*(i+3)+j+3] = _state.cov(10+i,10+j) + (i == j ? gyr_cov : 0);
    }
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = odomAftMapped.header.stamp;
  path.header.stamp = msg_body_pose.header.stamp;
  msg_body_pose.header.frame_id = "odom";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}

void LIVMapper::publish_lio_state()
{
  if (!imu_en || p_imu->imu_need_init) return;
  fast_livo::msg::LioState msg;
  msg.header.frame_id = "odom";
  const auto &measure = LidarMeasures.measures.back();
  msg.header.stamp = sec2Stamp(LidarMeasures.lio_vio_flg == VIO ? measure.vio_time : measure.lio_time);
  Eigen::Map<V3D>(msg.pos.data()) = _state.pos_end;
  Eigen::Map<V3D>(msg.vel.data()) = _state.vel_end;
  Eigen::Map<V3D>(msg.bg.data()) = _state.bias_g;
  Eigen::Map<V3D>(msg.ba.data()) = _state.bias_a;
  Eigen::Map<V3D>(msg.grav.data()) = _state.gravity;
  Eigen::Map<V3D>(msg.offset_t_l_i.data()) = extT;
  Eigen::Map<Eigen::Vector4d>(msg.rot.data()) = Eigen::Quaterniond(_state.rot_end).coeffs();
  Eigen::Map<Eigen::Vector4d>(msg.offset_r_l_i.data()) = Eigen::Quaterniond(extR).coeffs();

  // FAST-LIO estimates extrinsics and uses IKFoM's type-1 S2 gravity chart.
  // FAST-LIVO2 holds extrinsics fixed and represents gravity in Cartesian coordinates.
  Eigen::Matrix<double,23,DIM_STATE> J = Eigen::Matrix<double,23,DIM_STATE>::Zero();
  J.block<3,3>(0,3).setIdentity();
  J.block<3,3>(3,0).setIdentity();
  J.block<9,9>(12,7).setIdentity();
  const double norm = _state.gravity.norm();
  if (norm > 1e-6)
  {
    const V3D n = _state.gravity / norm;
    Eigen::Matrix<double,3,2> basis;
    if (1 + n.x() > 1e-10)
    {
      basis.row(0) = -n.tail<2>().transpose();
      basis.bottomRows<2>() = Eigen::Matrix2d::Identity() -
          n.tail<2>() * n.tail<2>().transpose() / (1 + n.x());
    }
    else
    {
      basis.col(0) = V3D::UnitZ();
      basis.col(1) = -V3D::UnitY();
    }
    for (int i = 0; i < 2; ++i)
      J.block<1,3>(21+i,16) = basis.col(i).cross(n).transpose() / norm;
  }
  Eigen::Map<Eigen::Matrix<double,23,23,Eigen::RowMajor>>(msg.cov.data()) =
      J * _state.cov * J.transpose();
  pubState->publish(msg);
}
