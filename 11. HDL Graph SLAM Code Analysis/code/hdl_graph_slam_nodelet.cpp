// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <hdl_graph_slam/SaveMap.h>
#include <hdl_graph_slam/DumpGraph.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

namespace hdl_graph_slam {

class HdlGraphSlamNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> ApproxSyncPolicy;

  HdlGraphSlamNodelet() {}
  virtual ~HdlGraphSlamNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    //MTNodeHandle은 콜백 함수를 처리하기 위한 스레드를 생성
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity();

    max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

    //param 정리
    //nullptr > C++11부터는 nullptr을 가지고 초기화 (원래는 0으로 초기화)
    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    //GPS > nmea는 시간, 위치, 방위 등의 정보를 전송하기 위한 규격, parser > 그것을 이루고 있는 구성 성분으로 분해하고 관계를 분석하여 문장의 구조를 결정하는 것 
    nmea_parser.reset(new NmeaSentenceParser());
  
    gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z = private_nh.param<double>("gps_edge_stddev_z", 10.0);
    floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);

    imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);
    enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
    enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false);
    imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1);
    imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0);

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/odom", 256));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 32));
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *odom_sub, *cloud_sub));
    sync->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2));
    imu_sub = nh.subscribe("/gpsimu_driver/imu_data", 1024, &HdlGraphSlamNodelet::imu_callback, this);
    floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024, &HdlGraphSlamNodelet::floor_coeffs_callback, this);

    //만약, gps를 사용한다면
    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &HdlGraphSlamNodelet::gps_callback, this);
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &HdlGraphSlamNodelet::nmea_callback, this);
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &HdlGraphSlamNodelet::navsat_callback, this);
    }

    // publishers
    markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1, true);
    read_until_pub = mt_nh.advertise<std_msgs::Header>("/hdl_graph_slam/read_until", 32);

    //pub이지만 service > 요청시 pub
    //dump > 어떤 장애가 발생했을 때, 오류를 복사해서 가지고오는 역할
    dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);

    graph_updated = false;
    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    //optimization timer callback 함수가 그래프 최적화 수행 > 1. loop detector 2. 단순화된 keyframe 생성
    optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this);
  }

private:
  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * 수신된 포인트 클라우드는 #keyframe_queue로 푸시
   * @param odom_msg
   * @param cloud_msg
   */
  
  //이 기능은 주로 odometry 정보를 cloud 정보와 동기화 하는 것
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    const ros::Time& stamp = cloud_msg->header.stamp;
    //odometry정보를 Eigen::Isometry3d로 변환
    Eigen::Isometry3d odom = odom2isometry(odom_msg);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if(base_frame_id.empty()) {
      base_frame_id = cloud_msg->header.frame_id;
    }

    //key frame updater가 odom을 업데이트 하지않은 경우
    if(!keyframe_updater->update(odom)) {
      //std::lock_guard > std::mutex사용할 때 제공되는 것으로 리소스의 사용 scope가 끝날 경우 획득한 자원을 자동으로 해제해서 leak을 방지하는 디자인 패턴을 제공하는 class
      //template으로 mutex객체를 전달 받아서 생성자에서 lock()함수를 호출하고, 소멸자에서 unlock()함수를 호출
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }

      return;
    }

    //accum_d = key frame에 의해 누적된 거리 
    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    //keyframe queue에 keyframe을 넣음
    keyframe_queue.push_back(keyframe);
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * 이 메서드는 #keyframe_queue의 모든 키프레임을 포즈 그래프(오도메트리 가장자리)에 추가
   * @return if true, at least one keyframe was added to the pose graph
   * true인 경우 포즈 그래프에 하나 이상의 키프레임이 추가
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    for(int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframes will be tested later for loop closure
      // new_keyframes는 loop closure를 위해 나중에 테스트
      new_keyframes.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      //포인터 키프레임 변수 값을 노드가 참조 = 그래프 슬램 포인터 변수 값을 add se3 node가 참조
      keyframe->node = graph_slam->add_se3_node(odom);
      //std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;
      //unordered_map > map보다 더 빠른 탐색을 하기 위한 자료구조로 unordered_map은 해쉬테이블로 구현한 자료구조로 탐색 시간복잡도는 O(1)
      //hash함수 > 멤버 함수는 저장된 해시 함수 개체를 반환
      keyframe_hash[keyframe->stamp] = keyframe;

      // fix the first node
      //첫번째 노드 픽스 > 키프레임 비어있고 새로운 키프레임 들어온다는 말은 첫번째 노드가 들어왔다는 말임 
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(private_nh.param<bool>("fix_first_node", false)) {
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
          std::stringstream sst(private_nh.param<std::string>("fix_first_node_stddev", "1 1 1 1 1 1"));
          for(int i = 0; i < 6; i++) {
            double stddev = 1.0;
            sst >> stddev;
            inf(i, i) = 1.0 / stddev;
          }

          anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
        }
      }

      if(i == 0 && keyframes.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      //연속 키프레임 사이에 edge 추가
      //loop closure edge 제약 조건은 laser odometry edge 제약 조건과 동일
      //⭐Laser odometry constraints⭐
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      //pose graph에서 관측값과 예측값은 상대포즈로 나타내므로 e=z-1*z임 원래
      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
    }

    std_msgs::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub.publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
    return true;
  }

  //gps 정보 구독
  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    // -> 는 포인터 변수의 값을 참조하기 위한 연산자로 nmea_parser는 ptr임 맨밑에 정의 ㅇㅇ
    // 즉, nmea_parser 포인터 변수의 값을 parse 멤버 변수가 참조 (nmea_msg 포인터 변수의 값을 sentence가 참조)
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if(grmc.status != 'A') {
      return;
    }

    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    //여기서도 gps_msg가 ptr이라서 -> 로 간접적으로 접근
    //grmc는 포인터가 아닌 멤버 변수므로 .으로 직접적으로 접근
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    gps_callback(gps_msg);
  }

  /**
   * @brief received gps data is added to #gps_queue
   * 수신된 GPS 데이터는 #gps_queue에 추가
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_msg->header.stamp += ros::Duration(gps_time_offset);
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  //Pose_graph에 GPS 제약 조건 추가 1단계: 먼저 GPS 위도와 경도를 UTM 좌표계로 변환
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    //keyframe이 gps queue의 뒤에서 시작하면 멈추고
    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > gps_queue.back()->header.stamp) {
        break;
      }

      //keyframe이 gps queue의 앞에서 시작하면 고
      if(keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      //키프레임에 가장 가까운 GPS 데이터 찾기
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
        auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
        //실제 gps-keyframe좌표가 closest gps-keyframe좌표보다 크면 멈춰라
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      //GPS와 키프레임 사이의 잔여 시간이 너무 크면 건너뛰기
      gps_cursor = closest_gps;
      if(0.2 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      //(위도, 경도, 고도) -> UTM좌표의 (동향, 북향, 고도)로 변환
      //GPS를 UTM 좌표계로 변환
      geodesy::UTMPoint utm;
      geodesy::fromMsg((*closest_gps)->position, utm);
      Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

      // the first gps data position will be the origin of the map
      //첫 번째 GPS 데이터 위치는 지도의 원점
      //먼저 초기 원점 좌표를 정의
      if(!zero_utm) {
        zero_utm = xyz;
      }
      //모든 후속 전역 좌표 - 원점 좌표 = 로컬 좌표
      //초기 오프셋을 뺌(즉, 초기 GPS위치)
      xyz -= (*zero_utm);

      //각 키 프레임은 실제로 GPS 포즈 정보 프레임에 해당
      //키 프레임에 GPS 위치 추가
      keyframe->utm_coord = xyz;

      //GPS에 높이 값이 있는지 여부에 따라 확률 그래프 edge 추가
      g2o::OptimizableGraph::Edge* edge;
      if(std::isnan(xyz.z())) {
        //여기서 gps의 정보 행렬은 상수 값이고 gps_edge_stddev는 기본적으로 1000 또는 100임을 알 수 있음
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
        //여기에서는 xy만 제한되며, 제한된 정보 행렬은 1/100 또는 1/1000으로 상대적으로 작음
        edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
      } else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
        information_matrix(2, 2) /= gps_edge_stddev_z;
        edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      }
      //견고성 증가
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) { return stamp < geopoint->header.stamp; });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }

  void imu_callback(const sensor_msgs::ImuPtr& imu_msg) {
    if(!enable_imu_orientation && !enable_imu_acceleration) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    imu_msg->header.stamp += ros::Duration(imu_time_offset);
    imu_queue.push_back(imu_msg);
  }

  //hdl은 주로 자세 각도와 중력 가속도의 방향을 제한하기 위해 imu 정보를 사용
  //실시간으로 imu 데이터를 수집하여 캐시 큐에 넣음
  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if(keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > imu_queue.back()->header.stamp) {
        break;
      }

      if(keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      //키프레임에 가장 가까운 imu 데이터 찾기
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }

      //가장 가까운 타임스탬프가 있는 키프레임을 찾기 위해 imu queue 탐색
      //자세 각도와 선형 가속도를 얻음
      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      geometry_msgs::Vector3Stamped acc_imu;
      geometry_msgs::Vector3Stamped acc_base;
      geometry_msgs::QuaternionStamped quat_imu;
      geometry_msgs::QuaternionStamped quat_base;
    
      quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
      quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
      acc_imu.vector = (*closest_imu)->linear_acceleration;
      quat_imu.quaternion = (*closest_imu)->orientation;

      //imu 위치를 기본 좌표계로 변환
      try {
        tf_listener.transformVector(base_frame_id, acc_imu, acc_base);
        tf_listener.transformQuaternion(base_frame_id, quat_imu, quat_base);
      //현재 오류의 일반적인 원인을 설명하는 C 스타일 문자열을 반환
      } catch(std::exception& e) {
        std::cerr << "failed to find transform!!" << std::endl;
        return false;
      }

      keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
      keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z);
      keyframe->orientation = keyframe->orientation;
      //imu의 과거 키 프레임에 해당하는 최신 일치 데이터 프레임을 가져오고 정적 tf 변환에 따라 기본 프레임 좌표계의 자세 및 가속도 정보로 변환
      if(keyframe->orientation->w() < 0.0) {
        keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
      }

      //포즈 제약 조건 추가
      if(enable_imu_orientation) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
        auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
      }

      //가속도 제약 조건 추가
      if(enable_imu_acceleration) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
        g2o::OptimizableGraph::Edge* edge = graph_slam->add_se3_prior_vec_edge(keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_acceleration_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_acceleration_edge_robust_kernel_size", 1.0));
      }
      updated = true;
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& imu) { return stamp < imu->header.stamp; });
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * 수신된 바닥 계수는 #floor_coeffs_queue에 추가
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const hdl_graph_slam::FloorCoeffsConstPtr& floor_coeffs_msg) {
    if(floor_coeffs_msg->coeffs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * 이 메서드는 바닥 계수 메시지를 등록된 키프레임과 연결한 다음 연결된 계수를 포즈 그래프에 추가
   * @return if true, at least one floor plane edge is added to the pose graph
   * true이면 포즈 그래프에 최소 하나의 바닥 평면 edge 추가
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    //처리를 위해 각 프레임에서 감지된 평면 매개변수 정보를 대기열 버퍼에 넣음
    //constrained edges 추가
    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue) {
      if(floor_coeffs->header.stamp > latest_keyframe_stamp) {
        break;
      }

      //타임스탬프에 해당하는 키 프레임 찾기
      auto found = keyframe_hash.find(floor_coeffs->header.stamp);
      if(found == keyframe_hash.end()) {
        continue;
      }

      //각 평면 매개변수를 탐색하고 타임스탬프에 따라 해당 키프레임을 찾아야 함
      //그 목적은 키프레임 포즈를 제한하는 것
      //plane node 초기화
      if(!floor_plane_node) {
        floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        floor_plane_node->setFixed(true);
      }

      //constrain될 첫 번째 평면이 초기값, 즉 고정 평면으로 사용
      //모든 후속 평면은 첫 번째 프레임 평면을 ground plane information reference로 사용
      const auto& keyframe = found->second;

      //plane constraints 추가
      //평면 제약 조건 추가는 0.1로 설정해야 하며 일부 설정은 100
      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      //평면 constration 추가
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("floor_edge_robust_kernel", "NONE"), private_nh.param<double>("floor_edge_robust_kernel_size", 1.0));

      keyframe->floor_coeffs = coeffs;

      updated = true;
      //그 중 plane constraint edge는 g2o에 미리 정의된 정의가 있으므로 생성할 필요가 없음
      //구성된 constrained edge는 평면과 고정 vertex(노드(와 비슷)) 사이의 edge
    }

    //upper bound = 원하는 값 k를 초과한 값이 처음 나오는 위치를 찾는 과정
    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp, [=](const ros::Time& stamp, const hdl_graph_slam::FloorCoeffsConstPtr& coeffs) { return stamp < coeffs->header.stamp; });
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate map point cloud and publish it
   * 맵 포인트 클라우드 생성 및 게시
   * @param event
   */
  void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
    if(!map_points_pub.getNumSubscribers() || !graph_updated) {
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub.publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * 이 메서드는 대기열의 모든 데이터를 포즈 그래프에 추가한 다음 포즈 그래프를 최적화
   * @param event
   */
  void optimization_timer_callback(const ros::WallTimerEvent& event) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    // 포즈 그래프에 대기열의 키프레임 및 바닥 계수 추가 = 키 프레임 대기열 처리
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated) {
      std_msgs::Header read_until;
      read_until.stamp = ros::Time::now() + ros::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);
    }
    
    // 4가지 센서 데이터에 대한 처리 결과가 없으면 최적화할 필요가 없음
    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue()) {
      return;
    }

    // loop detection
    // 여기서 detect함수는 평면 감지 nodelet.cpp에 있는 함수임

    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐잘 모르겠음⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
    // 근데 이게 왜 loop detector에서 사용되는지 모르겠음...
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

    //닫힌 루프가 있는 경우 확률 그래프에 닫힌 루프 제약 조건을 추가
    //여기서 for(:)은 JAVA구문인데, 왜 C++에서 ? 무튼, for(A:B) > B에서 차례대로 객체를 꺼내서 A에게 넣겠다는 의미 
    for(const auto& loop : loops) {
      //relpose > 확률적 상대 회전 추정 : 여기 relpose는 닫힌 루프 프레임 간의 포즈 제약 조건을 나타냄
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      //포즈의 동적 가중치, 각도 계산
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
      //닫힌 루프의 제한된 edge를 추가
      auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
    }

    //copy의 경우 new_keyframe을 키프레임에 복사하고 copy 키프레임이 있는 폐쇄 루프 감지의 경우 new_keyframes를 복사
    //back_inserter > x의 끝에 새로운 요소를 삽입
    //back-insert iterator는 일반적으로 요소(예: 복사)를 덮어쓰는 알고리즘이 대신 컨테이너 끝에 새 요소를 자동으로 삽입할 수 있도록 설계된 특수한 유형의 출력 반복기
    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    //new 키 프레임 지우기
    //vector clear는 내부에 들어있는 값을 모두 없애줌
    new_keyframes.clear();

    // move the first node anchor position to the current estimate of the first node pose
    // 첫 번째 노드 앵커 위치를 첫 번째 노드 포즈의 현재 추정치로 이동
    // so the first node moves freely while trying to stay around the origin
    // 첫 번째 노드가 원점 주위에 머물려고 하는 동안 자유롭게 이동
    if(anchor_node && private_nh.param<bool>("fix_first_node_adaptive", true)) {
      Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    // optimize the pose graph
    // pose graph 최적화
    int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);
    graph_slam->optimize(num_iterations);

    // publish tf
    // tf 게시 
    // 그래프 최적화를 위해 g2o를 호출
    const auto& keyframe = keyframes.back();
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    //최신 프레임 키프레임의 최적화된 포즈를 주행 거리계로 게시
    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    //맵 게시를 위해 최적화된 결과를 각 최적화에 대해 keyframes_snapshot에 복사
    //exchange 메소드는 keyframes_snapshot 이전의 데이터 메모리가 재활용
    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
    graph_updated = true;

    if(odom2map_pub.getNumSubscribers()) {
      geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
      odom2map_pub.publish(ts);
    }

    if(markers_pub.getNumSubscribers()) {
      auto markers = create_marker_array(ros::Time::now());
      markers_pub.publish(markers);
    }
  }

  /**
   * @brief create visualization marker
   * 시각화 마커 생성
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(4);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    visualization_msgs::Marker& imu_marker = markers.markers[1];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 1;
    imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes.size());
    traj_marker.colors.resize(keyframes.size());
    for(int i = 0; i < keyframes.size(); i++) {
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes.size();
      traj_marker.colors[i].r = 1.0 - p;
      traj_marker.colors[i].g = p;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes[i]->acceleration) {
        Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
        geometry_msgs::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }

    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[2];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
        edge_marker.colors[i * 2].r = 1.0 - p1;
        edge_marker.colors[i * 2].g = p1;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0 - p2;
        edge_marker.colors[i * 2 + 1].g = p2;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i * 2].z += 0.5;
          edge_marker.points[i * 2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].b = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].b = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge);
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
      if(edge_priori_xyz) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xyz->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = 3;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    if(!keyframes.empty()) {
      Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
      sphere_marker.pose.position.x = pos.x();
      sphere_marker.pose.position.y = pos.y();
      sphere_marker.pose.position.z = pos.z();
    }
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 0.3;

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * 모든 데이터를 현재 디렉토리에 덤프
   * dump > 어떤 장애가 발생했을 때, 오류를 복사해서 가지고오는 역할
   * @param req
   * @param res
   * @return
   */
  bool dump_service(hdl_graph_slam::DumpGraphRequest& req, hdl_graph_slam::DumpGraphResponse& res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req.destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    }

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << directory << std::endl;

    graph_slam->save(directory + "/graph.g2o");
    for(int i = 0; i < keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->save(sst.str());
    }

    if(zero_utm) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
    ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;
    ofs << "floor_node " << (floor_plane_node == nullptr ? -1 : floor_plane_node->id()) << std::endl;

    res.success = true;
    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(hdl_graph_slam::SaveMapRequest& req, hdl_graph_slam::SaveMapResponse& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return true;
    }

    if(zero_utm && req.utm) {
      for(auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm).cast<float>();
      }
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm) {
      std::ofstream ofs(req.destination + ".utm");
      ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;

    return true;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::WallTimer optimization_timer;
  ros::WallTimer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber navsat_sub;

  ros::Subscriber imu_sub;
  ros::Subscriber floor_sub;

  ros::Publisher markers_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  ros::Publisher odom2map_pub;

  std::string points_topic;
  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;

  tf::TransformListener tf_listener;

  ros::ServiceServer dump_service_server;
  ros::ServiceServer save_map_service_server;

  // keyframe queue
  std::string base_frame_id;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  boost::optional<Eigen::Vector3d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

  // imu queue
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue;

  // floor_coeffs queue
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
