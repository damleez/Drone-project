// SPDX-License-Identifier: BSD-2-Clause
// 각도나 움직임이 일정시간을 초과하면 재설정

#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <hdl_graph_slam/ScanMatchingStatus.h>

namespace hdl_graph_slam {

class ScanMatchingOdometryNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  //fixed-size 벡터화 가능한 Eigen types를 정의할 때, operator new가 적절하게 정렬된 버퍼를 할당해야하는데 그걸 자동적으로 해줌
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //생성자, 가상소멸자 선언
  ScanMatchingOdometryNodelet() {}
  virtual ~ScanMatchingOdometryNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing scan_matching_odometry_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    if(private_nh.param<bool>("enable_imu_frontend", false)) {
      //msf_pose_callback이 바인딩 된 상태에서 msf_pose_callback이 호출되면 첫번째 인자에 값이 전달되며, 기본값은 false 
      msf_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose", 1, boost::bind(&ScanMatchingOdometryNodelet::msf_pose_callback, this, _1, false));
      //msf_pose_callback이 바인딩 된 상태에서 msf_pose_callback이 호출되면 첫번째 인자에 값이 전달되며, 기본값은 true > 얘는 after update sub 
      msf_pose_after_update_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose_after_update", 1, boost::bind(&ScanMatchingOdometryNodelet::msf_pose_callback, this, _1, true));
    }

    points_sub = nh.subscribe("/filtered_points", 256, &ScanMatchingOdometryNodelet::cloud_callback, this);
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until", 32);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 32);
    trans_pub = nh.advertise<geometry_msgs::TransformStamped>("/scan_matching_odometry/transform", 32);
    status_pub = private_nh.advertise<ScanMatchingStatus>("/scan_matching_odometry/status", 8);
    aligned_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 32);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    points_topic = pnh.param<std::string>("points_topic", "/velodyne_points");
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");
    robot_odom_frame_id = pnh.param<std::string>("robot_odom_frame_id", "robot_odom");

    // The minimum tranlational distance and rotation angle between keyframes.
    // 키프레임 사이의 최소 변환 거리 및 회전 각도
    // If this value is zero, frames are always compared with the previous frame
    // 이 값이 0이면 프레임은 항상 이전 프레임과 비교
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);

    // Registration validation by thresholding
    // threshold값에 의한 registration 검증
    transform_thresholding = pnh.param<bool>("transform_thresholding", false);
    max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 1.0);

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    // 다운샘플링방법 선택
    std::string downsample_method = pnh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter.reset(voxelgrid);
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      // pcl::passthrough > range 기반으로 filtering을 해주는 함수
      pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }

    //icp, gicp, ndt 중 정합 방법 선택 가능 > pnh
    registration = select_registration_method(pnh);
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if(!ros::ok()) {
      return;
    }

    //sensor msg pointcloud > pcl pointcloud 
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    //pose는 4x1 float matrix고 matching 함수(cloud를입력으로 받음)
    Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);
    //odometry publish 함수는 아래에 있음 pose를 입력으로 받음
    publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);

    // In offline estimation, point clouds until the published time will be supplied
    // offline추정시, pub될때 까지 포인트클라우드 제공
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);
  }

  void msf_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, bool after_update) {
    if(after_update) {
      msf_pose_after_update = pose_msg;
    } else {
      msf_pose = pose_msg;
    }
  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe cloud
   * 입력 클라우드와 키프레임 클라우드 사이의 상대 포즈 추정
   * @param stamp  the timestamp of the input cloud
   * 입력 클라우드의 타임스탬프
   * @param cloud  the input cloud
   * 입력 클라우드
   * @return the relative pose between the input cloud and the keyframe cloud
   * 입력 클라우드와 키프레임 클라우드 사이의 상대 포즈
   */
  Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    //키프레임이 아니라면 즉, 아직 키프레임이 없어서 첫 번째 프레임을 알리고 키 프레임으로 설정하고 포즈를 단위 행렬로 초기화하고 직접 반환
    if(!keyframe) {
      prev_time = ros::Time();                  //이전 시간 = 로스 타임
      prev_trans.setIdentity();                 //이전 변환은 단위 행렬로 set
      keyframe_pose.setIdentity();              //키프레임 포즈는 단위 행렬로 set
      keyframe_stamp = stamp;                   //키프레임 스템프 = 스템프 (타임스탬프)
      keyframe = downsample(cloud);             //키프레임 = 다운샘플링 함수(입력 클라우드)
      registration->setInputTarget(keyframe);   //정합은 키프레임 입력으로 받아서 
      return Eigen::Matrix4f::Identity();       //반환은 4x1 float matrix (단위 행렬)
    }

    //정합을 위한 현재 포인트 클라우드
    auto filtered = downsample(cloud);
    registration->setInputSource(filtered);

    std::string msf_source;
    //isometry란 affine과 동일(선형 부분이 회전을 나타낸다는 가정), 이러한 가정은 inverse() 및 rotation()과 같은 일부 함수의 속도를 높이는데 이용
    Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();

    //경우 1 > imu fronted가 이용가능 하다면 
    if(private_nh.param<bool>("enable_imu_frontend", false)) {
      //msf_pose && msf_pose의 타임스탬프가 keyframe 타임스탬프보다 큼 && msf_pose_after_update && msf_pose_after_update의 타임스탬프가 keyframe 타임스탬프보다 크다면 
      if(msf_pose && msf_pose->header.stamp > keyframe_stamp && msf_pose_after_update && msf_pose_after_update->header.stamp > keyframe_stamp) {
        //pose2isometry는 position와 orientation mat계산
        Eigen::Isometry3d pose0 = pose2isometry(msf_pose_after_update->pose.pose);
        Eigen::Isometry3d pose1 = pose2isometry(msf_pose->pose.pose);
        Eigen::Isometry3d delta = pose0.inverse() * pose1;

        msf_source = "imu";
        //msf_delta를 float형으로 형변환 (cast함수)
        msf_delta = delta.cast<float>();
      } else {
        std::cerr << "msf data is too old" << std::endl;
      }

    //경우 2 > 로봇 오도메트리의 초기 추측이 이용가능하며 이전 시간이 0이 아니라면
    } else if(private_nh.param<bool>("enable_robot_odometry_init_guess", false) && !prev_time.isZero()) {
      tf::StampedTransform transform;
      if(tf_listener.waitForTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id, ros::Duration(0))) {
        tf_listener.lookupTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id, transform);
      } else if(tf_listener.waitForTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, prev_time, robot_odom_frame_id, ros::Duration(0))) {
        tf_listener.lookupTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, prev_time, robot_odom_frame_id, transform);
      }

      if(transform.stamp_.isZero()) {
        NODELET_WARN_STREAM("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        msf_source = "odometry";
        msf_delta = tf2isometry(transform).cast<float>();
      }
    }

    //matching 실행
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    // 이전 변환 * msf_delta.matrix한 결과를 align의 output aligned로 출력
    registration->align(*aligned, prev_trans * msf_delta.matrix());

    //scan matching상태를 pub
    publish_scan_matching_status(stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

    //매치가 끝나면 키프레임 기준으로 포즈 반환
    //hasconverged > 마지막 align 실행 후 수렴 상태를 반환
    if(!registration->hasConverged()) {
      NODELET_INFO_STREAM("scan matching has not converged!!");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return keyframe_pose * prev_trans;
    }

    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐MAIN⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
    //trans와 odom 정의
    Eigen::Matrix4f trans = registration->getFinalTransformation();
    Eigen::Matrix4f odom = keyframe_pose * trans;
    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

    //허용가능한 trans와 angle의 max만큼 움직이는지 확인
    if(transform_thresholding) {
      Eigen::Matrix4f delta = prev_trans.inverse() * trans;

      ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐잘모르겠음⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
      double dx = delta.block<3, 1>(0, 3).norm();
      double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());
      ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

      if(dx > max_acceptable_trans || da > max_acceptable_angle) {
        NODELET_INFO_STREAM("too large transform!!  " << dx << "[m] " << da << "[rad]");
        NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
        return keyframe_pose * prev_trans;
      }
    }

    prev_time = stamp;
    prev_trans = trans;

    auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    keyframe_broadcaster.sendTransform(keyframe_trans);

    //이 프레임의 위치나 각도가 키프레임에 비해 상대적으로 큰지 판단
    //상대적으로 크면 키프레임을 업데이트하고 새 키프레임의 포인트 클라우드를 Target으로 설정
    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp).toSec();
    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
      keyframe = filtered;
      registration->setInputTarget(keyframe);

      keyframe_pose = odom;
      keyframe_stamp = stamp;
      prev_time = stamp;
      prev_trans.setIdentity();
    }

    if (aligned_points_pub.getNumSubscribers() > 0)
    {
      pcl::transformPointCloud (*cloud, *aligned, odom);
      aligned->header.frame_id=odom_frame_id;
      aligned_points_pub.publish(*aligned);
    }

    return odom;
  }

  /**
   * @brief publish odometry
   * 오도메트리 pub
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // publish transform stamped for IMU integration
    // IMU 인터그레이션을 위해 transform pub
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    trans_pub.publish(odom_trans);

    // broadcast the transform over tf
    // tf를 통해 변환 브로드캐스트
    odom_broadcaster.sendTransform(odom_trans);

    // publish the transform
    // transform 게시 pub
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom);
  }

  /**
   * @brief publish scan matching status
   * 스캔 매칭 상태 pub
   */
  void publish_scan_matching_status(const ros::Time& stamp, const std::string& frame_id, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string& msf_source, const Eigen::Isometry3f& msf_delta) {
    if(!status_pub.getNumSubscribers()) {
      return;
    }

    ScanMatchingStatus status;
    status.header.frame_id = frame_id;
    status.header.stamp = stamp;
    status.has_converged = registration->hasConverged();
    status.matching_error = registration->getFitnessScore();

    // 대응관계가 유효하기 위한 최대 거리
    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(int i=0; i<aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();

    status.relative_pose = isometry2pose(Eigen::Isometry3f(registration->getFinalTransformation()).cast<double>());

    if(!msf_source.empty()) {
      status.prediction_labels.resize(1);
      status.prediction_labels[0].data = msf_source;

      status.prediction_errors.resize(1);
      Eigen::Isometry3f error = Eigen::Isometry3f(registration->getFinalTransformation()).inverse() * msf_delta;
      status.prediction_errors[0] = isometry2pose(error.cast<double>());
    }

    status_pub.publish(status);
  }

private:
  // ROS topics
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Subscriber msf_pose_sub;
  ros::Subscriber msf_pose_after_update_sub;

  ros::Publisher odom_pub;
  ros::Publisher trans_pub;
  ros::Publisher aligned_points_pub;
  ros::Publisher status_pub;
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster keyframe_broadcaster;

  std::string points_topic;
  std::string odom_frame_id;
  std::string robot_odom_frame_id;
  ros::Publisher read_until_pub;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  // odometry calculation
  geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose;
  geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose_after_update;

  ros::Time prev_time;
  Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;               // keyframe pose
  ros::Time keyframe_stamp;                    // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud

  //
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::ScanMatchingOdometryNodelet, nodelet::Nodelet)
