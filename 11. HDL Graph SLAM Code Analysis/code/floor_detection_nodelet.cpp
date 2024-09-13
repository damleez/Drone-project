// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <iostream>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace hdl_graph_slam {

class FloorDetectionNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  //fixed-size 벡터화 가능한 Eigen types를 정의할 때, operator new가 적절하게 정렬된 버퍼를 할당해야하는데 그걸 자동적으로 해줌
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //생성자 및 가상 소멸자
  FloorDetectionNodelet() {}
  virtual ~FloorDetectionNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing floor_detection_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    //prefiltering nodelet에서 생성된 filtered points sub
    points_sub = nh.subscribe("/filtered_points", 256, &FloorDetectionNodelet::cloud_callback, this);
    //floor detection nodelet에서 생성될 floor detection/floor coeffs pub
    floor_pub = nh.advertise<hdl_graph_slam::FloorCoeffs>("/floor_detection/floor_coeffs", 32);

    //floor detection nodelet에서 생성될 floor detection과 관련된 topic pub
    read_until_pub = nh.advertise<std_msgs::Header>("/floor_detection/read_until", 32);
    floor_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_filtered_points", 32);
    floor_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_points", 32);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    tilt_deg = private_nh.param<double>("tilt_deg", 0.0);                           // approximate sensor tilt angle [deg]
                                                                                    // 대략적인 센서 기울기 각도
    sensor_height = private_nh.param<double>("sensor_height", 2.0);                 // approximate sensor height [m]
                                                                                    // 대략적인 센서 높이
    height_clip_range = private_nh.param<double>("height_clip_range", 1.0);         // points with heights in [sensor_height - height_clip_range, sensor_height + height_clip_range] will be used for floor detection
                                                                                    // [sensor_height - height_clip_range, sensor_height + height_clip_range] 높이의 포인트가 바닥 감지에 사용
    floor_pts_thresh = private_nh.param<int>("floor_pts_thresh", 512);              // minimum number of support points of RANSAC to accept a detected floor plane
                                                                                    // 감지된 바닥 평면을 수용하기 위한 RANSAC의 최소 지지점 수
    floor_normal_thresh = private_nh.param<double>("floor_normal_thresh", 10.0);    // verticality check thresold for the detected floor plane [deg]
                                                                                    // 감지된 바닥 평면에 대한 수직성 검사 threshold [deg]
    use_normal_filtering = private_nh.param<bool>("use_normal_filtering", true);    // if true, points with "non-"vertical normals will be filtered before RANSAC
                                                                                    // true인 경우 수직 법선이 "non-"인 점은 RANSAC 전에 필터링
    normal_filter_thresh = private_nh.param<double>("normal_filter_thresh", 20.0);  // "non-"verticality check threshold [deg]
                                                                                    // "non-"수직 확인 threshold [deg]
    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    //Pointcloud의 포인터 cloud를 new를 통해 동적할당
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    //point cloud 형 변환 > sensor_msgs::PointCloud2 → pcl::PointCloud 
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(cloud->empty()) {
      return;
    }

    // floor detection
    // boost::optional > 값을 리턴하는 함수에서 유효하지 않은 값을 리턴하기 위해 만들어짐
    // 즉, optional은 값을 리턴하면서 값이 없음을 표시하며 / boost는 c++ 템플릿을 위한 라이브러리
    // Eigen::Vector4f > pointXYZI라서 4x1 matrix에 float형
    boost::optional<Eigen::Vector4f> floor = detect(cloud);

    // publish the detected floor coefficients
    // 감지된 floor 계수 pub
    hdl_graph_slam::FloorCoeffs coeffs;
    coeffs.header = cloud_msg->header;
    if(floor) {
      //coeffs.coeffs의 element의 size가 4 > 4f니까
      coeffs.coeffs.resize(4);
      for(int i = 0; i < 4; i++) {
        coeffs.coeffs[i] = (*floor)[i];
      }
    }

    // coeffs pub
    floor_pub.publish(coeffs);

    // for offline estimation
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);
  }

  /**
   * @brief detect the floor plane from a point cloud
   * 포인트 클라우드로 부터 floor plane(평면) 감지
   * @param cloud  input cloud
   * @return detected floor plane coefficients
   */
  boost::optional<Eigen::Vector4f> detect(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    // compensate the tilt rotation
    // 기울기 회전 보상
    // tilt_matrix는 4f Identity(단위)행렬
    Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    // tilt_matrix의 왼쪽상단 3x3 matrix = 각축의 float형( , , )
    // AngleAxisf는 AngleAxis의 한 종류로, 임의의 3D 축을 중심으로 한 회전 각도로 3D 회전을 나타냄
    // [Eigen] Rotation matrix <-> Euler angle > MatrixBase::Unit{X,Y,Z}와 결합하면 AngleAxis를 사용하여 오일러 각도를 쉽게 모방
    tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    // filtering before RANSAC (height and normal filtering)
    // Ransac 전 filtering (높이와 법선 필터링)
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    // cloud를 filtered에 tilt_matrix로 변환된 결과를 넣어줌 
    // 즉 cloud+tilt_matrix > filtered
    pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
    // filtered = 지면 위아래로 양방향으로 다른 포인트 클라우드 제거
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);

    // non 법선 벡터 필터링
    if(use_normal_filtering) {
      filtered = normal_filtering(filtered);
    }

    // filtered를 input으로 받아 tilt matrix의 inverse한 matrix를 filtered로 다시 transform
    pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

    if(floor_filtered_pub.getNumSubscribers()) {
      filtered->header = cloud->header;
      floor_filtered_pub.publish(*filtered);
    }

    // too few points for RANSAC
    if(filtered->size() < floor_pts_thresh) {
      return boost::none;
    }

    // too few inliers
    if(inliers->indices.size() < floor_pts_thresh) {
      return boost::none;
    }

    // RANSAC
    // SampleConsensusModelPlane의 포인터 model_p는 new로 동적할당 (filtered)
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
    // model_p RANSAC 수행
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    // 거리 threshold는 0.1
    ransac.setDistanceThreshold(0.1);
    // RANSAC 계산
    ransac.computeModel();

    // pointindices > 사용된 인덱스 벡터에 대한 포인터를 가져옴 를 inliers라고 정의
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // verticality check of the detected floor's normal
    // 감지된 floor의 법선의 수직성 확인
    Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

    // coeffs는 현재 크기가 0X0이고 초기화 되지 않은 동적 크기의 행렬
    Eigen::VectorXf coeffs;
    // getModelCoefficients > 지금까지 찾은 가장 좋은 모델의 계수를 반환
    ransac.getModelCoefficients(coeffs);

    // 법선 벡터가 수직이 아님
    // .head() > DataFrame 내의 처음 n줄의 데이터 출력 (z보니까 3인듯)
    double dot = coeffs.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
      // the normal is not vertical
      return boost::none;
    }

    // make the normal upward
    // 법선 벡터 반전
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
      coeffs *= -1.0f;
    }

    if(floor_points_pub.getNumSubscribers()) {
      pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filtered);
      extract.setIndices(inliers);
      extract.filter(*inlier_cloud);
      inlier_cloud->header = cloud->header;

      floor_points_pub.publish(*inlier_cloud);
    }

    return Eigen::Vector4f(coeffs);
  }

  /**
   * @brief plane_clip
   * 평면 나누기
   * @param src_cloud
   * @param plane
   * @param negative
   * @return
   * pcl::PlaneClipper3D > 평면 clipper로 지정된 포인트 클라우드 영역의 한쪽에 있는 모든 포인트를 제거
   */
  // 포인트 클라우드의 한 면을 제거하는 함수인 plane_clip 함수
  // 145~146에서 filtered로 위 아래로 포인트 클라우드 제거
  pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) const {
    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);

    return dst_cloud;
  }

  /**
   * @brief filter points with non-vertical normals
   * 비 수직 법선 포인트 필터링
   * 그라운드 포인트 클라우드에서 각 포인트의 위치에 해당하는 법선 벡터는 위쪽에 있어야 하며 약간의 차이가 있더라도 너무 크지 않아야 하고 그렇지 않으면 그라운드 포인트가 아님
   * @param cloud  input cloud
   * @return filtered cloud
   * 법선 벡터 계산은 pcl 라이브러리의 pcl::NormalEstimation 함수에서 사용
   */
  pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // 최근접 이웃 10
    ne.setKSearch(10);
    // setViewPoint 메서드를 사용하여 시점을 수동으로 설정한 경우 이 메서드는 설정된 시점 좌표를 반환 (z = 2m)
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    // 그라운드 포인트 클라우드의 법선 벡터 계산
    ne.compute(*normals);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    //reserve > 초기화 X, capacity만 확보
    filtered->reserve(cloud->size());

    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐잘 모르겠음⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
    // 핵심 단락 > 이 점은 법선 벡터가 요구 사항을 충족할 때만 선택
    for(int i = 0; i < cloud->size(); i++) {
      //🧠피셜 : dot은 normals의 i번째 문자 반환해서 normalvector3fmap으로 변환해서 정규화하고 거기서 z얻음
      float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
      if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
        filtered->push_back(cloud->at(i));
      }
    }
    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // ROS topics
  ros::Subscriber points_sub;

  ros::Publisher floor_pub;
  ros::Publisher floor_points_pub;
  ros::Publisher floor_filtered_pub;

  std::string points_topic;
  ros::Publisher read_until_pub;

  // floor detection parameters
  // see initialize_params() for the details
  double tilt_deg;
  double sensor_height;
  double height_clip_range;

  int floor_pts_thresh;
  double floor_normal_thresh;

  bool use_normal_filtering;
  double normal_filter_thresh;
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::FloorDetectionNodelet, nodelet::Nodelet)
