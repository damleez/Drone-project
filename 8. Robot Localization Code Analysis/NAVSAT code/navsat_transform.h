#ifndef ROBOT_LOCALIZATION_NAVSAT_TRANSFORM_H
#define ROBOT_LOCALIZATION_NAVSAT_TRANSFORM_H

#include <robot_localization/SetDatum.h>    //geographic_msgs/GeoPose geo_pose
#include <robot_localization/ToLL.h>        //geometry_msgs/Point map_point and GeoPoint ll_point
#include <robot_localization/FromLL.h>      //geographic_msgs/GeoPoint ll_point and Point map_point
#include <robot_localization/SetUTMZone.h>  //UTM zone setting

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>          //모든 글로벌 항법 위성 시스템에 대한 항법 위성 수정

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <GeographicLib/Geocentric.hpp>     //위도 경도 높이를 지구 중심 좌표(x,y,z)로
#include <GeographicLib/LocalCartesian.hpp> //로컬데카르트 좌표로 위 헤더에서 지구 중심 좌표를 통해 발생
#include <GeographicLib/MGRS.hpp>           //UTM/UPS와 MGRS간의 변환 (MGRS : 군사용 좌표계)
#include <GeographicLib/UTMUPS.hpp>         //UTM/UPS 좌표계

#include <string>

namespace RobotLocalization
{

class NavSatTransform
{
  public:
    //! @brief Constructor
    //! 생성자
    NavSatTransform(ros::NodeHandle nh, ros::NodeHandle nh_priv);

    //! @brief Destructor
    //! 소멸자
    ~NavSatTransform();

  private:
    //! @brief callback function which is called for periodic updates
    //! 주기적 업데이트를 위해 호출되는 콜백 함수
    void periodicUpdate(const ros::TimerEvent& event);

    //! @brief Computes the transform from the UTM frame to the odom frame
    //! UTM 프레임에서 Odom 프레임으로의 변환을 계산
    void computeTransform();

    //! @brief Callback for the datum service
    //! 데이터 서비스에 대한 콜백
    bool datumCallback(robot_localization::SetDatum::Request& request, robot_localization::SetDatum::Response&);

    //! @brief Callback for the to Lat Long service
    //! 위도, 경도 서비스에 대한 콜백
    bool toLLCallback(robot_localization::ToLL::Request& request, robot_localization::ToLL::Response& response);

    //! @brief Callback for the from Lat Long service
    //! 위도, 경도로 부터의 서비스에 대한 콜백
    bool fromLLCallback(robot_localization::FromLL::Request& request, robot_localization::FromLL::Response& response);

    //! @brief Callback for the UTM zone service
    //! UTM 영역 서비스에 대한 콜백
    bool setUTMZoneCallback(robot_localization::SetUTMZone::Request& request,
                            robot_localization::SetUTMZone::Response& response);

    //! @brief Given the pose of the navsat sensor in the cartesian frame, removes the offset from the vehicle's
    //! centroid and returns the cartesian-frame pose of said centroid.
    //! 데카르트 좌표계에서 navsat 센서의 포즈가 주어지면 차량의 중심에서 오프셋을 제거하고 해당 중심의 데카르트 좌표 포즈를 반환
    //! 여기서 catesian = 데카르트
    void getRobotOriginCartesianPose(const tf2::Transform &gps_cartesian_pose,
                                     tf2::Transform &robot_cartesian_pose,
                                     const ros::Time &transform_time);

    //! @brief Given the pose of the navsat sensor in the world frame, removes the offset from the vehicle's centroid
    //! and returns the world-frame pose of said centroid.
    //! 월드 프레임에서 navsat 센서의 포즈가 주어지면 차량의 중심에서 오프셋을 제거하고 해당 중심의 월드 프레임 포즈를 반환
    void getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                 tf2::Transform &robot_odom_pose,
                                 const ros::Time &transform_time);

    //! @brief Callback for the GPS fix data
    //! GPS 수정 데이터에 대한 콜백
    //! @param[in] msg The NavSatFix message to process
    //! input : 처리할 NavSatFix 메시지
    void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);

    //! @brief Callback for the IMU data
    //! IMU 데이터에 대한 콜백
    //! @param[in] msg The IMU message to process
    //! input : 처리할 IMU 메시지
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    //! @brief Callback for the odom data
    //! odom 데이터에 대한 콜백
    //! @param[in] msg The odometry message to process
    //! input : 처리할 odom 메시지
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    //! @brief Converts the odometry data back to GPS and broadcasts it
    //! odom 데이터를 다시 GPS로 변환하고 브로드캐스트
    //! @param[out] filtered_gps The NavSatFix message to prepare
    //! output : 준비할 NavSatFix 메시지
    bool prepareFilteredGps(sensor_msgs::NavSatFix &filtered_gps);

    //! @brief Prepares the GPS odometry message before sending
    //! 전송하기 전에 GPS 주행 거리 측정 메시지를 준비
    //! @param[out] gps_odom The odometry message to prepare
    //! output : gps_odom 준비할 주행 거리 측정 메시지
    bool prepareGpsOdometry(nav_msgs::Odometry &gps_odom);

    //! @brief Used for setting the GPS data that will be used to compute the transform
    //! 변환을 계산하는 데 사용할 GPS 데이터를 설정하는 데 사용
    //! @param[in] msg The NavSatFix message to use in the transform
    //! input : 변환에 사용할 NavSatFix 메시지
    void setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg);

    //! @brief Used for setting the odometry data that will be used to compute the transform
    //! 변환을 계산하는 데 사용할 odom 데이터를 설정하는 데 사용
    //! @param[in] msg The odometry message to use in the transform
    //! input : 변환에 사용할 odom 메시지
    void setTransformOdometry(const nav_msgs::OdometryConstPtr& msg);

    //! @brief Transforms the passed in pose from utm to map frame
    //! 전달된 포즈를 utm에서 맵 프레임으로 변환
    //! @param[in] cartesian_pose the pose in cartesian frame to use to transform
    //! input : 변환에 사용할 데카르트 좌표계의 포즈
    nav_msgs::Odometry cartesianToMap(const tf2::Transform& cartesian_pose) const;

    //! @brief Transforms the passed in point from map frame to lat/long
    //! 전달된 지점을 지도 프레임에서 위도/경도로 변환
    //! @param[in] point the point in map frame to use to transform
    //! input : 변환에 사용할 맵 프레임의 점을 가리킴
    void mapToLL(const tf2::Vector3& point, double& latitude, double& longitude, double& altitude) const;

    //! @brief Whether or not we broadcast the cartesian transform
    //! 데카르트 변환을 브로드캐스트할지 여부
    bool broadcast_cartesian_transform_;

    //! @brief Whether to broadcast the cartesian transform as parent frame, default as child
    //! 데카르트 변환을 부모 프레임으로 브로드캐스트할지 여부, 기본값은 자식 프레임
    bool broadcast_cartesian_transform_as_parent_frame_;

    //! @brief Whether or not we have new GPS data
    //! 새로운 GPS 데이터가 있는지 여부
    //! We only want to compute and broadcast our transformed GPS data if it's new. This variable keeps track of that.
    //! 우리는 변환된 GPS 데이터가 새로운 경우에만 계산하고 브로드캐스트하기를 원함, 이 변수는 이를 추적
    bool gps_updated_;

    //! @brief Whether or not the GPS fix is usable
    //! GPS 수정을 사용할 수 있는지 여부
    bool has_transform_gps_;

    //! @brief Signifies that we have received a usable IMU message
    //! 사용 가능한 IMU 메시지를 수신했음을 나타냄
    bool has_transform_imu_;

    //! @brief Signifies that we have received a usable odometry message
    //! 사용 가능한 odom 메시지를 수신했음을 나타냄
    bool has_transform_odom_;

    //! @brief Whether or not we have new odometry data
    //! 새로운 odom 데이터가 있는지 여부
    //! If we're creating filtered GPS messages, then we only want to broadcast them when new odometry data arrives.
    //! 필터링된 GPS 메시지를 생성하는 경우 새 odom 데이터가 도착할 때만 이를 브로드캐스트하려고 함
    bool odom_updated_;

    //! @brief Whether or not we publish filtered GPS messages
    //! 필터링된 GPS 메시지 게시 여부
    bool publish_gps_;

    //! @brief Whether or not we've computed a good heading
    //! 좋은 헤딩을 계산했는지 여부
    bool transform_good_;

    //! @brief Whether we get our datum from the first GPS message or from the set_datum
    //! service/parameter
    //! 첫 번째 GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터를 가져오는지 여부
    bool use_manual_datum_;

    //! @brief Whether we get the transform's yaw from the odometry or IMU source
    //! 주행 거리 또는 IMU 소스에서 변환의 yaw을 가져오는지 여부
    bool use_odometry_yaw_;

    //! @brief Whether we use a Local Cartesian (tangent plane ENU) or the UTM coordinates as our cartesian
    //! 로컬 데카르트(탄젠트 평면 ENU) 또는 UTM 좌표를 데카르트 좌표로 사용할지 여부
    bool use_local_cartesian_;

    //! @brief When true, do not print warnings for tf lookup failures.
    //! true인 경우 tf 조회 실패에 대한 경고를 인쇄하지 않음
    bool tf_silent_failure_;

    //! @brief Local Cartesian projection around gps origin
    //! GPS 원점 주변의 로컬 데카르트 투영법
    GeographicLib::LocalCartesian gps_local_cartesian_;

    //! @brief Whether or not to report 0 altitude
    //! 0 고도 보고 여부
    //! If this parameter is true, we always report 0 for the altitude of the converted GPS odometry message.
    //! 이 매개변수가 true이면 변환된 GPS 주행 거리 측정 메시지의 고도에 대해 항상 0을 보고
    bool zero_altitude_;

    //! @brief Parameter that specifies the magnetic declination for the robot's environment.
    //! 로봇 환경에 대한 자기 편각을 지정하는 매개변수

    //! 💥️ @brief magnetic declination 뜻 💥️
    //! 편각(偏角, declination)은 지구 자기의 3요소 중 하나로,
    //! 북반구를 기준으로 지구 상의 현재 위치에서 진북극(지리상의 북극점) 방향과 자기북극 방향(나침반의 빨간 바늘이 가리키는 방향) 사이의 각도
    double magnetic_declination_;

    //! @brief UTM's meridian convergence
    //! UTM의 자오선 수렴
    //! Angle between projected meridian (True North) and UTM's grid Y-axis.
    //! 투영된 자오선(진북)과 UTM의 그리드 Y축 사이의 각도
    //! For UTM projection (Ellipsoidal Transverse Mercator) it is zero on the equator and non-zero everywhere else.
    //! UTM 투영법(Ellipsoidal Transverse Mercator)의 경우 적도에서는 0이고 다른 곳에서는 0이 아님
    //! It increases as the poles are approached or as we're getting farther from central meridian.
    //! 극에 접근하거나 중심 자오선에서 멀어질수록 증가
    double utm_meridian_convergence_;

    //! @brief IMU's yaw offset
    //! IMU의 yaw 오프셋
    //! Your IMU should read 0 when facing *magnetic* north. If it doesn't, this (parameterized) value gives the offset
    //! *자기* 북쪽을 향할 때 IMU는 0을 읽어야 하며, 그렇지 않은 경우 이(매개변수화된) 값은 오프셋을 제공
    //! (NOTE: if you have a magenetic declination, use the parameter setting for that).
    //! (참고: 자기 편각이 있는 경우 해당 매개변수 설정을 사용)
    double yaw_offset_;

    //! @brief Frame ID of the robot's body frame
    //! 로봇 몸체 프레임의 프레임 ID
    //! This is needed for obtaining transforms from the robot's body frame to the frames of sensors (IMU and GPS)
    //! 이것은 로봇의 몸체 프레임에서 센서 프레임(IMU 및 GPS)으로의 변환을 얻기 위해 필요
    std::string base_link_frame_id_;

    //! @brief The cartesian frame ID, default as 'local_enu' if using Local Cartesian or 'utm' if using the UTM
    //! coordinates as our cartesian.
    //! 데카르트 좌표계 ID로, 로컬 데카르트를 사용하는 경우 기본값은 'local_enu'이고 UTM 좌표를 직교로 사용하는 경우 'utm'
    std::string cartesian_frame_id_;

    //! @brief The frame_id of the GPS message (specifies mounting location)
    //! GPS 메시지의 frame_id(장착 위치 지정)
    std::string gps_frame_id_;

    //! @brief the UTM zone (zero means UPS)
    //! UTM 영역(0은 UPS를 의미함)
    int utm_zone_;

    //! @brief hemisphere (true means north, false means south)
    //! 반구 (true는 북쪽을 의미하고 false는 남쪽을 의미)
    bool northp_;

    //! @brief Frame ID of the GPS odometry output
    //! GPS 주행 거리 측정 출력의 프레임 ID
    //! This will just match whatever your odometry message has
    //! 이것은 귀하의 odom 메시지가 무엇이든 일치
    std::string world_frame_id_;

    //! @brief Covariance for most recent odometry data
    //! 가장 최근의 odom 데이터에 대한 공분산
    Eigen::MatrixXd latest_odom_covariance_;

    //! @brief Covariance for most recent GPS/UTM/LocalCartesian data
    //! 가장 최근의 GPS/UTM/LocalCartesian 데이터에 대한 공분산
    Eigen::MatrixXd latest_cartesian_covariance_;

    //! @brief Timestamp of the latest good GPS message
    //! 최신 GPS 메시지의 간략한 타임스탬프
    //! We assign this value to the timestamp of the odometry message that we output
    //! 이 값을 우리가 출력하는 odom 측정 메시지의 타임스탬프에 할당
    ros::Time gps_update_time_;

    //! @brief Timestamp of the latest good odometry message
    //! 최신 odom 측정 메시지의 타임스탬프
    //! We assign this value to the timestamp of the odometry message that we output
    //! 이 값을 우리가 출력하는 odom 측정 메시지의 타임스탬프에 할당
    ros::Time odom_update_time_;

    //! @brief Parameter that specifies the how long we wait for a transform to become available.
    //! 변환을 사용할 수 있을 때까지 기다리는 시간을 지정하는 매개변수
    ros::Duration transform_timeout_;

    //! @brief timer calling periodicUpdate
    //! 주기적 업데이트를 호출하는 타이머
    ros::Timer periodicUpdateTimer_;

    //! @brief Latest IMU orientation
    //! 최신 IMU 오리엔테이션
    tf2::Quaternion transform_orientation_;

    //! @brief Latest GPS data, stored as Cartesian coords
    //! 데카르트 좌표로 저장된 최신 GPS 데이터
    tf2::Transform latest_cartesian_pose_;

    //! @brief Latest odometry pose data
    //! 최신 odom 포즈 데이터
    tf2::Transform latest_world_pose_;

    //! @brief Holds the cartesian (UTM or local ENU) pose that is used to compute the transform
    //! 변환을 계산하는 데 사용되는 데카르트(UTM 또는 로컬 ENU) 포즈를 유지
    tf2::Transform transform_cartesian_pose_;

    //! @brief Latest IMU orientation
    //! 최신 IMU 오리엔테이션
    tf2::Transform transform_world_pose_;

    //! @brief Holds the Cartesian->odom transform
    //! 데카르트->odom 변환 유지
    tf2::Transform cartesian_world_transform_;

    //! @brief Holds the odom->UTM transform for filtered GPS broadcast
    //! 필터링된 GPS 브로드캐스트에 대한 odom->UTM 변환을 유지
    tf2::Transform cartesian_world_trans_inverse_;

    //! @brief Publiser for filtered gps data
    //! 필터링된 GPS 데이터 게시자
    ros::Publisher filtered_gps_pub_;

    //! @brief GPS subscriber
    //! GPS 구독
    ros::Subscriber gps_sub_;

    //! @brief Subscribes to imu topic
    //! imu 주제 구독
    ros::Subscriber imu_sub_;

    //! @brief Odometry subscriber
    //! odom 구독
    ros::Subscriber odom_sub_;

    //! @brief Publisher for gps data
    //! GPS 데이터용 게시자
    ros::Publisher gps_odom_pub_;

    //! @brief Service for set datum
    //! 데이터 설정을 위한 서비스
    ros::ServiceServer datum_srv_;

    //! @brief Service for to Lat Long
    //! 위도, 경도를 위한 서비스
    ros::ServiceServer to_ll_srv_;

    //! @brief Service for from Lat Long
    //! 위도, 경도로부터의 서비스
    ros::ServiceServer from_ll_srv_;

    //! @brief Service for set UTM zone
    //! UTM zone을 위한 서비스
    ros::ServiceServer set_utm_zone_srv_;

    //! @brief Transform buffer for managing coordinate transforms
    //! 좌표 변환 관리를 위한 변환 버퍼
    tf2_ros::Buffer tf_buffer_;

    //! @brief Transform listener for receiving transforms
    //! 변환 수신을 위한 변환 수신기
    tf2_ros::TransformListener tf_listener_;

    //! @brief Used for publishing the static world_frame->cartesian transform
    //! 정적 world_frame->직교 변환을 게시하는 데 사용
    tf2_ros::StaticTransformBroadcaster cartesian_broadcaster_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_NAVSAT_TRANSFORM_H
