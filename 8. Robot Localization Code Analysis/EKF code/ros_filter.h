#ifndef ROBOT_LOCALIZATION_ROS_FILTER_H
#define ROBOT_LOCALIZATION_ROS_FILTER_H

#include "robot_localization/ros_filter_utilities.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/filter_base.h"

//! setpose는 내부 pose 추정치를 수동으로 설정, 재설정
//! ToggleFilterProcessing은 ros service로 스탠바이모드에 대한 프로세싱 측정을 토글스위칭하는 서비스 콜백
//! but, 계속 publish 함
//! 즉, 게시하는 동안 다른 노드가 필터 처리를 켜고 끌 수 있도록 하는 서비스
#include <robot_localization/SetPose.h>
#include <robot_localization/ToggleFilterProcessing.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>                             //ROS node에 empty(서비스와 클라이언트간 실제 데이터 교환 X)신호 보냄
#include <nav_msgs/Odometry.h>                          //자유 공간에서 위치와 속도의 추정치
#include <sensor_msgs/Imu.h>                            //IMU데이터 담음, '공분산'요소 중요
#include <geometry_msgs/Twist.h>                        //선형 부분과 각 부분으로 나뉜 자유 공간에서의 속도
#include <geometry_msgs/TwistStamped.h>                 //참조 좌표 프레임과 타임스탬프가 있는 twist
#include <geometry_msgs/TwistWithCovarianceStamped.h>   //기준 좌표 프레임과 타임스탬프로 추정된 twist
#include <geometry_msgs/PoseWithCovarianceStamped.h>    //기준 좌표계와 타임스탬프로 추정 pose를 표현
#include <geometry_msgs/AccelWithCovarianceStamped.h>   //기준 좌표계와 타임스탬프로 추정된 가속도
#include <tf2_ros/transform_listener.h>                 //좌표 프레임 변환 정보를 요청하고 수신하는 쉬운 방법을 제공
#include <tf2_ros/transform_broadcaster.h>              //좌표 프레임 변환 정보를 게시하는 쉬운 방법을 제공
#include <tf2_ros/message_filter.h>                     //사용 가능한 변환 데이터가 있을 때만 메시지를 통과시키는 필터를 구현
#include <tf2/LinearMath/Transform.h>                   //Transform 클래스는 크기 scaling/shear 없이 변환 및 회전만 있는 고정 변환을 지원
#include <message_filters/subscriber.h>                 //이 클래스는 단순히 메시지를 전달하는 최상위 필터 역할
#include <diagnostic_updater/diagnostic_updater.h>      //진단 작업 목록을 관리, update중요
#include <diagnostic_updater/publisher.h>               //ros::Publisher + FrequencyStatus 및 TimeStampStatus를 사용하여 주제에 대한 진단을 용이하게 하는 클래스
#include <diagnostic_msgs/DiagnosticStatus.h>           //로봇의 개별 구성 요소의 상태를 담음

#include <XmlRpcException.h>                            //서버 메서드에서 이 예외가 발생하면 클라이언트에 오류 응답이 반환

#include <Eigen/Dense>                                  //선형 대수학 헤더
#include <fstream>                                      //ofstream은 파일에 기록할 때 사용하고 ifstream은 파일에 저장된 데이터를 읽어올 때 사용
#include <map>                                          //stl. key, value값설정
#include <memory>                                       //메모리 조작 헤더
#include <numeric>                                      //stl. 수치 알고리즘 헤더
#include <queue>                                        //stl. 큐(위 아래가 뚫린 형태) 헤더
#include <string>                                       //문자열 헤더
#include <vector>                                       //stl. 벡터 헤더
#include <deque>                                        //stl. 앞 뒤에서 입력과 출력이 모두 가능, 큐와 비슷하지만 확장형 느낌

namespace RobotLocalization
{

//callbackdata 구조체 선언
struct CallbackData
{
  //callbackdata 구조체 생성자
  //멤버변수 선언하고, ex) topicName_(topicName)은 뒤의 토픽네임을 앞에 할당한다는 뜻
  CallbackData(const std::string &topicName,
               const std::vector<int> &updateVector,
               const int updateSum,
               const bool differential,
               const bool relative,
               const bool pose_use_child_frame,
               const double rejectionThreshold) :
    topicName_(topicName),
    updateVector_(updateVector),
    updateSum_(updateSum),
    differential_(differential),
    relative_(relative),
    pose_use_child_frame_(pose_use_child_frame),
    rejectionThreshold_(rejectionThreshold)
  {
  }
  //callbackdata 멤버들 정의 : 자료형
  std::string topicName_;
  std::vector<int> updateVector_;
  int updateSum_;
  bool differential_;
  bool relative_;
  bool pose_use_child_frame_;
  double rejectionThreshold_;
};

//이름 재정의
typedef std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>, Measurement> MeasurementQueue;
typedef std::deque<MeasurementPtr> MeasurementHistoryDeque;
typedef std::deque<FilterStatePtr> FilterStateHistoryDeque;


template<class T> class RosFilter
{
  public:
    //! @brief Constructor
    //! 생성자
    //! The RosFilter constructor makes sure that anyone using
    //! this template is doing so with the correct object type
    //! RosFilter 생성자는 이 템플릿을 사용하는 모든 사람이 올바른 객체 유형으로 그렇게 하고 있는지 확인
    
    //! 자동형변환 안되게 함
    explicit RosFilter(ros::NodeHandle nh,
                       ros::NodeHandle nh_priv,
                       std::string node_name,
                       std::vector<double> args = std::vector<double>());

    //! @brief Constructor
    //! 생성자
    //! The RosFilter constructor makes sure that anyone using
    //! this template is doing so with the correct object type
    //! RosFilter 생성자는 이 템플릿을 사용하는 모든 사람이 올바른 객체 유형으로 그렇게 하고 있는지 확인
    explicit RosFilter(ros::NodeHandle nh, ros::NodeHandle nh_priv, std::vector<double> args = std::vector<double>());

    //! @brief Destructor
    //! 소멸자
    //! Clears out the message filters and topic subscribers.
    //! 메시지 필터와 토픽 서브스크라이버를 지움
    ~RosFilter();

    //! @brief Initialize filter
    // 필터 초기화
    void initialize();

    //! @brief Resets the filter to its initial state
    //! 필터를 초기 상태로 재설정
    void reset();

    //! @brief Service callback to toggle processing measurements for a standby mode but continuing to publish
    //! 스탠바이 모드에 대한 처리 측정을 전환하지만 계속 게시하는 서비스 콜백
    //! @param[in] request - The state requested, on (True) or off (False)
    //! input : request > 요청된 상태, 켜짐(True) 또는 꺼짐(False)
    //! @param[out] response - status if upon success
    //! output : response > 성공 시 상태
    //! @return boolean true if successful, false if not
    //! return : boolean 성공하면 true, 실패하면 false
    //! 👉️ 함수 : 요청, 응답 받는 toggleFilterProcessingCallback bool형 함수
    bool toggleFilterProcessingCallback(robot_localization::ToggleFilterProcessing::Request&,
                                        robot_localization::ToggleFilterProcessing::Response&);

    //! @brief Callback method for receiving all acceleration (IMU) messages
    //! 모든 가속(IMU) 메시지 수신을 위한 콜백 방식
    //! @param[in] msg - The ROS IMU message to take in.
    //!  input : msg > 받아들일 ROS IMU 메시지
    //! @param[in] callbackData - Relevant static callback data
    //!  input : callbackData > 관련 고정된 콜백 데이터
    //! @param[in] targetFrame - The target frame_id into which to transform the data
    //! input : targetFrame > 데이터를 변환할 대상 frame_id
    //! 👉️ 함수 : 위 param받아 accelerationCallback함수 선언
    void accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg,
                              const CallbackData &callbackData,
                              const std::string &targetFrame);

    //! @brief Callback method for receiving non-stamped control input
    //! 스탬프가 없는 제어 입력을 수신하기 위한 콜백 메서드
    //! @param[in] msg - The ROS twist message to take in
    //! input : msg > 받아들일 ROS twist 메세지
    void controlCallback(const geometry_msgs::Twist::ConstPtr &msg);

    //! @brief Callback method for receiving stamped control input
    //! 스탬프가 있는 제어 입력을 수신하기 위한 콜백 메서드
    //! @param[in] msg - The ROS stamped twist message to take in
    //! input : msg > 받아들일 ROS 스탬프 twist 메세지
    void controlCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    //! @brief Adds a measurement to the queue of measurements to be processed
    //! 처리할 측정 Queue에 측정을 추가
    //! @param[in] topicName - The name of the measurement source (only used for debugging)
    //! input : topicName > 측정 소스의 이름(디버깅에만 사용됨)
    //! @param[in] measurement - The measurement to enqueue
    //! input : measurement > Queue에 넣을 측정
    //! @param[in] measurementCovariance - The covariance of the measurement
    //! input : measurementCovariance > 측정의 공분산
    //! @param[in] updateVector - The boolean vector that specifies which variables to update from this measurement
    //! input : updateVector > 이 측정에서 업데이트할 변수를 지정하는 boolean 벡터
    //! @param[in] mahalanobisThresh - Threshold, expressed as a Mahalanobis distance, for outlier rejection
    //! input : mahalanobisThresh > 임계값, Mahalanobis 거리로 표시되는 이상치 거부에 대한 임계값
    //! @param[in] time - The time of arrival (in seconds)
    //! input : time > 도착 시간(초)
    void enqueueMeasurement(const std::string &topicName,
                            const Eigen::VectorXd &measurement,
                            const Eigen::MatrixXd &measurementCovariance,
                            const std::vector<int> &updateVector,
                            const double mahalanobisThresh,
                            const ros::Time &time);

    //! @brief Method for zeroing out 3D variables within measurements
    //! 측정 내 3D 변수를 0으로 만드는 방법
    //! @param[out] measurement - The measurement whose 3D variables will be zeroed out
    //! output : measurement > 3D 변수가 0이 될 측정
    //! @param[out] measurementCovariance - The covariance of the measurement
    //! output : measurementCovariance > 측정의 공분산
    //! @param[out] updateVector - The boolean update vector of the measurement
    //! output : updateVector > 측정값의 boolean 업데이트 벡터

    //! If we're in 2D mode, then for every measurement from every sensor, we call this.
    //! It sets the 3D variables to 0, gives those variables tiny variances, and sets
    //! their updateVector values to 1.
    //! 2D 모드에 있는 경우 모든 센서의 모든 측정에 대해 이것을 호출
    //! 3D 변수를 0으로 설정하고 해당 변수에 작은 편차를 주고 updateVector 값을 1로 설정
    void forceTwoD(Eigen::VectorXd &measurement,
                   Eigen::MatrixXd &measurementCovariance,
                   std::vector<int> &updateVector);

    //! @brief Retrieves the EKF's output for broadcasting
    //! 브로드캐스팅을 위해 EKF의 출력을 검색
    //! @param[out] message - The standard ROS odometry message to be filled
    //! output : message > 표준 ROS 오도메터리 메시지로 채워짐
    //! @return true if the filter is initialized, false otherwise
    //! return : 필터가 초기화되면 true, 그렇지 않으면 false
    bool getFilteredOdometryMessage(nav_msgs::Odometry &message);

    //! @brief Retrieves the EKF's acceleration output for broadcasting
    //! 브로드캐스팅을 위해 EKF 가속도 출력을 검색
    //! @param[out] message - The standard ROS acceleration message to be filled
    //! output : message > 표준 ROS acceleration 메시지로 채워짐
    //! @return true if the filter is initialized, false otherwise
    //! return : 필터가 초기화되면 true, 그렇지 않으면 false
    bool getFilteredAccelMessage(geometry_msgs::AccelWithCovarianceStamped &message);

    //! @brief Callback method for receiving all IMU messages
    //! 모든 IMU 메시지 수신을 위한 콜백 방식
    //! @param[in] msg - The ROS IMU message to take in.
    //! input : msg > 받아들일 ROS IMU 메시지
    //! @param[in] topicName - The topic name for the IMU message (only used for debug output)
    //! input : topicName > IMU 메시지의 주제 이름(디버그 출력에만 사용됨)
    //! @param[in] poseCallbackData - Relevant static callback data for orientation variables
    //! input : poseCallbackData > 방향 변수에 대한 관련 정적 콜백 데이터
    //! @param[in] twistCallbackData - Relevant static callback data for angular velocity variables
    //! input : twistCallbackData > 각속도 변수에 대한 관련 정적 콜백 데이터
    //! @param[in] accelCallbackData - Relevant static callback data for linear acceleration variables
    //! input : accelCallbackData > 선형 가속 변수에 대한 관련 정적 콜백 데이터

    //! This method separates out the orientation, angular velocity, and linear acceleration data and
    //! passed each on to its respective callback.
    //! 이 메서드는 방향, 각속도 및 선형 가속도 데이터를 분리하고 각각의 콜백에 전달
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg, const std::string &topicName,
      const CallbackData &poseCallbackData, const CallbackData &twistCallbackData,
      const CallbackData &accelCallbackData);

    //! @brief Processes all measurements in the measurement queue, in temporal order
    //! 측정 queue의 모든 측정을 시간 순서로 처리
    //! @param[in] currentTime - The time at which to carry out integration (the current time)
    //! input : 통합을 수행할 시간(현재 시간)
    void integrateMeasurements(const ros::Time &currentTime);

    //! @brief Differentiate angular velocity for angular acceleration
    //! 각가속도에 대한 각속도 미분
    //! @param[in] currentTime - The time at which to carry out differentiation (the current time)
    //! input : 미분을 수행할 시간(현재 시간)

    //! Maybe more state variables can be time-differentiated to estimate higher-order states,
    //! but now we only focus on obtaining the angular acceleration. It implements a backward-
    //! Euler differentiation.
    //! 고차 상태를 추정하기 위해 더 많은 상태 변수를 시간 미분할 수 있지만 지금은 각 가속도를 얻는 데만 초점을 맞춤
    //! 역방향 오일러 미분을 구현합니다.
    void differentiateMeasurements(const ros::Time &currentTime);

    //! @brief Loads all parameters from file
    //! 파일에서부터 모든 파라미터들을 로드
    void loadParams();

    //! @brief Callback method for receiving all odometry messages
    //! 모든 오도메트리(주행 거리) 메시지 수신을 위한 콜백 메소드
    //! @param[in] msg - The ROS odometry message to take in.
    //! input : msg > 수신할 ROS 주행 거리 측정 메시지
    //! @param[in] topicName - The topic name for the odometry message (only used for debug output)
    //! input : 주행 거리 측정 메시지의 주제 이름(디버그 출력에만 사용됨)
    //! @param[in] poseCallbackData - Relevant static callback data for pose variables
    //! input : 포즈 변수에 대한 관련 정적 콜백 데이터
    //! @param[in] twistCallbackData - Relevant static callback data for twist variables
    //! input : 트위스트 변수에 대한 관련 정적 콜백 데이터

    //! This method simply separates out the pose and twist data into two new messages, and passes them into their
    //! respective callbacks
    //! 이 메서드는 단순히 포즈와 twist 데이터를 두 개의 새 메시지로 분리하고 각각의 콜백으로 전달
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &topicName,
      const CallbackData &poseCallbackData, const CallbackData &twistCallbackData);

    //! @brief Callback method for receiving all pose messages
    //! 모든 포즈 메시지 수신을 위한 콜백 메소드
    //! @param[in] msg - The ROS stamped pose with covariance message to take in
    //! input : 받아들일 공분산 메시지가 있는 ROS 스탬프 포즈
    //! @param[in] callbackData - Relevant static callback data
    //! input : 관련 정적 콜백 데이터
    //! @param[in] targetFrame - The target frame_id into which to transform the data
    //! input : 데이터를 변환할 대상 frame_id
    //! @param[in] poseSourceFrame - The source frame_id from which to transform the data
    //! input : 데이터를 변환할 소스 frame_id
    //! @param[in] imuData - Whether this data comes from an IMU
    //! input : 이 데이터가 IMU에서 오는지 여부
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                      const CallbackData &callbackData,
                      const std::string &targetFrame,
                      const std::string &poseSourceFrame,
                      const bool imuData);

    //! @brief Callback method for manually setting/resetting the internal pose estimate
    //! 내부 포즈 추정치를 수동으로 설정/재설정하기 위한 콜백 메서드
    //! @param[in] msg - The ROS stamped pose with covariance message to take in
    //! input : 받아들일 공분산 메시지가 있는 ROS 스탬프 포즈
    void setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    //! @brief Service callback for manually setting/resetting the internal pose estimate
    //! 내부 포즈 추정치를 수동으로 설정/재설정하기 위한 서비스 콜백
    //! @param[in] request - Custom service request with pose information
    //! input : 포즈 정보가 포함된 맞춤형 서비스 요청
    //! @return true if successful, false if not
    //! return : 성공하면 true, 실패하면 false
    bool setPoseSrvCallback(robot_localization::SetPose::Request& request,
                            robot_localization::SetPose::Response&);

    //! @brief Service callback for manually enable the filter
    //! 필터를 수동으로 활성화하기 위한 서비스 콜백
    //! @param[in] request - N/A
    //! @param[out] response - N/A
    //! @return boolean true if successful, false if not
    //! return : 성공하면 true, 실패하면 false
    bool enableFilterSrvCallback(std_srvs::Empty::Request&,
                                 std_srvs::Empty::Response&);

    //! @brief Callback method for receiving all twist messages
    //! 모든 트위스트 메시지를 수신하기 위한 콜백 메소드
    //! @param[in] msg - The ROS stamped twist with covariance message to take in.
    //! input : 받아들일 공분산 메시지가 있는 ROS 스탬프 트위스트
    //! @param[in] callbackData - Relevant static callback data
    //! input : 관련 정적 콜백 데이터
    //! @param[in] targetFrame - The target frame_id into which to transform the data
    //! input : 데이터를 변환할 대상 frame_id
    void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                       const CallbackData &callbackData,
                       const std::string &targetFrame);

    //! @brief Validates filter outputs for NaNs and Infinite values
    //! NaN 및 Infinite 값에 대한 필터 출력의 유효성을 검사
    //! @param[out] message - The standard ROS odometry message to be validated
    //! output : 검증할 표준 ROS 주행 거리 메시지
    //! @return true if the filter output is valid, false otherwise
    //! return : 필터 출력이 유효하면 true, 그렇지 않으면 false
    bool validateFilterOutput(const nav_msgs::Odometry &message);

  protected:
    //! @brief Finds the latest filter state before the given timestamp and makes it the current state again.
    //! 주어진 타임스탬프 이전의 최신 필터 상태를 찾아 다시 현재 상태로 만듬
    //! This method also inserts all measurements between the older filter timestamp and now into the measurements
    //! queue.
    //! 이전 필터 타임스탬프와 현재 측정 queue 사이의 모든 측정을 삽입
    //! @param[in] time - The time to which the filter state should revert
    //! input :  필터 상태를 되돌려야 하는 시간
    //! @return True if restoring the filter succeeded. False if not.
    //! return : 필터 복원에 성공하면 True
    bool revertTo(const double time);

    //! @brief Saves the current filter state in the queue of previous filter states
    //! 이전 필터 상태의 대기열에 현재 필터 상태를 저장
    //! These measurements will be used in backwards smoothing in the event that older measurements come in.
    //! 이러한 측정값은 이전 측정값이 들어오는 경우 역방향 평활화에 사용
    //! @param[in] filter - The filter base object whose state we want to save
    //! input : 상태를 저장하려는 필터 기본 개체
    void saveFilterState(FilterBase &filter);

    //! @brief Removes measurements and filter states older than the given cutoff time.
    //! 주어진 컷오프 시간보다 오래된 측정 및 필터 상태를 제거
    //! @param[in] cutoffTime - Measurements and states older than this time will be dropped.
    //! input : 이 시간보다 오래된 측정 및 상태는 삭제
    void clearExpiredHistory(const double cutoffTime);

    //! @brief Clears measurement queue
    //! 측정 queue을 지움
    void clearMeasurementQueue();

    //! @brief Adds a diagnostic message to the accumulating map and updates the error level
    //! 누적 맵에 진단 메시지를 추가하고 오류 수준을 업데이트
    //! @param[in] errLevel - The error level of the diagnostic
    //! input : 진단의 오류 수준
    //! @param[in] topicAndClass - The sensor topic (if relevant) and class of diagnostic
    //! input : 센서 주제(해당되는 경우) 및 진단 클래스
    //! @param[in] message - Details of the diagnostic
    //! input : 진단 세부 정보
    //! @param[in] staticDiag - Whether or not this diagnostic information is static
    //! input :  이 진단 정보가 정적인지 여부
    void addDiagnostic(const int errLevel,
                       const std::string &topicAndClass,
                       const std::string &message,
                       const bool staticDiag);

    //! @brief Aggregates all diagnostics so they can be published
    //! 모든 진단을 집계하여 게시
    //! @param[in] wrapper - The diagnostic status wrapper to update
    //! input : 업데이트할 진단 상태 포장
    void aggregateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &wrapper);

    //! @brief Utility method for copying covariances from ROS covariance arrays
    //! to Eigen matrices
    //! ROS 공분산 배열에서 고유 행렬로 공분산을 복사하는 유틸리티 메서드
    //! This method copies the covariances and also does some data validation, which is
    //! why it requires more parameters than just the covariance containers.
    //! 이 방법은 공분산을 복사하고 일부 데이터 유효성 검사도 수행하므로 공분산 컨테이너보다 더 많은 매개변수가 필요

    //! @param[in] arr - The source array for the covariance data
    //! input : 공분산 데이터의 소스 배열
    //! @param[in] covariance - The destination matrix for the covariance data
    //! input : 공분산 데이터의 대상 행렬
    //! @param[in] topicName - The name of the source data topic (for debug purposes)
    //! input : 소스 데이터 주제의 이름(디버그용)
    //! @param[in] updateVector - The update vector for the source topic
    //! input : 소스 토픽의 업데이트 벡터
    //! @param[in] offset - The "starting" location within the array/update vector
    //! input : 배열/업데이트 벡터 내의 "시작" 위치
    //! @param[in] dimension - The number of values to copy, starting at the offset
    //! input : 오프셋에서 시작하여 복사할 값의 수
    void copyCovariance(const double *arr,
                        Eigen::MatrixXd &covariance,
                        const std::string &topicName,
                        const std::vector<int> &updateVector,
                        const size_t offset,
                        const size_t dimension);

    //! @brief Utility method for copying covariances from Eigen matrices to ROS
    //! covariance arrays
    //! 고유 행렬에서 ROS 공분산 배열로 공분산을 복사하는 유틸리티 방법
    //! @param[in] covariance - The source matrix for the covariance data
    //! input : 공분산 데이터의 소스 행렬
    //! @param[in] arr - The destination array
    //! input : 대상 배열
    //! @param[in] dimension - The number of values to copy
    //! input : 복사할 값의 수
    void copyCovariance(const Eigen::MatrixXd &covariance,
                        double *arr,
                        const size_t dimension);

    //! @brief Loads fusion settings from the config file
    //! config 파일에서 퓨전 설정을 로드
    //! @param[in] topicName - The name of the topic for which to load settings
    //! input : 설정을 로드할 주제의 이름
    //! @return The boolean vector of update settings for each variable for this topic
    //! return :  이 주제에 대한 각 변수에 대한 업데이트 설정의 boolean 벡터
    std::vector<int> loadUpdateConfig(const std::string &topicName);

    //! @brief Prepares an IMU message's linear acceleration for integration into the filter
    //! 필터에 통합하기 위해 IMU 메시지의 선형 가속을 준비
    //! @param[in] msg - The IMU message to prepare
    //! input : 준비할 IMU 메시지
    //! @param[in] topicName - The name of the topic over which this message was received
    //! input : 이 메시지가 수신된 주제의 이름
    //! @param[in] targetFrame - The target tf frame
    //! input : 대상 tf 프레임
    //! @param[in] updateVector - The update vector for the data source
    //! input : 데이터 소스의 업데이트 벡터
    //! @param[in] measurement - The twist data converted to a measurement
    //! input : 측정으로 변환된 twist 데이터
    //! @param[in] measurementCovariance - The covariance of the converted measurement
    //! input : 변환된 측정의 공분산
    bool prepareAcceleration(const sensor_msgs::Imu::ConstPtr &msg,
                             const std::string &topicName,
                             const std::string &targetFrame,
                             const bool relative,
                             std::vector<int> &updateVector,
                             Eigen::VectorXd &measurement,
                             Eigen::MatrixXd &measurementCovariance);

    //! @brief Prepares a pose message for integration into the filter
    //! 필터에 통합하기 위한 포즈 메시지 준비
    //! @param[in] msg - The pose message to prepare
    //! input : 준비할 포즈 메시지
    //! @param[in] topicName - The name of the topic over which this message was received
    //! input : 이 메시지가 수신된 주제의 이름
    //! @param[in] targetFrame - The target tf frame
    //! input : 대상 tf 프레임
    //! @param[in] sourceFrame - The source tf frame
    //! input : 소스 tf 프레임
    //! @param[in] differential - Whether we're carrying out differential integration
    //! input : 미분 적분을 수행하는지 여부
    //! @param[in] relative - Whether this measurement is processed relative to the first
    //! input : 이 측정이 첫 번째에 상대적으로 처리되는지 여부
    //! @param[in] imuData - Whether this measurement is from an IMU
    //! input : 이 측정이 IMU에서 온 것인지 여부
    //! @param[in,out] updateVector - The update vector for the data source
    //! input, output : 데이터 소스의 업데이트 벡터
    //! @param[out] measurement - The pose data converted to a measurement
    //! output : 측정으로 변환된 포즈 데이터
    //! @param[out] measurementCovariance - The covariance of the converted measurement
    //! output : 변환된 측정의 공분산
    //! @return true indicates that the measurement was successfully prepared, false otherwise
    //! return : true는 측정이 성공적으로 준비되었음을 나타내고, 그렇지 않으면 false를 나타냄
    bool preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                     const std::string &topicName,
                     const std::string &targetFrame,
                     const std::string &sourceFrame,
                     const bool differential,
                     const bool relative,
                     const bool imuData,
                     std::vector<int> &updateVector,
                     Eigen::VectorXd &measurement,
                     Eigen::MatrixXd &measurementCovariance);

    //! @brief Prepares a twist message for integration into the filter
    //! 필터에 통합하기 위한 트위스트 메시지 준비
    //! @param[in] msg - The twist message to prepare
    //! input : 준비할 트위스트 메시지
    //! @param[in] topicName - The name of the topic over which this message was received
    //! input : 이 메시지가 수신된 주제의 이름
    //! @param[in] targetFrame - The target tf frame
    //! input : 대상 tf 프레임
    //! @param[in,out] updateVector - The update vector for the data source
    //! input,output : 데이터 소스의 업데이트 벡터
    //! @param[out] measurement - The twist data converted to a measurement
    //! output : 측정으로 변환된 twist 데이터
    //! @param[out] measurementCovariance - The covariance of the converted measurement
    //! output : 변환된 측정의 공분산 
    //! @return true indicates that the measurement was successfully prepared, false otherwise
    //! return : true는 측정이 성공적으로 준비되었음을 나타내고, 그렇지 않으면 false를 나타냄
    bool prepareTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                      const std::string &topicName,
                      const std::string &targetFrame,
                      std::vector<int> &updateVector,
                      Eigen::VectorXd &measurement,
                      Eigen::MatrixXd &measurementCovariance);


    //! @brief callback function which is called for periodic updates
    //! 주기적 업데이트를 위해 호출되는 콜백 함수
    void periodicUpdate(const ros::TimerEvent& event);

    //! @brief Start the Filter disabled at startup
    //! 시작 시 비활성화된 필터 시작
    //! If this is true, the filter reads parameters and prepares publishers and subscribes
    //! but does not integrate new messages into the state vector.
    //! The filter can be enabled later using a service.
    //! 사실이면 필터는 매개변수를 읽고 게시자와 구독을 준비하지만 새 메시지를 상태 벡터에 통합하지 않음
    //! 필터는 나중에 서비스를 사용하여 활성화할 수 있음
    bool disabledAtStartup_;

    //! @brief Whether the filter is enabled or not. See disabledAtStartup_.
    //! 필터 사용 여부. disabledAtStartup_을 참조
    bool enabled_;

    //! Whether we'll allow old measurements to cause a re-publication of the updated state
    //! 오래된 측정이 업데이트된 상태를 다시 게시하도록 허용할지 여부
    bool permitCorrectedPublication_;

    //! @brief By default, the filter predicts and corrects up to the time of the latest measurement. If this is set
    //! to true, the filter does the same, but then also predicts up to the current time step.
    //! 기본적으로 필터는 최신 측정 시간까지 예측하고 수정
    //! true로 설정하면 필터가 동일한 작업을 수행하지만 현재 시간 단계까지 예측
    bool predictToCurrentTime_;

    //! @brief Whether or not we print diagnostic messages to the /diagnostics topic
    //! diagnostics 주제에 진단 메시지를 인쇄할지 여부
    bool printDiagnostics_;

    //! @brief Whether we publish the acceleration
    //! 가속을 게시할지 여부
    bool publishAcceleration_;

    //! @brief Whether we publish the transform from the world_frame to the base_link_frame
    //! world_frame에서 base_link_frame으로 변환을 게시할지 여부
    bool publishTransform_;

    //! @brief Whether to reset the filters when backwards jump in time is detected
    //! 시간 역행이 감지될 때 필터를 재설정할지 여부
    //! This is usually the case when logs are being used and a jump in the logi
    //! is done or if a log files restarts from the beginning.
    //! 일반적으로 로그가 사용 중이고 로그의 점프가 완료되거나 로그 파일이 처음부터 다시 시작되는 경우
    bool resetOnTimeJump_;

    //! @brief Whether or not we use smoothing
    //! smoothing 사용 여부
    bool smoothLaggedData_;

    //! @brief Whether the filter should process new measurements or not.
    //! 필터가 새로운 측정을 처리해야 하는지 여부
    bool toggledOn_;

    //! @brief Whether or not we're in 2D mode
    //! 2D 모드인지 여부
    //! If this is true, the filter binds all 3D variables (Z,
    //! roll, pitch, and their respective velocities) to 0 for
    //! every measurement.
    //! 사실이면 필터는 모든 측정에 대해 모든 3D 변수(Z, 롤, 피치 및 해당 속도)를 0으로 바인딩
    bool twoDMode_;

    //! @brief Whether or not we use a control term
    //! 제어 텀을 사용하는지 여부
    bool useControl_;

    //! @brief When true, do not print warnings for tf lookup failures.
    //! true인 경우 tf 조회 실패에 대한 경고를 인쇄하지 않음
    bool tfSilentFailure_;

    //! @brief The max (worst) dynamic diagnostic level.
    //! 최대(최악) 동적 진단 수준
    int dynamicDiagErrorLevel_;

    //! @brief The max (worst) static diagnostic level.
    //! 최대(최악) 정적 진단 수준
    int staticDiagErrorLevel_;

    //! @brief The frequency of the run loop
    //! 런 루프의 주파수
    double frequency_;

    //! @brief What is the acceleration in Z due to gravity (m/s^2)? Default is +9.80665.
    //! 중력으로 인한 Z의 가속도(m/s^2)는 무엇입니까? 기본값은 +9.80665
    double gravitationalAcc_;

    //! @brief The depth of the history we track for smoothing/delayed measurement processing
    //! smoothing/지연 측정 처리를 위해 추적하는 기록의 깊이
    //! This is the guaranteed minimum buffer size for which previous states and measurements are kept.
    //! 이것은 이전 상태와 측정이 유지되는 보장된 최소 버퍼 크기
    double historyLength_;

    //! @brief minimal frequency
    //! 간단한 최소 빈도
    double minFrequency_;

    //! @brief maximal frequency
    //! 간단한 최대 주파수
    double maxFrequency_;

    //! @brief tf frame name for the robot's body frame
    //! 로봇 본체 프레임의 프레임 이름
    std::string baseLinkFrameId_;

    //! @brief tf frame name for the robot's body frame
    //! 로봇 본체 프레임의 프레임 이름
    //! When the final state is computed, we "override" the output transform and message to have this frame for its
    //! child_frame_id. This helps to enable disconnected TF trees when multiple EKF instances are being run.
    //! 🌟️ 최종 상태가 계산될 때 우리는 출력 변환과 메시지를 "override"하여 이 프레임의 child_frame_id를 가짐
    //! 이는 여러 EKF 인스턴스가 실행 중일 때 연결이 끊긴 TF 트리를 활성화하는 데 도움 🌟️
    std::string baseLinkOutputFrameId_;

    //! @brief tf frame name for the robot's map (world-fixed) frame
    //! 로봇의 맵(world-fixed) 프레임에 대한 프레임 이름
    std::string mapFrameId_;

    //! @brief tf frame name for the robot's odometry (world-fixed) frame
    //! 로봇의 주행 거리(world-fixed) 프레임에 대한 프레임 이름
    std::string odomFrameId_;

    //! @brief tf frame name that is the parent frame of the transform that this node will calculate and broadcast.
    //! 이 노드가 계산하고 브로드캐스트할 변환의 상위 프레임인 프레임 이름
    std::string worldFrameId_;

    //! @brief Used for outputting debug messages
    //! 디버그 메시지 출력에 사용
    std::ofstream debugStream_;

    //! @brief Contains the state vector variable names in string format
    //! 문자열 형식의 상태 벡터 변수 이름을 포함
    std::vector<std::string> stateVariableNames_;

    //! @brief Vector to hold our subscribers until they go out of scope
    //! Vector는 구독자가 범위를 벗어날 때까지 유지
    std::vector<ros::Subscriber> topicSubs_;

    //! @brief This object accumulates dynamic diagnostics, e.g., diagnostics relating
    //! to sensor data.
    //! 이 개체는 동적 진단(예: 센서 데이터와 관련된 진단)을 누적
    //! The values are considered transient and are cleared at every iteration.
    //! 값은 일시적인 것으로 간주되며 모든 반복에서 지워짐
    std::map<std::string, std::string> dynamicDiagnostics_;

    //! @brief Stores the first measurement from each topic for relative measurements
    //! 상대 측정을 위해 각 주제의 첫 번째 측정을 저장
    //! When a given sensor is being integrated in relative mode, its first measurement
    //! is effectively treated as an offset, and future measurements have this first
    //! measurement removed before they are fused. This variable stores the initial
    //! measurements. Note that this is different from using differential mode, as in
    //! differential mode, pose data is converted to twist data, resulting in boundless
    //! error growth for the variables being fused. With relative measurements, the
    //! vehicle will start with a 0 heading and position, but the measurements are still
    //! fused absolutely.

    //! 주어진 센서가 상대 모드에서 통합될 때 첫 번째 측정은 효과적으로 오프셋으로 처리되고 향후 측정에서는 융합되기 전에 이 첫 번째 측정이 제거
    //! 이 변수는 초기 측정값을 저장
    //! 이것은 미분 모드를 사용하는 것과는 다른데, 미분 모드에서 포즈 데이터가 twist 데이터로 변환되어 융합되는 변수에 대해 무한한 오류 증가가 발생한다는 점에 유의
    //! 상대 측정을 통해 차량은 0 방향 및 위치로 시작하지만 측정은 여전히 절대적으로 융합
    std::map<std::string, tf2::Transform> initialMeasurements_;

    //! @brief Store the last time a message from each topic was received
    //!  메시지를 빠르게 받는 경우 실수로 새 메시지 다음에 이전 메시지가 도착할 수 있음
    //! If we're getting messages rapidly, we may accidentally get
    //! an older message arriving after a newer one. This variable keeps
    //! track of the most recent message time for each subscribed message
    //! topic. We also use it when listening to odometry messages to
    //! determine if we should be using messages from that topic.

    //! 이 변수는 구독된 각 메시지 주제에 대한 가장 최근 메시지 시간을 추적
    //! 또한 해당 주제의 메시지를 사용해야 하는지 여부를 결정하기 위해 주행 거리 측정 메시지를 수신할 때 사용
    std::map<std::string, ros::Time> lastMessageTimes_;

    //! @brief We also need the previous covariance matrix for differential data
    //! 미분 데이터에 대한 이전 공분산 행렬도 필요
    std::map<std::string, Eigen::MatrixXd> previousMeasurementCovariances_;

    //! @brief Stores the last measurement from a given topic for differential integration
    //! 미분 통합을 위해 주어진 주제의 마지막 측정값을 저장
    //! To carry out differential integration, we have to (1) transform
    //! that into the target frame (probably the frame specified by
    //! @p odomFrameId_), (2) "subtract"  the previous measurement from
    //! the current measurement, and then (3) transform it again by the previous
    //! state estimate. This holds the measurements used for step (2).

    //! 미분 적분을 수행하려면 (1) 대상 프레임(아마도 @p odomFrameId_에 의해 지정된 프레임)으로 변환
    //! (2) 현재 측정에서 이전 측정을 "빼기"한 다음 (3) 이전 상태 추정으로 다시 변환
    //! 이것은 단계 (2)에 사용된 측정을 유지
    std::map<std::string, tf2::Transform> previousMeasurements_;

    //! @brief If including acceleration for each IMU input, whether or not we remove acceleration due to gravity
    //! 각 IMU 입력에 대한 가속도를 포함하면 중력 가속도 제거 여부
    std::map<std::string, bool> removeGravitationalAcc_;

    //! @brief This object accumulates static diagnostics, e.g., diagnostics relating
    //! to the configuration parameters.
    //! 이 개체는 구성 매개변수와 관련된 진단과 같은 정적 진단을 누적
    //! The values are treated as static and always reported (i.e., this object is never cleared)
    //! 값은 정적으로 처리되고 항상 보고(즉, 이 개체는 지워지지 않음)
    std::map<std::string, std::string> staticDiagnostics_;

    //! @brief Last time mark that time-differentiation is calculated
    //! 시간-미분이 계산되는 마지막 시간 표시
    ros::Time lastDiffTime_;

    //! @brief Last record of filtered angular velocity
    //! 필터링된 각속도의 마지막 기록
    tf2::Vector3 lastStateTwistRot_;

    //! @brief Calculated angular acceleration from time-differencing
    //! 시차로부터 계산된 각가속도
    tf2::Vector3 angular_acceleration_;

    //! @brief Covariance of the calculated angular acceleration
    //! 계산된 각가속도의 공분산
    Eigen::MatrixXd angular_acceleration_cov_;

    //! @brief The most recent control input
    //! 가장 최근의 제어 입력
    Eigen::VectorXd latestControl_;

    //! @brief Message that contains our latest transform (i.e., state)
    //! 최신 변환(즉, 상태)을 포함하는 메시지
    //! We use the vehicle's latest state in a number of places, and often
    //! use it as a transform, so this is the most convenient variable to
    //! use as our global "current state" object
    //! 우리는 차량의 최신 상태를 여러 곳에서 사용하고 종종 TF으로 사용하므로 전역 "현재 상태" 개체로 사용하기에 가장 편리한 변수
    geometry_msgs::TransformStamped worldBaseLinkTransMsg_;

    //! @brief last call of periodicUpdate
    //! 주기적 업데이트의 마지막 호출
    ros::Time lastDiagTime_;

    //! @brief The time of the most recent published state
    //! 가장 최근에 게시된 상태의 시간
    ros::Time lastPublishedStamp_;

    //! @brief Store the last time set pose message was received
    //! 마지막으로 설정된 포즈 메시지가 수신된 시간
    //! If we receive a setPose message to reset the filter, we can get in
    //! strange situations wherein we process the pose reset, but then even
    //! after another spin cycle or two, we can get a new message with a time
    //! stamp that is *older* than the setPose message's time stamp. This means
    //! we have to check the message's time stamp against the lastSetPoseTime_.

    //! 필터를 재설정하라는 setPose 메시지를 받으면 포즈 재설정을 처리하는 이상한 상황에 처할 수 있지만
    //! 한 두 번 더 회전한 후에도 다음보다 *오래된* 타임스탬프가 있는 새 메시지를 받을 수 있음
    //! 이것은 lastSetPoseTime_에 대해 메시지의 타임스탬프를 확인해야 함을 의미
    ros::Time lastSetPoseTime_;

    //! @brief The time of the most recent control input
    //! 가장 최근의 제어 입력 시간
    ros::Time latestControlTime_;

    //! @brief For future (or past) dating the world_frame->base_link_frame transform
    //! 미래(또는 과거) world_frame->base_link_frame 변환
    ros::Duration tfTimeOffset_;

    //! @brief Parameter that specifies the how long we wait for a transform to become available.
    //! tf을 사용할 수 있을 때까지 기다리는 시간을 지정하는 매개변수
    ros::Duration tfTimeout_;

    //! @brief Service that allows another node to toggle on/off filter processing while still publishing.
    //! Uses a robot_localization ToggleFilterProcessing service.
    //! 게시하는 동안 다른 노드가 필터 처리를 켜고 끌 수 있도록 하는 서비스
    ros::ServiceServer toggleFilterProcessingSrv_;

    //! @brief timer calling periodicUpdate
    //! 주기적 업데이트를 호출하는 타이머
    ros::Timer periodicUpdateTimer_;

    //! @brief An implicitly time ordered queue of past filter states used for smoothing.
    // Smoothing에 사용된 과거 필터 상태의 암시적으로 시간 순서가 지정된 Queue
    // front() refers to the filter state with the earliest timestamp.
    // back() refers to the filter state with the latest timestamp.
    // front()는 가장 빠른 타임스탬프가 있는 필터 상태
    // back()은 최신 타임스탬프가 있는 필터 상태를 참조
    FilterStateHistoryDeque filterStateHistory_;

    //! @brief A deque of previous measurements which is implicitly ordered from earliest to latest measurement.
    //! 가장 이른 측정에서 최신 측정으로 암시적으로 정렬된 이전 측정의 Deque
    // when popped from the measurement priority queue.
    // 측정 우선순위 queue에서 pop(꺼낼 때)일 때
    // front() refers to the measurement with the earliest timestamp.
    // back() refers to the measurement with the latest timestamp.
    // front()는 가장 빠른 타임스탬프가 있는 필터 상태
    // back()은 최신 타임스탬프가 있는 필터 상태를 참조
    MeasurementHistoryDeque measurementHistory_;

    //! @brief We process measurements by queueing them up in
    //! callbacks and processing them all at once within each
    //! iteration
    //! 측정을 콜백에서 큐에 넣고 각 반복 내에서 한 번에 모두 처리하여 측정을 처리
    MeasurementQueue measurementQueue_;

    //! @brief Our filter (EKF, UKF, etc.)
    //! 우리 필터(EKF, UKF 등)
    T filter_;

    //! @brief Node handle
    //! 노드 핸들
    ros::NodeHandle nh_;

    //! @brief Local node handle (for params)
    //! 로컬 노드 핸들(params용)
    ros::NodeHandle nhLocal_;

    //! @brief optional acceleration publisher
    //! 선택적 가속 게시자
    ros::Publisher accelPub_;

    //! @brief position publisher
    //! 위치 게시자
    ros::Publisher positionPub_;

    //! @brief Subscribes to the control input topic
    //! 제어 입력 항목을 구독
    ros::Subscriber controlSub_;

    //! @brief Subscribes to the set_pose topic (usually published from rviz). Message
    //! type is geometry_msgs/PoseWithCovarianceStamped.
    //! set_pose 주제를 구독합니다(일반적으로 rviz에서 게시됨)
    ros::Subscriber setPoseSub_;

    //! @brief Service that allows another node to enable the filter. Uses a standard Empty service.
    //! 다른 노드가 필터를 활성화할 수 있도록 하는 서비스
    ros::ServiceServer enableFilterSrv_;

    //! @brief Service that allows another node to change the current state and recieve a confirmation. Uses
    //! a custom SetPose service.
    //! 다른 노드가 현재 상태를 변경하고 확인을 받을 수 있도록 하는 서비스
    ros::ServiceServer setPoseSrv_;

    //! @brief Used for updating the diagnostics
    //! 진단 업데이트에 사용
    diagnostic_updater::Updater diagnosticUpdater_;

    //! @brief Transform buffer for managing coordinate transforms
    //! 좌표 변환 관리를 위한 변환 버퍼
    tf2_ros::Buffer tfBuffer_;

    //! @brief Transform listener for receiving transforms
    //! 변환 수신을 위한 변환 수신기
    tf2_ros::TransformListener tfListener_;

    //! @brief broadcaster of worldTransform tfs
    //! worldTransform tfs의 브로드캐스터
    tf2_ros::TransformBroadcaster worldTransformBroadcaster_;

    //! @brief optional signaling diagnostic frequency
    //! 선택적 신호 진단 주파수
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freqDiag_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_ROS_FILTER_H
