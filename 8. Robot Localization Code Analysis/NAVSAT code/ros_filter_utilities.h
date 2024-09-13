#ifndef ROBOT_LOCALIZATION_ROS_FILTER_UTILITIES_H
#define ROBOT_LOCALIZATION_ROS_FILTER_UTILITIES_H

#include <tf2/LinearMath/Quaternion.h>  //tf
#include <tf2/LinearMath/Transform.h>   
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

#include <iomanip>                      //출력(cout)포맷 변경 가능
#include <iostream>                     //입출력
#include <string>                       //문자열
#include <vector>                       //벡터

//여기서 filter_는 ros_filter.h에 있으며, T filter_로 정의 
//T = 템플릿
#define RF_DEBUG(msg) if (filter_.getDebug()) { debugStream_ << msg; }

// Handy methods for debug output
// ostream 클래스 객체와 Vector3 두 개를 인자로 받는 전역 operator<< 함수를 정의
// 걍 출력 = 출력+vector3의 vec
std::ostream& operator<<(std::ostream& os, const tf2::Vector3 &vec);
std::ostream& operator<<(std::ostream& os, const tf2::Quaternion &quat);
std::ostream& operator<<(std::ostream& os, const tf2::Transform &trans);
std::ostream& operator<<(std::ostream& os, const std::vector<double> &vec);

namespace RobotLocalization
{
namespace RosFilterUtilities
{

// yaw값을 받음(tf 쿼터니안을 통해)
double getYaw(const tf2::Quaternion quat);

//! @brief Method for safely obtaining transforms.
//! 안전하게 변환을 얻는 방법
//! @param[in] buffer - tf buffer object to use for looking up the transform
//! 변환을 찾는 데 사용할 tf 버퍼 객체
//! @param[in] targetFrame - The target frame of the desired transform
//! input : targetFrame > 원하는 변환의 대상 프레임
//! @param[in] sourceFrame - The source frame of the desired transform
//! input : sourceFrame > 원하는 변환의 소스 프레임
//! @param[in] time - The time at which we want the transform
//! input : time > 변환을 원하는 시간
//! @param[in] timeout - How long to block before falling back to last transform
//! input : timeout > 마지막 변환으로 돌아가기 전에 차단할 시간
//! @param[out] targetFrameTrans - The resulting transform object
//! output : targetFrameTrans > 변환 객체 결과
//! @param[in] silent - Whether or not to print transform warnings
//! input : silent > 변환 경고를 인쇄할지 여부
//! @return Sets the value of @p targetFrameTrans and returns true if successful,
//! false otherwise.
//! return : targetFrameTrans의 값을 설정하고 성공하면 true를, 그렇지 않으면 false를 반환

//! This method attempts to obtain a transform from the @p sourceFrame to the @p
//! targetFrame at the specific @p time. If no transform is available at that time,
//! it attempts to simply obtain the latest transform. If that still fails, then the
//! method checks to see if the transform is going from a given frame_id to itself.
//! If any of these checks succeed, the method sets the value of @p targetFrameTrans
//! and returns true, otherwise it returns false.

//! 이 메서드는 특정 @p 시간에 @p sourceFrame에서 @p targetFrame으로의 변환을 얻으려고 시도
//! 그 때 사용할 수 있는 변환이 없으면 단순히 최신 변환을 얻으려고 시도
//! 그래도 실패하면 메서드는 변환이 지정된 frame_id에서 그자체로 이동하는지 확인
//! 하나라도 성공하면 메서드는 @p targetFrameTrans의 값을 설정하고 true를 반환, 그렇지 않으면 false를 반환
//! 👉️ 함수 : bool형 lookupTransformSafe함수
bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame,
                         const ros::Time &time,
                         const ros::Duration &timeout,
                         tf2::Transform &targetFrameTrans,
                         const bool silent = false);

//! @brief Method for safely obtaining transforms.
//! 안전하게 변환을 얻는 방법
//! @param[in] buffer - tf buffer object to use for looking up the transform
//! 변환을 찾는 데 사용할 tf 버퍼 객체
//! @param[in] targetFrame - The target frame of the desired transform
//! input : targetFrame > 원하는 변환의 대상 프레임
//! @param[in] sourceFrame - The source frame of the desired transform
//! input : sourceFrame > 원하는 변환의 소스 프레임
//! @param[in] time - The time at which we want the transform
//! input : time > 변환을 원하는 시간
//! @param[out] targetFrameTrans - The resulting transform object
//! output : targetFrameTrans > 변환 객체 결과
//! @param[in] silent - Whether or not to print transform warnings
//! input : silent > 변환 경고를 인쇄할지 여부
//! @return Sets the value of @p targetFrameTrans and returns true if successful,
//! false otherwise.
//! return : targetFrameTrans의 값을 설정하고 성공하면 true를, 그렇지 않으면 false를 반환

//! This method attempts to obtain a transform from the @p sourceFrame to the @p
//! targetFrame at the specific @p time. If no transform is available at that time,
//! it attempts to simply obtain the latest transform. If that still fails, then the
//! method checks to see if the transform is going from a given frame_id to itself.
//! If any of these checks succeed, the method sets the value of @p targetFrameTrans
//! and returns true, otherwise it returns false.

//! 이 메서드는 특정 @p 시간에 @p sourceFrame에서 @p targetFrame으로의 변환을 얻으려고 시도
//! 그 때 사용할 수 있는 변환이 없으면 단순히 최신 변환을 얻으려고 시도
//! 그래도 실패하면 메서드는 변환이 지정된 frame_id에서 그자체로 이동하는지 확인
//! 하나라도 성공하면 메서드는 @p targetFrameTrans의 값을 설정하고 true를 반환, 그렇지 않으면 false를 반환
//! 👉️ 함수 : bool형 lookupTransformSafe함수
bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame,
                         const ros::Time &time,
                         tf2::Transform &targetFrameTrans,
                         const bool silent = false);

//! @brief Utility method for converting quaternion to RPY
//! 쿼터니언을 RPY로 변환하는 유틸리티 메서드
//! @param[in] quat - The quaternion to convert
//! input : 변환할 쿼터니언
//! @param[out] roll - The converted roll
//! output : 변환된 roll
//! @param[out] pitch - The converted pitch
//! output : 변환된 pitch
//! @param[out] yaw - The converted yaw
//! output : 변환된 yaw
//! 👉️ 함수 : 쿼터니안, r, p, y를 멤버변수로 받는 quatToRPY 멤버 함수
void quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw);

//! @brief Converts our Eigen state vector into a TF transform/pose
//! 고유 상태 벡터를 TF 변환/포즈로 변환
//! @param[in] state - The state to convert
//! input : state > 변환할 상태
//! @param[out] stateTF - The converted state
//! output : stateTF > 변환된 상태
//! 👉️ 함수 : 상태, 상태TF를 멤버변수로 받는 stateToTF 멤버 함수
void stateToTF(const Eigen::VectorXd &state, tf2::Transform &stateTF);

//! @brief Converts a TF transform/pose into our Eigen state vector
//! TF 변환/포즈를 고유 상태 벡터로 변환
//! @param[in] stateTF - The state to convert
//! input : stateTF > 변환할 상태
//! @param[out] state - The converted state
//! output : state > 변환된 상태
//! 👉️ 함수 : 상태, 상태TF를 멤버변수로 받는 TFTostate 멤버 함수
void TFtoState(const tf2::Transform &stateTF, Eigen::VectorXd &state);

}  // namespace RosFilterUtilities
}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_ROS_FILTER_UTILITIES_H