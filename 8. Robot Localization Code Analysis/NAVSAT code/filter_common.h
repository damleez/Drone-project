#ifndef ROBOT_LOCALIZATION_FILTER_COMMON_H
#define ROBOT_LOCALIZATION_FILTER_COMMON_H

namespace RobotLocalization
{

//! @brief Enumeration that defines the state vector
//!
enum StateMembers //사용자 정의 자료형 :  (ㅌ열거형
{
  StateMemberX = 0, //assigned 0 이므로
  StateMemberY,     //assigned 1 이 됨
  StateMemberZ,         
  StateMemberRoll,
  StateMemberPitch,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz     //assigned 14
};

//! @brief Enumeration that defines the control vector
//! brief의 뜻은 짧은 summary를 적을 때 사용
enum ControlMembers
{
  ControlMemberVx,  //기본 assigned 0 
  ControlMemberVy,
  ControlMemberVz,
  ControlMemberVroll,
  ControlMemberVpitch,
  ControlMemberVyaw //assigned 5
};

//! @brief Global constants that define our state
//! vector size and offsets to groups of values
//! within that state.
const int STATE_SIZE = 15;
const int POSITION_OFFSET = StateMemberX;
const int ORIENTATION_OFFSET = StateMemberRoll;
const int POSITION_V_OFFSET = StateMemberVx;
const int ORIENTATION_V_OFFSET = StateMemberVroll;
const int POSITION_A_OFFSET = StateMemberAx;

//! @brief Pose and twist messages each
//! contain six variables
const int POSE_SIZE = 6;
const int TWIST_SIZE = 6;
const int POSITION_SIZE = 3;
const int ORIENTATION_SIZE = 3;
const int LINEAR_VELOCITY_SIZE = 3;
const int ACCELERATION_SIZE = 3;

//! @brief Common variables
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_FILTER_COMMON_H