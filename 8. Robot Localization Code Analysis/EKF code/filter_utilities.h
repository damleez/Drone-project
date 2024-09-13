#ifndef ROBOT_LOCALIZATION_FILTER_UTILITIES_H //헤더파일 중복포함 방지
#define ROBOT_LOCALIZATION_FILTER_UTILITIES_H //헤더파일 중복포함 방지

#include <Eigen/Dense> //행렬, 벡터 등 선형대수학을 위한 헤더
                       //! @brief Input Output MANIPmanipulation:입출력 조정자
#include <iomanip>     //! 출력(cout) 포맷 변경 가능
#include <iostream>    //기본 입출력 헤더
#include <string>      //문자열 헤더
#include <vector>      //벡터 헤더

#define FB_DEBUG(msg) if (getDebug()) { *debugStream_ << msg; } //만약 getdebug면 debugstream(포인터 > filter_base에 있음)는 msg를 출력함

//Handy methods for debug output
//ostream 클래스 객체와 MatrixXd객체 두 개를 인자로 받는 전역 operator<< 함수를 정의
//call by reference > &mat
//이로인해 출력과 mat객체 따로가 아닌 같이 출력 1+"p" > "1+p"
std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd &mat);
std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<size_t> &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<int> &vec);

namespace RobotLocalization
{
namespace FilterUtilities
{
  //RPY를 위해 rotation을 받음 -pi~pi
  //! @brief Utility method keeping RPY angles in the range [-pi, pi]
  //! @param[in] rotation - The rotation to bind
  //! @return the bounded value
  //!
  double clampRotation(double rotation);

  //tf를 위해 tfPrefix와 frameId를 받음(frame id+tfprefix > new frameid인듯)
  //! @brief Utility method for appending tf2 prefixes cleanly
  //! @param[in] tfPrefix - the tf2 prefix to append
  //! @param[in, out] frameId - the resulting frame_id value
  //!
  void appendPrefix(std::string tfPrefix, std::string &frameId);

}  // namespace FilterUtilities
}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_FILTER_UTILITIES_H
        //헤더파일 중복포함 방지 끝(ifndef,define,endif set)