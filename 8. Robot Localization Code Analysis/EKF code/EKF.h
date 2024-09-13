#ifndef ROBOT_LOCALIZATION_EKF_H
#define ROBOT_LOCALIZATION_EKF_H

#include "robot_localization/filter_base.h"

#include <fstream>
#include <vector>
#include <set>
#include <queue>

namespace RobotLocalization
{

//! @brief Extended Kalman filter class
//!
//! Implementation of an extended Kalman filter (EKF). This
//! class derives from FilterBase and overrides the predict()
//! and correct() methods in keeping with the discrete time
//! EKF algorithm.

//! 확장 칼만 필터(EKF) 구현
//! 이 클래스는 FilterBase에서 파생되며 이산 시간 EKF 알고리즘에 따라 predict() 및 correct() 메서드를 재정의
class Ekf: public FilterBase
{
  public:
    //! @brief Constructor for the Ekf class
    //! EKF 클래스를 위한 생성자
    //! @param[in] args - Generic argument container (not used here, but
    //! needed so that the ROS filters can pass arbitrary arguments to
    //! templated filter types).
    //! input : args > 일반 인수 컨테이너
    //! 여기에서는 사용되지 않지만 ROS 필터가 템플릿 필터 유형에 임의의 인수를 전달할 수 있도록 필요
    explicit Ekf(std::vector<double> args = std::vector<double>());

    //! @brief Destructor for the Ekf class
    //! EKF 클래스를 위한 소멸자
    ~Ekf();

    //! @brief Carries out the correct step in the predict/update cycle.
    //! 예측/업데이트 주기에서 업데이트 단계를 수행
    //! @param[in] measurement - The measurement to fuse with our estimate
    //! input : measurement > 우리의 추정치와 융합하는 측정
    void correct(const Measurement &measurement);

    //! @brief Carries out the predict step in the predict/update cycle.
    //! 예측/업데이트 주기에서 예측 단계를 수행
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion.
    //! 차량 모션 모델을 사용하여 상태 및 오류 행렬을 앞으로 투영
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //! input : referenceTime(예측이 이루어지는 시간), delta(예측할 시간 단계)
    void predict(const double referenceTime, const double delta);
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_EKF_H