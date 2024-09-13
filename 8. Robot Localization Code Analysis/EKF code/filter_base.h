#ifndef ROBOT_LOCALIZATION_FILTER_BASE_H
#define ROBOT_LOCALIZATION_FILTER_BASE_H

#include "robot_localization/filter_utilities.h"
#include "robot_localization/filter_common.h"

#include <Eigen/Dense> //행렬, 벡터 등 선형대수학을 위한 헤더

#include <algorithm>  //stl.검색, 정렬, 원소 수정등 작업가능한 헤더
#include <limits>     //stl.자료형의 최대값 최소값 정의 헤더
#include <map>        //stl.(key(string),value(int))설정하는 헤더
#include <ostream>    //출력 헤더
#include <queue>      //stl.큐(위 아래가 뚫린 형태) 헤더
#include <set>        //stl.원하는 자료형 및 클래스 생성 헤더(중복X, 순서대로 정렬)
#include <string>     //문자열 헤더
#include <vector>     //stl.벡터 헤더

//! @brief //boost : C++ 기반의 템플릿 라이브러리 집합
//! shared_ptr : 스마트포인터로 어떤 하나의 객체를 참조하는 sp의 개수를 참조하는 sp로
//! 수명이 다하면 알아서 delete사용해서 메모리 자동 해제
#include <boost/shared_ptr.hpp> 

namespace RobotLocalization
{

//! @brief Structure used for storing and comparing measurements
//! (for priority queues)
//! 측정값을 저장하고 비교하는데 사용되는 구조로 우선순위는 큐
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in seconds.
//! 측정단위 : m, rad / 시간 : 실제 값이며 s단위
struct Measurement
{
  // The time stamp of the most recent control term (needed for lagged data)
  // 가장 최근의 제어 텀의 타임스탬프(지연된 데이터에 필요)
  double latestControlTime_;

  // The Mahalanobis distance threshold in number of sigmas
  // 마할라노비스의 거리 시그마 수(공분산)의 임계값
  double mahalanobisThresh_;

  // The real-valued time, in seconds, since some epoch
  // (presumably the start of execution, but any will do)
  // 어떤 에포크 이후의 실제 값 시간으로 시작할 때 실행
  double time_;

  // The topic name for this measurement. Needed
  // for capturing previous state values for new
  // measurements.
  // 이 측정값의 토픽 이름,새 측정 위해 이전 상태 값 포착하는데 필요하며 문자열
  std::string topicName_;

  // This defines which variables within this measurement
  // actually get passed into the filter. std::vector<bool>
  // is generally frowned upon, so we use ints.
  // 이 측정 내의 변수가 실제로 필터안에 전달되는 것을 정의 bool대신 int사용
  // ㄴ filter_utiliteies.h에 정의
  std::vector<int> updateVector_;

  // The most recent control vector (needed for lagged data)
  // 최근 제어 벡터(지연된 데이터에 필요)
  Eigen::VectorXd latestControl_;

  // The measurement and its associated covariance
  // 측정 및 관련 공분산 
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd covariance_;

  // We want earlier times to have greater priority
  // 이전 시간이 더 높은 우선 순위를 갖기를 원함

  // ❌️ 함수 매개변수로써 const 참조 : a, b ❌️
  // ❌️ 스마트 포인터의 경우 함수의 매개변수로 사용하기 위해 const참조자로 사용 ❌️
  // ❌️ 즉, Measurement 구조체는 스마트포인터로 함수 매개변수(parameter) a,b를 const(상수-값안변함)참조함 ❌️
  // ❌️ 그리고 연산자 오버로딩인 operator 함수는 원래 const로 만들고 매개변수도 const로 받음 ❌️
  // ❌️ 💥️ call by reference(const &) : 값 전달은 원본의 값이 그대로 사용하며, 참조만 전달 ❌️
  // ❌️ 원본을 전달하기 때문에, 함수안에서 개체에 조작을 행한경우, 함수 종료후에도 영향이 남아있음 💥️ ❌️

  // operator()는 오버로딩이긴한데 밑에 함수를 지정해주면 그대로 수만 넣어도 실행이 되게끔함 그리고 get 자체가 저장하는 역할임 저 a와 b 포인터 저장
  bool operator()(const boost::shared_ptr<Measurement> &a, const boost::shared_ptr<Measurement> &b)
  { 
    // ❓️ *this는 클래스 그 자체 : 그 this클래스에 포인터 a,b를 get으로 저장 ❓️
    return (*this)(*(a.get()), *(b.get()));
  }

  // ❌️ Measurement구조체의 함수 매개변수 a,b를 const참조하며 ❌️
  // ❌️ a.time이 b.time보다 크면 1(true)를 return > bool 함수이기 때문 ❌️
  // ❌️ 💥️ call by reference기때문에 a,b값 영향이 남아있어서 그 값을 들고 올 수 있는듯 💥️ ❌️
  // 위에서 get으로 저장한 포인터 a,b를 여기서 사용해서 atime 이 btime 보다 크면 true 반환
  bool operator()(const Measurement &a, const Measurement &b)
  {
    return a.time_ > b.time_;
  ]

  // Measurement 생성자에서 최근제어시간, 마할노비스쓰레드값(max를 double로), 시간, 토픽네임 초기화
  Measurement() :
    latestControlTime_(0.0),
    mahalanobisThresh_(std::numeric_limits<double>::max()),
    time_(0.0),
    topicName_("")
  {
  }
};
// typedef를 사용하여 타입의 실제 이름이 아니라 별칭 생성
typedef boost::shared_ptr<Measurement> MeasurementPtr;


//! @brief Structure used for storing and comparing filter states
//!
//! This structure is useful when higher-level classes need to remember filter history.
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in seconds.
//! 필터 상태를 저장하고 비교
//! 이 구조는 상위 수준 클래스가 필터 history를 기억해야할 때 유용
//! 측정 단위는 m와 rad / 시간은 실제값이며 s
struct FilterState
{
  // The time stamp of the most recent measurement for the filter
  // 필터에 대한 가장 최근 측정의 타임스탬프
  double lastMeasurementTime_;

  // The time stamp of the most recent control term
  // 가장 최근 제어 텀의 타임스탬프
  double latestControlTime_;

  // The most recent control vector
  // 가장 최근의 제어 벡터
  Eigen::VectorXd latestControl_;

  // The filter state vector
  // 필터 상태 벡터
  Eigen::VectorXd state_;

  // The filter error covariance matrix
  // 필터 오차 공분산 행렬
  Eigen::MatrixXd estimateErrorCovariance_;

  // We want the queue to be sorted from latest to earliest timestamps.
  // 대기열이 최신 타임스탬프에서 가장 빠른 타임스탬프로 정렬되기를 원함
  bool operator()(const FilterState &a, const FilterState &b)
  {
    // int add(int a, int b) return a+b 함수가 있고, int main에서 c=10, d=5, add(c+d) 하면 15되는거랑 같은것
    // 여기서 const FilterState &a = a / a.lastMeasurementTime_=c 인 것임
    // ❓️ 이렇게 이름이 달라도 됨 > 근데 이거 뭐라 서치하면 나올까 ❓️
    return a.lastMeasurementTime_ < b.lastMeasurementTime_;
  }
  // FilterState 생성자에서 마지막측정시간, 최신제어시간 초기화
  FilterState() :
    lastMeasurementTime_(0.0),
    latestControlTime_(0.0)
  {}
};
// typedef를 사용하여 타입의 실제 이름이 아니라 별칭 생성
typedef boost::shared_ptr<FilterState> FilterStatePtr;

//클래스 FilterBase 선언
class FilterBase
{
  public:
    //! @brief Constructor for the FilterBase class
    //! FilterBase 클래스의 생성자
    FilterBase();

    //! @brief Destructor for the FilterBase class
    //! FilterBase 클래스의 소멸자
    //! 👉️ 함수 : 가상함수를 생성자/소멸자에 사용시 만약 클래스 1이 있고 파생 클래스 2가 있음 
    //! 👉️ 함수 : 팩토리 함수(새로 생성된 파생 클래스(2)객체에 대한 기본 클래스(1) 포인터를 반환하는 함수)에서
    //! 👉️ 함수 : 이 포인터가 가리키는 객체 삭제될때 기본 클래스(1)이 삭제됨 그면 팩토리 함수에 대한 파생 클래스 부분이 소멸되지 않음
    virtual ~FilterBase();

    //! @brief Resets filter to its unintialized state
    //! 필터를 초기화되지 않은 상태로 재설정
    void reset();

    //! @brief Computes a dynamic process noise covariance matrix using the parameterized state
    //! 매개변수화된 상태를 사용하여 동적 프로세스 노이즈 공분산 행렬을 계산
    //! This allows us to, e.g., not increase the pose covariance values when the vehicle is not moving
    //! 이를 통해 예를 들어 차량이 움직이지 않을 때 포즈 공분산 값을 증가시키지 않을 수 있음
    //! @param[in] state - The STATE_SIZE state vector that is used to generate the dynamic process noise covariance
    //! 동적 프로세스 노이즈 공분산을 생성하는 데 사용되는 STATE_SIZE 상태 벡터
    //! STATE_SIZE는 filter_common.h에서 정의
    //! 👉️ 함수 : 동적프로세스노이즈공분산 함수는 매개변수로 VectorXd &state와 double delta를 받음
    void computeDynamicProcessNoiseCovariance(const Eigen::VectorXd &state, const double delta);

    //! @brief Carries out the correct step in the predict/update cycle. This method
    //! must be implemented by subclasses.
    //! 예측/업데이트 주기에서 올바른 단계를 수행되며 이 방법은 서브클래스에 의해 구현해야함
    //! @param[in] measurement - The measurement to fuse with the state estimate
    //! 상태 추정과 융합할 예정
    //! 👉️ 함수 : 순수가상함수로 인터페이스만 제공
    //! 👉️ 가상함수란 : virtual 함수는 가상함수로 기본 클래스에 선언되어 파생 클래스에 재정의 (포인터나 참조사용)
    //! 👉️ 가상함수는 실행시간에 함수의 다형성을 구현하는데 사용
    virtual void correct(const Measurement &measurement) = 0;

    //! @brief Returns the control vector currently being used
    //! 현재 사용 중인 제어 벡터를 반환
    //! @return The control vector
    //! 제어 벡터 반환
    //! 👉️ 함수 : const형태(안변함, Eigen::VectorXd)로 getControl함수 선언
    const Eigen::VectorXd& getControl();

    //! @brief Returns the time at which the control term was issued
    //! 제어 텀이 발행된 시간을 return
    //! @return The time the control vector was issued
    //! 제어 벡터가 발행된 시간을 return
    //! 👉️ 함수 : double 형태로 getControlTime 함수 선언
    double getControlTime();

    //! @brief Gets the value of the debug_ variable.
    //! debug_ 변수의 값을 가짐
    //! @return True if in debug mode, false otherwise
    //! 디버그 모드이면 참, 그렇지 않으면 거짓
    //! 👉️ 함수 : bool 형태로 getDebug 함수 선언
    bool getDebug();

    //! @brief Gets the estimate error covariance
    //! 추정 오차 공분산을 가져옴
    //! @return A copy of the estimate error covariance matrix
    //! 추정 오차 공분산 행렬의 복사본을 return
    //! 👉️ 함수 : const형태로 getEstimateErrorCovariance함수 선언
    const Eigen::MatrixXd& getEstimateErrorCovariance();

    //! @brief Gets the filter's initialized status
    //! 필터의 초기화 상태를 가져옴
    //! @return True if we've received our first measurement, false otherwise
    //! 첫 번째 측정을 받으면 True, 그렇지 않으면 False
    //! 👉️ 함수 : bool 형태로 getInitializedStatus 함수 선언
    bool getInitializedStatus();

    //! @brief Gets the most recent measurement time
    //! 가장 최근의 측정 시간을 가져옴
    //! @return The time at which we last received a measurement
    //! 우리가 마지막으로 측정을 받은 시간 return
    //! 👉️ 함수 : double 형태로 getLastMeasurementTime 함수 선언
    double getLastMeasurementTime();

    //! @brief Gets the filter's predicted state, i.e., the
    //! state estimate before correct() is called.
    //! 필터의 예측 상태를 가져옴 correct()가 불려지기 전의 상태 추정
    //! @return A constant reference to the predicted state
    //! 예측된 상태에 대한 상수 참조
    //! 👉️ 함수 : const형태로 getPredictedState함수 선언
    const Eigen::VectorXd& getPredictedState();

    //! @brief Gets the filter's process noise covariance
    //! 필터의 프로세스 노이즈 공분산을 가져옴
    //! @return A constant reference to the process noise covariance
    //! 프로세스 노이즈 공분산에 대한 상수 참조
    //! 👉️ 함수 : const형태로 getProcessNoiseCovariance함수 선언
    const Eigen::MatrixXd& getProcessNoiseCovariance();

    //! @brief Gets the sensor timeout value (in seconds)
    //! 센서 시간 초과 값(초)을 가져옴
    //! @return The sensor timeout value
    //! 센서 시간 초과 값을 return
    //! 👉️ 함수 : double 형태로 getSensorTimeout 함수 선언
    double getSensorTimeout();

    //! @brief Gets the filter state
    //! 필터 상태 얻음
    //! @return A constant reference to the current state
    //! 현재 상태에 대한 상수 참조
    //! 👉️ 함수 : const형태로 getState 함수 선언
    const Eigen::VectorXd& getState();

    //! @brief Carries out the predict step in the predict/update cycle.
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion. This method must be implemented by subclasses.
    //! 예측/업데이트 주기에서 예측 단계를 수행하며 차량 모션 모델을 사용하여 상태 및 오류 행렬을 앞으로 투영
    //! 이 메서드는 서브클래스에서 구현해야 함
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //! 레퍼런스 타임(예측이 이루어지는 시간), 델타(예측할 시간 단계)를 input으로 받음
    //! 👉️ 함수 : 순수가상함수로 인터페이스만 제공 (무조건 자식 클래스 override시켜야함 - 상속)
    virtual void predict(const double referenceTime, const double delta) = 0;

    //! @brief Does some final preprocessing, carries out the predict/update cycle
    //! 일부 최종 전처리를 수행하고 예측/업데이트 주기를 수행
    //! @param[in] measurement - The measurement object to fuse into the filter
    //! 측정을 input으로 받음 : 필터에 융합할 측정 객체
    //! 👉️ 함수 : 순수가상함수로 인터페이스만 제공 (무조건 자식 클래스 override시켜야함 - 상속)
    //! ❓️ 순수 가상 함수는 virtual 멤버함수의원형=0; 인데 왜 =0이 없음? ❓️
    virtual void processMeasurement(const Measurement &measurement);

    //! @brief Sets the most recent control term
    //! 가장 최근의 제어 텀를 설정
    //! @param[in] control - The control term to be applied
    //! @param[in] controlTime - The time at which the control in question was received
    //! 제어(적용할 제어 텀), 제어시간(해당 제어가 수신된 시간) 을 input으로 받음
    //! 👉️ 함수 : setcontorl 함수는 매개변수로 VectorXd &control와 controlTime를 받음
    void setControl(const Eigen::VectorXd &control, const double controlTime);

    //! @brief Sets the control update vector and acceleration limits
    //! 제어 업데이트 벡터 및 가속 제한을 설정
    //! @param[in] updateVector - The values the control term affects
    //! 제어 텀이 영향을 미치는 값
    //! @param[in] controlTimeout - Timeout value, in seconds, after which a control is considered stale
    //! 제어가 오래된 것으로 간주되는 시간 초과 값(초)
    //! @param[in] accelerationLimits - The acceleration limits for the control variables
    //! 제어 변수에 대한 가속 한계
    //! @param[in] accelerationGains - Gains applied to the control term-derived acceleration
    //! 제어 텀을 얻는 가속에 적용되는 게인
    //! @param[in] decelerationLimits - The deceleration limits for the control variables
    //! 제어 변수에 대한 감속 한계
    //! @param[in] decelerationGains - Gains applied to the control term-derived deceleration
    //! 제어 텀을 얻는 감속에 적용되는 게인
    //!  👉️ 함수 : const vector int 및 double 형으로 setControlParams 함수 선언
    void setControlParams(const std::vector<int> &updateVector, const double controlTimeout,
      const std::vector<double> &accelerationLimits, const std::vector<double> &accelerationGains,
      const std::vector<double> &decelerationLimits, const std::vector<double> &decelerationGains);

    //! @brief Sets the filter into debug mode
    //! 필터를 디버그 모드로 설정
    //! NOTE: this will generates a lot of debug output to the provided stream.
    //! The value must be a pointer to a valid ostream object.
    //! 이렇게 하면 제공된 스트림에 대한 많은 디버그 출력이 생성되며 값은 유효한 ostream 개체에 대한 포인터여야함
    //! @param[in] debug - Whether or not to place the filter in debug mode
    //! input : 필터를 디버그 모드로 둘지 여부
    //! @param[in] outStream - If debug is true, then this must have a valid pointer.
    //! input : 디버그가 true이면 유효한 포인터가 있어야 함
    //! If the pointer is invalid, the filter will not enter debug mode. If debug is
    //! false, outStream is ignored.
    //! 포인터가 유효하지 않으면 필터가 디버그 모드로 들어가지 않으며 false면 outstream이 무시
    //!  👉️ 함수 : const bool형인 debug, ostream형태인 outStream포인터를 Null로 초기화한 setdebug 함수 선언
    void setDebug(const bool debug, std::ostream *outStream = NULL);

    //! @brief Enables dynamic process noise covariance calculation
    //! 동적 프로세스 노이즈 공분산 계산 가능
    //! @param[in] dynamicProcessNoiseCovariance - Whether or not to compute dynamic process noise covariance matrices
    //! input : dynamicProcessNoiseCovariance > 동적 프로세스 노이즈 공분산 행렬을 계산할지 여부
    //!  👉️ 함수 : dynamicProcessNoiseCovariance bool여부로 작동하는 setUseDynamicProcessNoiseCovariance함수 선언
    void setUseDynamicProcessNoiseCovariance(const bool dynamicProcessNoiseCovariance);

    //! @brief Manually sets the filter's estimate error covariance
    //! 필터의 추정 오차 공분산을 수동으로 설정
    //! @param[in] estimateErrorCovariance - The state to set as the filter's current state
    //! input : EstimateErrorCovariance - 필터의 현재 상태로 설정할 상태
    //!  👉️ 함수 : const 행렬형태인 estimateErrorCovariance를 선언한 setEstimateErrorCovariance 함수 선언
    void setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance);

    //! @brief Sets the filter's last measurement time.
    //! 필터의 마지막 측정 시간을 설정
    //! @param[in] lastMeasurementTime - The last measurement time of the filter
    //! input : lastMeasurementTime - 필터의 마지막 측정 시간
    //!  👉️ 함수 : const double 형태인 lastMeasurementTime을 setLastMeasurementTime 함수 선언
    void setLastMeasurementTime(const double lastMeasurementTime);

    //! @brief Sets the process noise covariance for the filter.
    //! 필터에 대한 프로세스 노이즈 공분산을 설정
    //! This enables external initialization, which is important, as this
    //! matrix can be difficult to tune for a given implementation.
    //! 외부 초기화를 가능하게하며 🌟️중요함🌟️ 매트릭스는 주어진 구현에 대해 조정하기 어려울 수 있음
    //! @param[in] processNoiseCovariance - The STATE_SIZExSTATE_SIZE process noise covariance matrix
    //! to use for the filter
    //! input : processNoiseCovariance > 필터에 사용할 STATE_SIZExSTATE_SIZE 프로세스 노이즈 공분산 행렬
    //!  👉️ 함수 : const 행렬형태인 processNoiseCovariance를 선언한 setProcessNoiseCovariance 함수 선언
    void setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance);

    //! @brief Sets the sensor timeout
    //! 센서 시간 초과를 설정
    //! @param[in] sensorTimeout - The time, in seconds, for a sensor to be
    //! considered having timed out
    //! input : sensorTimeout > 센서가 시간 초과된 것으로 간주되는 시간(초)
    //!  👉️ 함수 : const double 형태로 선언된 sensorTimeout를 선언한 setSensorTimeout 함수 선언
    void setSensorTimeout(const double sensorTimeout);

    //! @brief Manually sets the filter's state
    //! 수동으로 필터의 상태를 설정
    //! @param[in] state - The state to set as the filter's current state
    //! input : state > 필터의 현재 상태로 설정할 상태
    //!  👉️ 함수 : const 행렬형태인 state를 선언한 setState 함수 선언
    void setState(const Eigen::VectorXd &state);

    //! @brief Ensures a given time delta is valid (helps with bag file playback issues)
    //! 주어진 시간 델타가 유효한지 확인(백 파일 재생 문제에 도움)
    //! @param[in] delta - The time delta, in seconds, to validate
    //! input : delta > 검증할 시간 델타(초)
    //!  👉️ 함수 : double형 참조자 delta를 선언한 validateDelta 함수
    void validateDelta(double &delta);
  
  // 상속관계일 때 접근이 가능
  protected:
    //! @brief Method for settings bounds on acceleration values derived from controls
    //! 컨트롤에서 파생된 가속 값의 설정 범위를 위한 메서드
    //! @param[in] state - The current state variable (e.g., linear X velocity)
    //! 현재 상태 변수(예: 선형 X 속도)
    //! @param[in] control - The current control commanded velocity corresponding to the state variable
    //! 상태 변수에 해당하는 현재 제어 명령 속도
    //! @param[in] accelerationLimit - Limit for acceleration (regardless of driving direction)
    //! 가속도 제한(주행 방향에 관계없이)
    //! @param[in] accelerationGain - Gain applied to acceleration control error
    //! 가속 제어 오류에 적용되는 게인
    //! @param[in] decelerationLimit - Limit for deceleration (moving towards zero, regardless of driving direction)
    //! 감속 한계(주행 방향에 관계없이 0을 향해 이동)
    //! @param[in] decelerationGain - Gain applied to deceleration control error
    //! 감속 제어 오류에 적용되는 게인
    //! @return a usable acceleration estimate for the control vector
    //! return 값 : 제어 벡터에 대한 사용 가능한 가속 추정값


    //! 👉️ 함수 : inline함수로 정의한 코드들이 인라인 함수 호출 시 그 자리에 인라인 함수 코드 자체가 안으로 들어감
    //! 👉️ 함수 : 여기서는 computeControlAcceleration 함수 호출하면 여기서 작성한 계산들이 다 적용되어 main문에 들어감
    inline double computeControlAcceleration(const double state, const double control, const double accelerationLimit,
      const double accelerationGain, const double decelerationLimit, const double decelerationGain)
    {
      // #define FB_DEBUG(msg) if (getDebug()) { *debugStream_ << msg; } 이며 getdebug는 bool형
      // 즉, getdebug가 true면 FB_DEBUG는 msg 출력
      FB_DEBUG("---------- FilterBase::computeControlAcceleration ----------\n");

      // error는 제어 - 상태
      const double error = control - state;
      // samesign true? false? : error의 절대값(double형) <=(관계 연산자) control의 절대값(double형) + 0.01
      const bool sameSign = (::fabs(error) <= ::fabs(control) + 0.01);
      // setpoint = samesign이 참이면 control로 받고 거짓이면 0.0으로 받음
      const double setPoint = (sameSign ? control : 0.0);
      // decelerating true? false? : setpoint의 절대값(double형) <= state의 절대값(double형)
      const bool decelerating = ::fabs(setPoint) < ::fabs(state);
      // limit = accelerationLimit
      double limit = accelerationLimit;
      // gain = accelerationGain
      double gain = accelerationGain;

      // 만약 감속한다면? limit은 decelerationLimit, gain은 decelerationGain
      if (decelerating)
      { 
        limit = decelerationLimit;
        gain = decelerationGain;
      }

      // finalaccel = 게인*error, -limit의 max값과 limit의 min값
      const double finalAccel = std::min(std::max(gain * error, -limit), limit);

      FB_DEBUG("Control value: " << control << "\n" <<
               "State value: " << state << "\n" <<
               "Error: " << error << "\n" <<
               "Same sign: " << (sameSign ? "true" : "false") << "\n" <<
               "Set point: " << setPoint << "\n" <<
               "Decelerating: " << (decelerating ? "true" : "false") << "\n" <<
               "Limit: " << limit << "\n" <<
               "Gain: " << gain << "\n" <<
               "Final is " << finalAccel << "\n");

      return finalAccel;
    }

    //! @brief Keeps the state Euler angles in the range [-pi, pi]
    //! 상태 오일러 각도를 [-pi, pi] 범위로 유지
    //! 👉️ 함수 : 가상함수 wrapStateAngles
    virtual void wrapStateAngles();

    //! @brief Tests if innovation is within N-sigmas of covariance. Returns true if passed the test.
    //! innovation이 공분산의 N-시그마 내에 있는지 테스트하며 테스트를 통과하면 true를 반환
    //! @param[in] innovation - The difference between the measurement and the state
    //! input : innovation > 측정과 상태의 차이
    //! @param[in] invCovariance - The innovation error
    //! input : invCovariance > innovation 에러
    //! @param[in] nsigmas - Number of standard deviations that are considered acceptable
    //! input : nsigmas > 수용 가능한 것으로 간주되는 표준 편차의 수
    //! 👉️ 함수 : 가상 bool 함수
    virtual bool checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                                           const Eigen::MatrixXd &invCovariance,
                                           const double nsigmas);

    //! @brief Converts the control term to an acceleration to be applied in the prediction step
    //! 제어 텀을 예측 단계에 적용할 가속도로 변환
    //! @param[in] referenceTime - The time of the update (measurement used in the prediction step)
    //! input : referenceTime > 업데이트 시간(예측 단계에서 사용된 측정)
    //! @param[in] predictionDelta - The amount of time over which we are carrying out our prediction
    //! input : predictionDelta > 예측을 수행하는 데 걸리는 시간
    //! 👉️ 함수 : const double형 파라미터 받는 prepareControl 함수 선언
    void prepareControl(const double referenceTime, const double predictionDelta);

    //! @brief Whether or not we've received any measurements
    //! 측정을 받았는지 여부
    bool initialized_;

    //! @brief Whether or not we apply the control term
    //! 제어 텀을 적용할지 여부
    bool useControl_;

    //! @brief If true, uses the robot's vehicle state and the static process noise covariance matrix to generate a
    //! dynamic process noise covariance matrix
    //! true인 경우 로봇의 차량 상태와 정적 프로세스 노이즈 공분산 행렬을 사용하여 동적 프로세스 노이즈 공분산 행렬을 생성
    bool useDynamicProcessNoiseCovariance_;

    //! @brief Tracks the time the filter was last updated using a measurement.
    //! 측정을 사용하여 필터가 마지막으로 업데이트된 시간을 추적
    //! This value is used to monitor sensor readings with respect to the sensorTimeout_.
    //! We also use it to compute the time delta values for our prediction step.
    //! 이 값은 sensorTimeout_과 관련하여 센서 판독값을 모니터링하는 데 사용
    //! 또한 이를 사용하여 예측 단계의 시간 델타 값을 계산
    double lastMeasurementTime_;

    //! @brief The time of reception of the most recent control term
    //! 가장 최근의 제어 텀을 수신한 시간
    double latestControlTime_;

    //! @brief Timeout value, in seconds, after which a control is considered stale
    //! 시간 초과 값(초), 이후 컨트롤은 이상한 것으로 간주
    double controlTimeout_;

    //! @brief The updates to the filter - both predict and correct - are driven
    //! by measurements. If we get a gap in measurements for some reason, we want
    //! the filter to continue estimating. When this gap occurs, as specified by
    //! this timeout, we will continue to call predict() at the filter's frequency.
    //! 필터에 대한 업데이트(예측 및 수정 모두)는 측정에 의해 구동
    //! 어떤 이유로 측정값에 차이가 생기면 필터가 계속해서 추정하기를 원함
    //! 이 타임아웃에 명시된 대로 측정값 차이의 간격이 발생하면 필터의 빈도에서 계속해서 predict()를 호출
    double sensorTimeout_;

    //! @brief Which control variables are being used (e.g., not every vehicle is controllable in Y or Z)
    //! 사용 중인 제어 변수(예: 모든 차량이 Y 또는 Z로 제어할 수 있는 것은 아님)
    std::vector<int> controlUpdateVector_;

    //! @brief Gains applied to acceleration derived from control term
    //! 제어 텀에서 파생된 가속에 적용되는 게인
    std::vector<double> accelerationGains_;

    //! @brief Caps the acceleration we apply from control input
    //! 제어 입력에서 적용한 가속을 제한
    std::vector<double> accelerationLimits_;

    //! @brief Gains applied to deceleration derived from control term
    //! 제어 텀에서 파생된 감속에 적용되는 게인
    std::vector<double> decelerationGains_;

    //! @brief Caps the deceleration we apply from control input
    //! 제어 입력에서 적용한 감속을 제한
    std::vector<double> decelerationLimits_;

    //! @brief Variable that gets updated every time we process a measurement and we have a valid control
    //! 측정을 처리하고 유효한 컨트롤을 가질 때마다 업데이트되는 변수
    Eigen::VectorXd controlAcceleration_;

    //! @brief Latest control term
    //! 최신 제어 텀
    Eigen::VectorXd latestControl_;

    //! @brief Holds the last predicted state of the filter
    //! 필터의 마지막 예측 상태를 유지
    Eigen::VectorXd predictedState_;

    //! @brief This is the robot's state vector, which is what we are trying to
    //! filter. The values in this vector are what get reported by the node.
    //! 🌟️🌟️ 이것은 우리가 하려고 하는 로봇의 상태 벡터로 이 벡터의 값은 노드에서 보고하는 값 🌟️🌟️
    Eigen::VectorXd state_;

    //! @brief Covariance matrices can be incredibly unstable. We can
    //! add a small value to it at each iteration to help maintain its
    //! positive-definite property.
    //! 공분산 행렬은 엄청나게 불안정할 수 있기에, 양의 정부호 속성을 유지하는 데 도움이 되도록 각 반복마다 작은 값을 추가
    Eigen::MatrixXd covarianceEpsilon_;

    //! @brief Gets updated when useDynamicProcessNoise_ is true
    //! useDynamicProcessNoise_가 true일 때 업데이트
    Eigen::MatrixXd dynamicProcessNoiseCovariance_;

    //! @brief This matrix stores the total error in our position
    //! estimate (the state_ variable).
    //! 이 행렬은 위치 추정치(state_ variable)에 총 오차를 저장
    Eigen::MatrixXd estimateErrorCovariance_;

    //! @brief We need the identity for a few operations. Better to store it.
    //! 몇 가지 작업에 대한 ID가 필요함
    Eigen::MatrixXd identity_;

    //! @brief As we move through the world, we follow a predict/update
    //! cycle. If one were to imagine a scenario where all we did was make
    //! predictions without correcting, the error in our position estimate
    //! would grow without bound. This error is stored in the
    //! stateEstimateCovariance_ matrix. However, this matrix doesn't answer
    //! the question of *how much* our error should grow for each time step.
    //! That's where the processNoiseCovariance matrix comes in. When we
    //! make a prediction using the transfer function, we add this matrix
    //! (times deltaT) to the state estimate covariance matrix.
    //! 세계를 이동하면서 예측/업데이트 주기를 따름
    //! 우리가 수정하지 않고 예측만 하는 시나리오를 상상한다면 위치 추정의 오류는 무한히 커짐
    //! 이 오류는 stateEstimateCovariance_ 행렬에 저장
    //! 그러나 이 행렬은 각 시간 단계에 대해 우리의 오류가 *얼마나* 증가해야 하는지에 대한 질문에 답하지 않음
    //! 바로 여기에서 processNoiseCovariance 행렬이 나옴
    //! 전달 함수를 사용하여 예측할 때 이 행렬(deltaT 곱하기)을 상태 추정 공분산 행렬에 추가
    Eigen::MatrixXd processNoiseCovariance_;

    //! @brief The Kalman filter transfer function
    //!
    //! Kalman filters and extended Kalman filters project the current
    //! state forward in time. This is the "predict" part of the predict/correct
    //! cycle. A Kalman filter has a (typically constant) matrix A that defines
    //! how to turn the current state, x, into the predicted next state. For an
    //! EKF, this matrix becomes a function f(x). However, this function can still
    //! be expressed as a matrix to make the math a little cleaner, which is what
    //! we do here. Technically, each row in the matrix is actually a function.
    //! Some rows will contain many trigonometric functions, which are of course
    //! non-linear. In any case, you can think of this as the 'A' matrix in the
    //! Kalman filter formulation.

    //! 칼만 필터 전달 함수
    //! 칼만 필터와 확장 칼만 필터는 현재 상태를 시간적으로 앞으로 투영
    //! 이것은 예측/정확한 주기의 "예측" 부분
    //! 칼만 필터에는 현재 상태 x를 예측된 다음 상태로 바꾸는 방법을 정의하는 (일반적으로 상수) 행렬 A가 있음
    //! EKF의 경우 이 행렬은 함수 f(x)가 됨 (비선형)
    //! 그러나 이 함수는 수학을 좀 더 명확하게 하기 위해 여전히 행렬로 표현될 수 있음
    //! 기술적으로 행렬의 각 행은 실제로 함수
    //! 일부 행에는 물론 비선형인 많은 삼각 함수가 포함
    //! 어쨌든 이것을 칼만 필터 공식에서 'A' 행렬로 생각할 수 있음
    Eigen::MatrixXd transferFunction_;

    //! @brief The Kalman filter transfer function Jacobian
    //!
    //! The transfer function is allowed to be non-linear in an EKF, but
    //! for propagating (predicting) the covariance matrix, we need to linearize
    //! it about the current mean (i.e., state). This is done via a Jacobian,
    //! which calculates partial derivatives of each row of the transfer function
    //! matrix with respect to each state variable.

    //! 칼만 필터 전달 함수 Jacobian
    //! 전달 함수는 EKF에서 비선형으로 허용되지만 공분산 행렬을 전파(예측)하려면 현재 평균(즉, 상태)에 대해 선형화
    //! 이것은 각 상태 변수에 대해 전달 함수 행렬의 각 행의 편미분을 계산하는 야코비안 행렬을 통해 수행
    Eigen::MatrixXd transferFunctionJacobian_;

    //! @brief Used for outputting debug messages
    //! 디버그 메시지 출력에 사용
    std::ostream *debugStream_;

  private:
    //! @brief Whether or not the filter is in debug mode
    //! 필터가 디버그 모드인지 여부
    bool debug_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_FILTER_BASE_H