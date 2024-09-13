//전처리문 (헤더파일 꼬임 방지)
#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

//ros 헤더 파일 선언
#include <ros/ros.h>

// 수학 헤더 파일 선언
#include <math.h>

//mavros 헤더 파일 선언
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <thread>

//DroneControl 클래스 선언부
class DroneControl
{

    public:
        //생성자로 객체가 생성되는 시점에 자동으로 호출되는 멤버 함수
        DroneControl(); //왜 사용? 객체의 초기화 작업들을 담당하기 위함

        // 클래스 안 멤버 함수 선언
        void Init();
        void Run();
        void Finish();

        // 값이 변경 안되길 원하기 때문에 const를 사용
        void setArm(const bool arm);
        void setMode(const std::string mode);
        void setTakeoff(const float alt);
        void setLocalPosePub(const float x, const float y, const float z);    
        void setVelocityPub(const float x, const float y, const float z);

        // 데이터 유형과 이름 선언
        std::vector<float> getCurrnetPosition();
        std::vector<float> getCurrnetOri();


    private:

        void RosSpin();

        // subscriber의 콜백함수
        void PositionStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        // thread함수의 데이터 유형과 이름 선언
        std::thread thread;

        // 노드 핸들 선언
        ros::NodeHandle nh;

        // publisher : set위치, set글로벌 포즈, set속도
        ros::Publisher set_local_pos_pub;
        ros::Publisher set_global_pos_pub;
        ros::Publisher set_vel_pub;

        // subscriber : 포즈
        ros::Subscriber get_local_pos_sub;

        // service : arming, setmode, takeoff (시작시 한 번만 실행하면 되기 때문)
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient takeoff_client;

        // 데이터 유형표시, 이름을 getpos로 지정
        geometry_msgs::PoseStamped getpos;

        // bool 형식 stop 선언
        bool stop;

};

#endif // 전처리문 끝
