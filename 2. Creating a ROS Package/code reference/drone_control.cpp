// 단지 drone control만을 위한 cpp 이며 drone_control.h 만 들고옴
// 메인 헤더파일은 필요 X
#include "drone_control/drone_control.h"

// DroneControl 클래스 구현부
// DroneControl 클래스의 생성자 구현부
DroneControl::DroneControl()
{    
}

// DroneControl 클래스의 Init 함수 구현 ... 이하 동일
void DroneControl::Init()
{
    stop = false; // stop이 false라면
    // publisher선언시 nodehandle 클래스인 advertise이용
    set_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("mavros/setpoint_position/local", 10);
    set_global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>
        ("mavros/setpoint_position/global", 10);
    set_vel_pub = nh.advertise<geometry_msgs::Twist>
        ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // subscribe선언시 nodehandle 클래스인 subscriber이용 (콜백함수 꼬옥 넣어줘야함)    
    // &DroneControl::PositionStateCallback this 때매 &. &이하의 주소가 this 매개 변수에 전달 
    get_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, &DroneControl::PositionStateCallback, this);

    // 서비스 클라이언트 객체 생성, 서비스 데이터 유형과 이름 (정의는 drone_control.h)
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
        ("/mavros/cmd/takeoff");
    return;
}

void DroneControl::Run()
{
    thread = std::thread(&DroneControl::RosSpin, this); // &이하의 주소가 this 매개 변수에 전달 
    return;
}

void DroneControl::Finish()
{
    stop = true;
    thread.join(); // 해당하는 쓰레드들이 실행을 종료하면 리턴
    return;
}

void DroneControl::setArm(const bool arm)
{
    mavros_msgs::CommandBool arm_cmd; // 데이터 유형 mavros_msgs::CommandBool의 이름 arm_cmd로 지정
    arm_cmd.request.value = arm; // 지정한 arm_cmd의 value값을 'arm'으로 request
    arming_client.call(arm_cmd); // serviceclient arming_clinet 를 (mavros msg bool 형식으로) 콜

    return;
}

void DroneControl::setMode(const std::string mode)
{
    mavros_msgs::SetMode set_mode_msg;
    set_mode_msg.request.custom_mode = mode;
    set_mode_client.call(set_mode_msg);

    return;
}

void DroneControl::setTakeoff(const float alt)
{
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.latitude = 0;
    takeoff_cmd.request.longitude = 0;
    takeoff_cmd.request.altitude = alt;
    takeoff_cmd.request.min_pitch = 0;
    takeoff_cmd.request.yaw = 0;
    takeoff_client.call(takeoff_cmd);

    return;
}

void DroneControl::setLocalPosePub(const float x, const float y, const float z)
{
    geometry_msgs::PoseStamped dest;
    dest.pose.position.x = x;
    dest.pose.position.y = y;
    dest.pose.position.z = z;
    set_local_pos_pub.publish(dest); //pub해줌
    return;
}

void DroneControl::setVelocityPub(const float x, const float y, const float z)
{
    geometry_msgs::Twist vel;
    vel.linear.x = x;
    vel.linear.y = y;
    vel.linear.z = z;
    set_vel_pub.publish(vel);
}

std::vector<float> DroneControl::getCurrnetPosition()
{
    std::vector<float> pose;
    pose.resize(3); // vector 크기를 3으로 재할당
    pose[0] = getpos.pose.position.x;
    pose[1] = getpos.pose.position.y;
    pose[2] = getpos.pose.position.z;
    return pose;
}
std::vector<float> DroneControl::getCurrnetOri()
{
    std::vector<float> ori;
    ori.resize(4);
    ori[0] = getpos.pose.orientation.x;
    ori[1] = getpos.pose.orientation.y;
    ori[2] = getpos.pose.orientation.z;
    ori[3] = getpos.pose.orientation.w;
    return ori;
}

void DroneControl::PositionStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    getpos = *msg; // msg를 받아옴
}

void DroneControl::RosSpin()
{
    ros::Rate r(30);
    while(!stop)
    {
        ros::spinOnce();
        r.sleep();
    }
    return;
}
