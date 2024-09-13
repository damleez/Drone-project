//#include <ros/ros.h> drone_control.h 에 선언 되어있어서 안적어도 됨
//#include <math.h> 애초에 circle을 없애서 필요 X

// 헤더 파일 들고오기
#include "drone_control/util.h"
#include "drone_control/drone_control.h"

// 키 설정 정의
#define SPACEBAR 32
#define T 116
#define W 119
#define A 97
#define S 115
#define D 100
#define R 114
#define C 99
#define ENTER 10
#define Q 113
#define E 101
#define F 102
#define G 103

using namespace std;

// 메인 함수
int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_control"); // 세번째 인수는 노드의 이름

    DroneControl *drone_control = new DroneControl(); // 클래스 타입의 메모리 동적 할당

    drone_control->Init(); // ->은 포인터를 통해 멤버를 접근
    drone_control->Run(); // 포인터변수 drone_control이 멤버 함수 Run에 접근

    bool stop = false; // stop이 false라면
    while(!stop) // stop 아닐동안 계속해서 돌려라
    {
        int key = key_input(); // key의 자료형은 int고 util.h에 있는 key_input이라는 전역함수를 받음
        switch(key){ // switch-case 제어문 : switch의 레이블은 'key'
            case SPACEBAR: // 만약 key가 spacebar이라면
            {
                ROS_INFO("'Spacebar'key pressed. \nSet mode to 'guided' and arm the drone.\n"); // =cout
                // drone_control.h에서 void SetMode(const std::string mode) 이기 때문에 mode가 string으로 받을 수 있음
                drone_control->setMode("GUIDED"); // 포인터변수 drone_control이 setMode GUIDED에 접근
                
                drone_control->setArm(true); // 포인터변수 drone_control이 setArm True에 접근
                break; // 멈춰라
            }
            case T: // 만약 key가 T라면
            {
                ROS_INFO("'T'key pressed. \nEnter the height of drone.(only 1 number)\n"); // =cout
                float alt = (key_input())-48; // 0=48, 1=49, ... 9=57
                drone_control->setTakeoff(alt); // 포인터변수 drone_control이 alt에 접근 (alt는 위와 같음)
                break; // 멈춰라
            }
            case E: // 만약 key가 E라면
            {
                float x,y,z; // x,y,z는 float형
                cout << "enter position 'x' 'y' 'z' : " << endl; // 출력
                cin >> x >> y >> z; // 입력 (float형으로)
                cout << "Input position : "<< x << "," << y << "," << z <<endl; // 출력
                drone_control->setLocalPosePub(x,y,z); // 포인터변수 drone_control이 setlocalposepub에 접근 (const라서 x,y,z변수 써야됨)
                break;
            }
            case W:
            {
                drone_control->setVelocityPub(2,0,0); // 포인터 변수가 setvelocitypub에 접근 x=2의 linear 속도로
                break;
            }
            case A:
            {
                drone_control->setVelocityPub(0,2,0);
                break;
            }
            case S:
            {
                drone_control->setVelocityPub(-2,0,0);
                break;
            }
            case D:
            {
                drone_control->setVelocityPub(0,-2,0);
                break;
            }
            case Q:
            {
                drone_control->setMode("BRAKE");
                break;
            }
            case R:
            {   
                drone_control->setMode("RTL");
                break;
            }
            case ENTER:
            {
                ROS_INFO("'Enter key' pressed. \n");
                break;
            }
            case F: 
            {
                drone_control->Finish(); // 포인터 변수 drone_control이 Finish 함수에 접근했다면
                stop = true; // stop은 true
                break; // 멈춰라
            }
            default:  // 키 입력이 정해진 것 외에는
                break; // 멈춰라
        }
    }

    delete drone_control; // 동적 할당 반환

    return 0;

}
