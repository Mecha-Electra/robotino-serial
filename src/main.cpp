#include <chrono>
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <serialib/serialib.h>
#include "SensorState.hpp"
#include "SetState.hpp"

#include "OmniDriveSystem.hpp"
#include "DriveLayout.hpp"

using namespace std::chrono_literals;

class RobotinoNode : public rclcpp::Node
{
public:
    RobotinoNode()
        : Node("robotino_driver"),
        _omni(DriveLayout())
    {
        this->declare_parameter("motor/0/kp", 0);
        this->declare_parameter("motor/0/ki", 0);
        this->declare_parameter("motor/0/kd", 0);
        this->declare_parameter("motor/1/kp", 0);
        this->declare_parameter("motor/1/ki", 0);
        this->declare_parameter("motor/1/kd", 0);
        this->declare_parameter("motor/2/kp", 0);
        this->declare_parameter("motor/2/ki", 0);
        this->declare_parameter("motor/2/kd", 0);
        this->declare_parameter("motor/max_rpm", 0);

        serial.openDevice("/dev/ttyUSB0", 115200);
        if(serial.isDeviceOpen()){
            RCLCPP_INFO(get_logger(), "device opened");
        }
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotinoNode::vel_callback, this, _1));
        bumper_publisher_ = this->create_publisher<std_msgs::msg::Bool>("bumper", 10);
        m0v_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m0/speed", 10);
        m1v_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m1/speed", 10);
        m2v_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m2/speed", 10);
        m0vs_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m0/speed_setpoint", 10);
        m1vs_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m1/speed_setpoint", 10);
        m2vs_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m2/speed_setpoint", 10);
        sendTimer_ = this->create_wall_timer(10ms, std::bind(&RobotinoNode::timer_send_callback, this));
        receiveTimer_ = this->create_wall_timer(5ms, std::bind(&RobotinoNode::timer_receive_callback, this));
        memset(buffer, 0, 255);
        _SetState.reset();
        motorEN = true;

        _SetState.kp[0] = this->get_parameter("motor/0/kp").as_int();
        _SetState.ki[0] = this->get_parameter("motor/0/ki").as_int();
        _SetState.kd[0] = this->get_parameter("motor/0/kd").as_int();
        _SetState.kp[1] = this->get_parameter("motor/1/kp").as_int();
        _SetState.ki[1] = this->get_parameter("motor/1/ki").as_int();
        _SetState.kd[1] = this->get_parameter("motor/1/kd").as_int();
        _SetState.kp[2] = this->get_parameter("motor/2/kp").as_int();
        _SetState.ki[2] = this->get_parameter("motor/2/ki").as_int();
        _SetState.kd[2] = this->get_parameter("motor/2/kd").as_int();
    }

private:
    void vel_callback(const geometry_msgs::msg::Twist msg)
    {
        if(motorEN){
            _omni.projectVelocity(_SetState.speedSetPoint, maxRPM, 
                msg.linear.x, msg.linear.y, msg.angular.z);
            //RCLCPP_INFO(get_logger(), "wheelVels: %f %f %f", _SetState.speedSetPoint[0], _SetState.speedSetPoint[1], _SetState.speedSetPoint[2]);
        }
    }

    void timer_send_callback(){
        status++;
        if(!motorEN){
            _SetState.speedSetPoint[0] = 0.0f;
            _SetState.speedSetPoint[1] = 0.0f;
            _SetState.speedSetPoint[2] = 0.0f;
            _SetState.dOut[0] = status & 128;
        }else{
            _SetState.dOut[0] = status & 32;
            //RCLCPP_INFO(get_logger(), "sending: %f %f %f", _SetState.speedSetPoint[0], _SetState.speedSetPoint[1], _SetState.speedSetPoint[2]);
        }
        std_msgs::msg::Float32 m0_vs = std_msgs::msg::Float32();
        std_msgs::msg::Float32 m1_vs = std_msgs::msg::Float32();
        std_msgs::msg::Float32 m2_vs = std_msgs::msg::Float32();
        m0_vs.data = _SetState.speedSetPoint[0];
        m1_vs.data = _SetState.speedSetPoint[1];
        m2_vs.data = _SetState.speedSetPoint[2];
        m0vs_publisher_->publish(m0_vs);
        m1vs_publisher_->publish(m1_vs);
        m2vs_publisher_->publish(m2_vs);
        _SetState.toQDSAProtocol(buffer);
        serial.writeBytes(buffer, 47);
    }

    void timer_receive_callback(){
        memset(recv, 0, 110);
        if(serial.available() > 100){
            serial.readBytes(recv, 101U, 10U); //robotino takes from 4 to 6 miliseconds to answer
            //RCLCPP_INFO(get_logger(), "%d", serial.available());
            if(_SensorState.fromQDSAProtocol(recv)){
                if(_SensorState.bumper){
                    motorEN = false;
                }
                std_msgs::msg::Bool message = std_msgs::msg::Bool();
                message.data = _SensorState.bumper;
                bumper_publisher_->publish(message);

                m0v_publisher_->publish(std_msgs::msg::Float32().set__data(
                    _SensorState.actualVelocity[0]
                ));
                m1v_publisher_->publish(std_msgs::msg::Float32().set__data(
                    _SensorState.actualVelocity[1]
                ));
                m2v_publisher_->publish(std_msgs::msg::Float32().set__data(
                    _SensorState.actualVelocity[2]
                ));
            }else{
                RCLCPP_WARN(get_logger(), "unable to decode");
                serial.flushReceiver();
            }
        }
    }

    rclcpp::TimerBase::SharedPtr sendTimer_;
    rclcpp::TimerBase::SharedPtr receiveTimer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m0v_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m1v_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m2v_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m0vs_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m1vs_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m2vs_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    serialib serial;
    unsigned short status = 0;
    bool motorEN = false;
    SetState _SetState;
    SensorState _SensorState;
    unsigned char buffer[255];
    unsigned char recv[110];
    OmniDriveSystem _omni;
    const float maxRPM[4] = {10000000.0f, 10000000.0f, 10000000.0f};
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotinoNode>());
    //serial.closeDevice();
    rclcpp::shutdown();
    return 0;

    /*
    switch (serial.openDevice("/dev/ttyUSB0", 115200))
    {
    case 1:
        printf("porta conectada");
        break;
    default:
        break;
    }
    int i = 0;
    bool disarm = false;
    while (true){
        unsigned char buffer[255];
        memset(buffer, 0, 255);
        _SetState.reset();

        _SetState.dOut[0] = false;
        _SetState.dOut[1] = false;
        _SetState.dOut[2] = i & (disarm ? 16 : 32);
        _SetState.dOut[3] = disarm;

        _SetState.brake[0] = disarm;
        _SetState.kp[0] = 60;
        _SetState.ki[0] = 20;
        _SetState.kd[0] = 20;
        //maximo teorico 13e12
        if(!disarm)
        _SetState.speedSetPoint[0] = 13000000.0f;
        else{
            _SetState.speedSetPoint[0] = 0.0f;
        }

        _SetState.toQDSAProtocol(buffer);
        serial.writeBytes(buffer, 47);

        while(serial.available() < 100){
        }
        unsigned char recv[110];
        memset(recv, 0, 110);
        serial.readBytes(recv, serial.available());

        _SensorState.fromQDSAProtocol(recv);

        printf("=============\n");
        printf("bumper:  %d\n", _SensorState.bumper);
        printf("current: %f\n", _SensorState.current);
        printf("M0 pos: %6d Vel: %10.1f current: %1.3f\n", _SensorState.actualPosition[0], _SensorState.actualVelocity[0], _SensorState.motorCurrent[0]);
        printf("M1 pos: %6d Vel: %10.1f current: %1.3f\n", _SensorState.actualPosition[1], _SensorState.actualVelocity[1], _SensorState.motorCurrent[1]);
        printf("M2 pos: %6d Vel: %10.1f current: %1.3f\n", _SensorState.actualPosition[2], _SensorState.actualVelocity[2], _SensorState.motorCurrent[2]);
        printf("M3 pos: %6d Vel: %10.1f current: %1.3f\n", _SensorState.actualPosition[3], _SensorState.actualVelocity[3], _SensorState.motorCurrent[3]);

        if(_SensorState.bumper || _SensorState.isPowerButtonPressed){
            disarm = true;
        }
        i++;
        if(i > 127) i = 0;
    }
    */
}