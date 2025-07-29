#include <chrono>

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "SensorState.hpp"
#include "SetState.hpp"

#include "OmniDriveSystem.hpp"
#include "DriveLayout.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <iostream>
#include <string>
#include <cstring>


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

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotinoNode::vel_callback, this, _1));
        bumper_publisher_ = this->create_publisher<std_msgs::msg::Bool>("bumper", 10);
        m0v_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m0/speed", 10);
        m1v_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m1/speed", 10);
        m2v_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m2/speed", 10);
        m0vs_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m0/speed_setpoint", 10);
        m1vs_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m1/speed_setpoint", 10);
        m2vs_publisher_ = this->create_publisher<std_msgs::msg::Float32>("m2/speed_setpoint", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        last_wheel_time_ = this->now();
        sendTimer_ = this->create_wall_timer(10ms, std::bind(&RobotinoNode::timer_send_callback, this));
        memset(buffer, 0, 255);
        _SetState.reset();
        motorEN = true;
        pose_local_.theta=0;
        pose_local_.x=0;
        pose_local_.y=0;

        _SetState.kp[0] = this->get_parameter("motor/0/kp").as_int();
        _SetState.ki[0] = this->get_parameter("motor/0/ki").as_int();
        _SetState.kd[0] = this->get_parameter("motor/0/kd").as_int();
        _SetState.kp[1] = this->get_parameter("motor/1/kp").as_int();
        _SetState.ki[1] = this->get_parameter("motor/1/ki").as_int();
        _SetState.kd[1] = this->get_parameter("motor/1/kd").as_int();
        _SetState.kp[2] = this->get_parameter("motor/2/kp").as_int();
        _SetState.ki[2] = this->get_parameter("motor/2/ki").as_int();
        _SetState.kd[2] = this->get_parameter("motor/2/kd").as_int();

        open_serial();
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

    bool open_serial() {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            std::cerr << "Erro ao abrir " << device_ << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "Erro tcgetattr" << std::endl;
            return false;
        }

        // Configuração básica para comunicação serial
        cfsetospeed(&tty, baudrate_);
        cfsetispeed(&tty, baudrate_);

        tty.c_cflag &= ~PARENB;    // Sem paridade
        tty.c_cflag &= ~CSTOPB;    // 1 bit de stop
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;        // 8 bits por byte
        tty.c_cflag &= ~CRTSCTS;   // Sem controle de fluxo hardware
        tty.c_cflag |= CREAD | CLOCAL; // Habilita leitura, ignora linhas de controle

        tty.c_lflag &= ~ICANON;    // Modo não-canônico
        tty.c_lflag &= ~ECHO;      // Desabilita eco
        tty.c_lflag &= ~ECHOE;     // Desabilita erase
        tty.c_lflag &= ~ECHONL;    // Desabilita new-line echo
        tty.c_lflag &= ~ISIG;      // Desabilita interpretação de sinais

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Desabilita controle de fluxo software
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Raw input

        tty.c_oflag &= ~OPOST;     // Saída raw
        tty.c_oflag &= ~ONLCR;     // Não converte newline para CRNL

        // Configura timeout de leitura: bloquear até receber pelo menos 1 caractere
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 5;       // Timeout de 0.5 segundos (em décimos de segundo)

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Erro tcsetattr" << std::endl;
            return false;
        }

        return true;
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

        serial.readBytes(recv, 101U, 10U, 0U); //robotino takes from 4 to 6 miliseconds to answer

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

            // odometry kinematics

            double m1 = _SensorState.actualVelocity[0];
            double m2 = _SensorState.actualVelocity[1];
            double m3 = _SensorState.actualVelocity[2];

            

            float vx, vy, omega;
            _omni.unprojectVelocity(&vx, &vy, &omega, m1, m2, m3, 0);

            rclcpp::Time msg_time = this->now();

            float dt = (msg_time - last_wheel_time_).seconds();

            last_wheel_time_ = msg_time;

            integrateByRungeKutta(vx, vy, omega, dt);

            nav_msgs::msg::Odometry odom_msg;
            geometry_msgs::msg::TransformStamped transform_stamped;
            //odom_msg.header.stamp = this->get_clock()->now();
            odom_msg.header.stamp = msg_time;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            transform_stamped.header.stamp = msg_time;
            transform_stamped.header.frame_id = "odom";        // Frame pai
            transform_stamped.child_frame_id = "base_link";    // Frame filho

            transform_stamped.transform.translation.x = pose_local_.x;  // Exemplo de deslocamento em X
            transform_stamped.transform.translation.y = pose_local_.y;  // Exemplo de deslocamento em Y
            transform_stamped.transform.translation.z = 0.0;  // Exemplo de deslocamento em Z


            // Preenche a pose na mensagem de odometria
            odom_msg.pose.pose.position.x = pose_local_.x;
            odom_msg.pose.pose.position.y = pose_local_.y;

            tf2::Quaternion q;
            q.setRPY(0, 0, pose_local_.theta);
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            transform_stamped.transform.rotation.x = q.x();
            transform_stamped.transform.rotation.y = q.y();
            transform_stamped.transform.rotation.z = q.z();
            transform_stamped.transform.rotation.w = q.w();

            // Preenche a velocidade na mensagem de odometria
            odom_msg.twist.twist.linear.x = vx;
            odom_msg.twist.twist.linear.y = vy;
            odom_msg.twist.twist.angular.z = omega;

            // Exemplo simples com variância/erro arbitrário
            for (int i = 0; i < 36; ++i) {
                odom_msg.pose.covariance[i] = 0.0;  // Inicializa com zeros
            }

            // Definindo variâncias e covariâncias
            odom_msg.pose.covariance[0] = 0.01;  // Variância em x
            odom_msg.pose.covariance[7] = 0.01;  // Variância em y
            odom_msg.pose.covariance[14] = 0.01; // Variância em θ

            // Covariâncias entre as variáveis
            odom_msg.pose.covariance[1] = 0.001;  // Covariância entre x e y
            odom_msg.pose.covariance[6] = 0.001;  // Covariância entre x e θ
            odom_msg.pose.covariance[13] = 0.001; // Covariância entre y e θ

            // Covariância entre as velocidades (simples exemplo)
            odom_msg.twist.covariance[0] = 0.1;  // Variância em vx
            odom_msg.twist.covariance[7] = 0.1;  // Variância em vy
            odom_msg.twist.covariance[14] = 0.1; // Variância em ω

            // Publica a mensagem de odometria
            odom_publisher_->publish(odom_msg);
            tf_broadcaster_->sendTransform(transform_stamped);

        }else{
            RCLCPP_WARN(get_logger(), "unable to decode");
            serial.flushReceiver();
        }
    }

    void integrateByRungeKutta(float vx, float vy, float wz, float dt_) {
        double theta_bar = pose_local_.theta + (wz*dt_ / 2.0f);
        pose_local_.x = pose_local_.x + (vx * cos(theta_bar) - vy * sin(theta_bar)) * dt_;
        pose_local_.y = pose_local_.y + (vx * sin(theta_bar) + vy * cos(theta_bar)) * dt_;
        pose_local_.theta += wz*dt_;
    }

    struct LocPose {
        double x, y, theta;
    } pose_local_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Time last_wheel_time_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr sendTimer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m0v_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m1v_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m2v_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m0vs_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m1vs_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m2vs_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    unsigned short status = 0;
    bool motorEN = false;
    SetState _SetState;
    SensorState _SensorState;
    unsigned char buffer[255];
    unsigned char recv[110];
    OmniDriveSystem _omni;
    const float maxRPM[4] = {10000000.0f, 10000000.0f, 10000000.0f};
    std::string device_;
    int baudrate_;
    int fd_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotinoNode>());
    rclcpp::shutdown();
    return 0;
}