#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

float current_velocity = 0.0;
float target_velocity = 0.0;
unsigned long prev_time = 0;  // Move prev_time out of the loop

class PID {
private:
    float Kp;
    float Ki;
    float Kd;
    float prevError;
    float integral;

public:
    PID(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd), prevError(0.0), integral(0.0) {}

    float calculate(float target, float current, float dt) {
        float dt = 0.01 ;
        float error = target - current;
        float P = Kp * error;
        integral += error * dt;
        float I = Ki * integral;

        float derivative = (error - prevError) / dt;
        float D = Kd * derivative;

        prevError = error;

        return P + I + D;
    }

    void setParameters(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
};

class PID_ROS : public PID {
private:
    ros::Subscriber<std_msgs::Float32> current_velocity_sub;
    ros::Subscriber<std_msgs::Float32> target_velocity_sub;
    ros::Subscriber<std_msgs::Float32MultiArray> pid_tuner_sub;
    ros::Publisher cmd_vel_pub;

    geometry_msgs::Twist cmd_vel_msg;

public:
    PID_ROS(float kp, float ki, float kd)
        : PID(kp, ki, kd),
          current_velocity_sub("/current_velocity", &PID_ROS::current_velocity_callback, this),
          target_velocity_sub("/setpoint", &PID_ROS::target_velocity_callback, this),
          pid_tuner_sub("/pid_parameters", &PID_ROS::pid_parameters_callback, this),
          cmd_vel_pub("cmd_vel", &cmd_vel_msg) {}

    void current_velocity_callback(const std_msgs::Float32& msg) {
        current_velocity = msg.data;
    }

    void target_velocity_callback(const std_msgs::Float32& msg) {
        target_velocity = msg.data;
    }

    void pid_parameters_callback(const std_msgs::Float32MultiArray& msg) {
        if (msg.data.size() >= 3) {
            float kp = msg.data[0];
            float ki = msg.data[1];
            float kd = msg.data[2];
            setParameters(kp, ki, kd);
        }
    }

    void pid_ros_initializer() {
        nh.initNode();
        nh.advertise(cmd_vel_pub);
        nh.subscribe(current_velocity_sub);
        nh.subscribe(target_velocity_sub);
        nh.subscribe(pid_tuner_sub);
    }

    void pid_ros_loop() {
        nh.spinOnce();  // Process ROS messages

        unsigned long current_time = millis();
        float dt = (current_time - prev_time) / 1000.0;  // Calculate dt in seconds

        if (dt >= 0.01) {  // Ensure dt is at least 10 ms (to avoid too fast processing)
            float pid_output = calculate(target_velocity, current_velocity, dt);

            // Update cmd_vel message with the PID output
            cmd_vel_msg.linear.x = pid_output;

            // Publish the command velocity
            cmd_vel_pub.publish(&cmd_vel_msg);

            prev_time = current_time;  // Update prev_time to the current time
        }
    }
};

// Create global instance of PID_ROS
PID_ROS pid_ros(0.0, 0.0, 0.0);  // Initializing with 0 for Kp, Ki, Kd

void setup() {
    pid_ros.pid_ros_initializer();  // Initialize ROS communication
}

void loop() {
    pid_ros.pid_ros_loop();  // Execute control loop
}
