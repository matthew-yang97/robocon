#include "mbed.h"
#include "ros.h"
#include "auto_bot/chassic_speed.h"
#include "std_msgs/Float32.h"

//ROS node setup----------------------------------------------------------------------------------
ros::NodeHandle nh;

//ROS Publsiher-----------------------------------------------------------------------------------
auto_bot::chassic_speed outgo_motor_message;
ros::Publisher motor_pub("Auto_Bot_Chassic_Speed", &outgo_motor_message);

//ROS Subscriber----------------------------------------------------------------------------------
ros::Subscriber<auto_bot::chassic_speed> sub("Auto_Bot_Chassic_Control", &update_chassic_motor);

//Mbed pin setup----------------------------------------------------------------------------------

//Chassis Motor
PwmOut MotorFL(PA_1);
PwmOut MotorFR(PB_6);
PwmOut MotorBL(PA_6);
PwmOut MotorBR(PA_8);

//All Motor Enable
DigitalOut motor_enable(PA_4);

//Stop Pins for Motor
DigitalOut Stop_motor_0(PC_1);
DigitalOut Stop_motor_1(PC_2);
DigitalOut Stop_motor_2(PC_3);
DigitalOut Stop_motor_3(PC_0);

//3.3V Reference Pin
DigitalOut reference_voltage(PB_1);

//ROS connect LED
DigitalOut led(PA_5);

//Variables----------------------------------------------------------------------------------------


//Functions----------------------------------------------------------------------------------------
void start_motor(int motor_number){
    switch(motor_number){    
        case 0:
            Stop_motor_0 = 0;
            break;
        case 1:
            Stop_motor_1 = 0;
            break;
        case 2:
            Stop_motor_2 = 0;
            break;
        case 3:
            Stop_motor_3 = 0;
            break;
    }
}

void stop_motor(int motor_number){
    switch(motor_number){    
        case 0:
            Stop_motor_0 = 1;
            break;
        case 1:
            Stop_motor_1 = 1;
            break;
        case 2:
            Stop_motor_2 = 1;
            break;
        case 3:
            Stop_motor_3 = 1;
            break;
    }
}

void update_chassic_motor(const auto_bot::chassic_speed &income_chassic_control_message){
	MotorFL = income_chassic_control_message.motor0_power;
	MotorFR = income_chassic_control_message.motor1_power;
	MotorBL = income_chassic_control_message.motor2_power;
	MotorBR = income_chassic_control_message.motor3_power;
    
	(income_chassic_control_message.motor0_power==0.5)?stop_motor(0):start_motor(0);
	(income_chassic_control_message.motor1_power==0.5)?stop_motor(1):start_motor(1);
	(income_chassic_control_message.motor2_power==0.5)?stop_motor(2):start_motor(2);
	(income_chassic_control_message.motor3_power==0.5)?stop_motor(3):start_motor(3);
}

void motor_init(){
	//Disable Motors while Init
	motor_enable = 0;

	//Chassic Motor Init
	MotorFL.period_ms(20);
	MotorFR.period_ms(20);
	MotorBL.period_ms(20);
	MotorBR.period_ms(20);
	MotorFL = 0.5f;
	MotorFR = 0.5f;
	MotorBL = 0.5f;
	MotorBR = 0.5f;

	//3.3V Reference
	reference_voltage = 1;

	//Enable all motors at last
	wait(1);
	motor_enable = 1;
}

void publish_motor_data(){
	outgo_motor_message.motor0_power = MotorFL;
	outgo_motor_message.motor1_power = MotorFR;
	outgo_motor_message.motor2_power = MotorBL;
	outgo_motor_message.motor3_power = MotorBR;
	Motor_Data_Publsiher.publish(&outgo_motor_message);
}
int main() {
	//Init ROS node
	nh.initNode();

	//Init ROS Publisher
	nh.advertise(motor_pub);

	//Init ROS Subscriber
	nh.subscribe(sub);

	//Init Motors
	motor_init();

	//Publish motor data
	
	while (1) {
		//Handle ROS Communication
		nh.spinOnce();

		//Turn on LED if Nucleo is connected with ROS master
		if(nh.connected()){
			led = 1;
		}
		//Turn offf and stop motors if Nucleo disconnected
		else{
			led = 0;
			motor_init();
		}
		wait_ms(20);
	}

}
