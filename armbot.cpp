/*
we will firstly reset_all() valve control
then we will move_arm_0() 
then we will trigger move_arm_1() when move_arm_0_time.read() == arm_0_delay
then we will trigger release_grab() when move_arm_1_time.read() == release_grab_delay 
*/

#include "mbed.h"
#include "ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "QEI.h"
#include "math.h"

#define BAUD_RATE 115200

ros::NodeHandle  nh;

//function setup---------------------------

Timer move_arm_0_time;
Timer move_arm_1_time;

Ticker rosloop;

DigitalOut arm0Pin(D11);
DigitalOut arm1Pin(D12);
DigitalOut grabPin(D13);

InterruptIn button(D10);


//parameter--------------------------------
float arm_0_delay = 0.0f;
float release_grab_delay = 1.0f;

std_msgs::Bool process_finish;
ros::Publisher pub_process_finish("process_finish", &process_finish);
//Flags-------------------------------------
bool do_once_flag = false , finish_do_once_flag = false;
bool reset_all_flag = false , finish_reset_all_flag = false;
bool move_arm_0_flag = false , finish_move_arm_0_flag = false;
bool release_grab_flag = false , finish_release_grab_flag = false;

//functions---------------------------------
void test_do_once(){

    do_once_flag = true;
    finish_do_once_flag = false;
    
    reset_all_flag = true;
    finish_reset_all_flag = false;
    
    move_arm_0_flag = false;
    finish_move_arm_0_flag = false;
    
    
    release_grab_flag = false;
    finish_release_grab_flag = false;
    
    process_finish.data = false;    
    
}


void reset_all(){
    arm0Pin = 1;
    arm1Pin = 0;
    grabPin = 0;
    finish_reset_all_flag = true;
    move_arm_0_flag =true;
    move_arm_0_time.reset();
    move_arm_0_time.start();
}

void move_arm_0(){
    if(move_arm_0_time.read()>=arm_0_delay){
        arm1Pin = 1;
        move_arm_0_time.stop();
        move_arm_0_time.reset();
        finish_move_arm_0_flag = true;
        move_arm_1_time.reset();        
        move_arm_1_time.start();
        release_grab_flag = true;
        finish_release_grab_flag = false;
    }
}

void release_grab(){
    if(move_arm_1_time.read() >= release_grab_delay){
        grabPin = 1;
        arm0Pin = 0;
        arm1Pin = 0;
        move_arm_1_time.stop();
        move_arm_1_time.reset();
        finish_do_once_flag = true;
        process_finish.data = true;
    }
}


void ros_spin(){
    
    pub_process_finish.publish(&process_finish);    
    
    nh.spinOnce();
}

//callback function---------------------------------------

void do_once_callback(const std_msgs::Bool &msg){
    
    do_once_flag = true;
    finish_do_once_flag = false;
    
    reset_all_flag = true;
    finish_reset_all_flag = false;
    
    move_arm_0_flag = false;
    finish_move_arm_0_flag = false;
    
    
    release_grab_flag = false;
    finish_release_grab_flag = false;
    
    process_finish.data = false;

}

void reset_callback(const std_msgs::Bool &msg){
}

void arm_0_delay_callback(const std_msgs::Float32 &msg){
    arm_0_delay = msg.data;
}

void release_grab_delay_callback(const std_msgs::Float32 &msg){
    release_grab_delay = msg.data;
}

ros::Subscriber<std_msgs::Bool> sub_do_once("do_once" , &do_once_callback);
ros::Subscriber<std_msgs::Bool> sub_reset("reset" , &reset_callback);
ros::Subscriber<std_msgs::Float32> sub_arm_0_delay("arm_0_delay" , &arm_0_delay_callback);
ros::Subscriber<std_msgs::Float32> sub_release_grab_delay("release_grab_delay" , &release_grab_delay_callback);
//init-----------------------------------

void node_init(){
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();

    //publisher
    nh.advertise(pub_process_finish);   
    
    //subscriber
    nh.subscribe(sub_do_once);
    nh.subscribe(sub_arm_0_delay);
    nh.subscribe(sub_release_grab_delay);
    nh.subscribe(sub_reset);
    

    
}
//main-------------------------------------

int main(){
    
    node_init();
    button.rise(&test_do_once);
    rosloop.attach(&ros_spin , 0.02);
    
    while(1){

        if(do_once_flag && !finish_do_once_flag){
        
            if(reset_all_flag && !finish_reset_all_flag){
                reset_all();
            }
        
            if(move_arm_0_flag && !finish_move_arm_0_flag){
                move_arm_0();
            }   

            if(release_grab_flag && !finish_release_grab_flag){
                release_grab();
            }
        
    }

}


}