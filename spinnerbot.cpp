//we will firstly return_init_posi()
//then we will write_pwm() for 5 seconds
//then we will release_grab() for 2 seconds
//then we will return_init_posi()
//and reset_data()

//#include "ST_F401_84MHZ.h"
#include "mbed.h"
#include "ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "QEI.h"
#include "math.h"

#define BAUD_RATE 115200

ros::NodeHandle  nh;
//subscriber-----------------------------------
bool stop_motor = false;
//bool release_grab = false;
bool reset = false;
float tar_deg = 0.0;
float tar_rpm = 0.0;
float rpm_pwm = 0.0;

float rpm_mod = 100.0;
float rpm_step = 60.0;

//publisher-------------------------------------
std_msgs::Bool throw_is_finished;
std_msgs::Float32 cur_deg;
std_msgs::Float32 cur_rpm;

ros::Publisher pub_throw_is_finished("throw_is_finished", &throw_is_finished);
ros::Publisher pub_cur_deg("cur_deg", &cur_deg);
ros::Publisher pub_cur_rpm("cur_rpm", &cur_rpm);

//setup------------------------------------------------------------
InterruptIn button(USER_BUTTON);

Ticker rosloop;
Ticker fastloop;

PwmOut pwmSignal(D5);

Timer timer;
Timer grab_time;
Timer pwm_time;
Timer write_time;

DigitalOut myled(LED1);
DigitalOut stopPin(D6);
DigitalOut enablePin(D7);
DigitalOut releasePin(D8);

//encoder setting constants-------------------------------------------
int countLimit = 1000;
float gear_ratio = 0.5f;
int inv_gear_ratio = 2;
float init_posi = 90.0f;
QEI qei(D2, D3, D4, countLimit, QEI::X4_ENCODING);   //what is this 

//--------------------------------------------------------------

int delay = 20;
int temp_pluse = 0;
int r =36;
int finished = 0;
int timeA = 0;
int timeB = 0;
float time_difference = 10000.0f; //in us
float time_duration = 0.0f;
float temp_count = 0.0f;
float calculated_speed = 0.0;
float feedback_deg= 0.0;
float err_bound = 10.0;

//---------------------------------FLAGS---------------------------
bool write_pwm_flag = false , finish_write_pwm_flag = false;
bool release_grab_flag = false , finish_release_grab_flag = false;
bool do_once_flag = false , finish_do_once_flag = false;
bool return_init_posi_flag = false, finish_return_init_posi_flag = false;
bool return_reset_posi_flag = false, finish_return_reset_posi_flag = false;

bool test_do_once_flag = false , finish_test_do_once_flag = false;
bool test_run_flag = false, finish_test_run_flag = false;
//----------------------------------------------------------------
float cal_pwm(float data){
    return 0.1f+(data*0.0016f);
}


void cal_deg(){
    cur_deg.data = abs((float)((qei.getPulses() * 360/ ( (4 * (countLimit/inv_gear_ratio) )) % 360)));
}

void cal_rpm(){
    
    if(finished == 0) {
        temp_count = abs(qei.getPulses());
        timeA = timer.read_us();
        finished = 1;
    } else if(finished == 1) {
        //if CW, temp_count < getP; if CCW, temp_count > getP
        //if(abs(qei.getPulses()) >= temp_count + 200) { // 36deg;
        //temp_count is prev
        
        float difference = abs((float)(temp_count - abs(qei.getPulses())));
        timeB = timer.read_us();
        float time_duration=(float)( timeB - timeA);
               
        if(time_duration >= time_difference ) { // 100us
            calculated_speed = (float)(difference /  (time_difference*gear_ratio*(float)(countLimit))*12000000.0f) ;
            cur_rpm.data = calculated_speed ;
            finished = 0;
        }
    }    
}

void blink_led()
{
    myled = !myled;
    wait_ms(1);
}

void reset_encoder(){
    //qei.reset();
    releasePin = !releasePin; 
}


//main function for auto -------------------------------------------------------------------

//1st
void return_init_posi(float data){
    
    stopPin = 0;
    pwmSignal.write(cal_pwm(20.0f));

    if(cur_deg.data > data-2.0f && cur_deg.data < data+2.0f){
        stopPin = 1;
        finish_return_init_posi_flag = true;
        pwm_time.reset();
        pwm_time.start();
        write_pwm_flag = true;
    }
}

//2nd
void write_pwm(float data){
    
    stopPin = 0;
    int cal_wait_time = int (((data - 100) / rpm_step)/2) ;
    
    if(rpm_mod < data && write_time.read() >= 0.5f ){
        rpm_mod = rpm_mod + rpm_step ;
        if(rpm_mod >= data)
            rpm_mod = data;
        write_time.reset();
        write_time.start();
    } 
        
    pwmSignal.write(cal_pwm(rpm_mod));
    
    if(pwm_time.read() > cal_wait_time+2 ){
      
    //pwmSignal.write(cal_pwm(data));
    
    //if(pwm_time.read() > 0.5 ){  
        finish_write_pwm_flag = true;
        release_grab_flag = true;
        pwm_time.stop();
        grab_time.reset();
    }
    
}

//3rd
void release_grab(){
    
    if(cur_deg.data >= tar_deg-err_bound && cur_deg.data < tar_deg+err_bound){
        releasePin = 1 ;
        myled = 1;
        throw_is_finished.data = true;
        grab_time.start();
    }
    
    if(grab_time.read() >2){
        releasePin = 1;
        myled = 0;
        grab_time.stop();
        grab_time.reset();
        finish_release_grab_flag = true;
        throw_is_finished.data = true;
        return_reset_posi_flag = true;
    }

}

//4th
void return_reset_posi(float data){
    
    stopPin = 0;
    pwmSignal.write(cal_pwm(20.0));

    if(cur_deg.data > data-2 && cur_deg.data < data+2){
        stopPin = 1;
        rpm_mod = 100.0f;
        finish_return_reset_posi_flag = true;
        finish_do_once_flag = true;
    }

}

//testing function---------------------------------------------------------------------------------------
void do_once_test_callback(const std_msgs::Float32 &msg){
    
    stopPin = 0;
    pwmSignal.write(cal_pwm(msg.data));
    

}


//callback function----------------------------------------------------------------------------------------

void stop_motor_callback(const std_msgs::Bool &msg)
{
    if(msg.data) {
        stopPin = 1;
    } else {
        stopPin = 0;
    }
}

void reset_callback(const std_msgs::Bool &msg)
{
    reset = msg.data;
    //rpm_pwm = 0.1;
    //stopPin = 1;
    qei.reset();
    //release_grab = false;
    throw_is_finished.data = false;
}

void enable_callback(const std_msgs::Bool &msg)
{
    if (!msg.data){
            enablePin = true;
            }
    if (msg.data){
            enablePin = false;
            }
             
    }

void tar_deg_callback(const std_msgs::Float32 &msg)
{
    tar_deg = msg.data;
}

void release_grab_callback(const std_msgs::Bool &msg)
{   
    releasePin = !releasePin;
    //grab_time.reset();
    //release_grab = msg.data;
}

void tar_rpm_callback(const std_msgs::Float32 &msg)
{   
    //pwm_time.reset();
    tar_rpm = msg.data;
    rpm_pwm = cal_pwm(tar_rpm);
    //write_pwm_flag = true;
    //finish_write_pwm_flag = false;
    //pwmSignal.write(rpm_pwm);
    //pwm_time.start();
}

void do_once_callback(const std_msgs::Bool &msg){
    
    
    do_once_flag = true;
    finish_do_once_flag = false;

    return_init_posi_flag = true;
    finish_return_init_posi_flag =false;

    write_pwm_flag = false; 
    finish_write_pwm_flag = false;

    release_grab_flag = false;
    finish_release_grab_flag = false;

    return_reset_posi_flag = false;
    finish_return_reset_posi_flag = false;
    
    throw_is_finished.data = false;
}
    
    
//sub obj
ros::Subscriber<std_msgs::Bool> sub_stop_motor("stop_motor", &stop_motor_callback);
ros::Subscriber<std_msgs::Float32> sub_tar_rpm("tar_rpm", &tar_rpm_callback);
ros::Subscriber<std_msgs::Float32> sub_tar_deg("tar_deg", &tar_deg_callback);
ros::Subscriber<std_msgs::Bool> sub_release_grab("release_grab" , &release_grab_callback);
ros::Subscriber<std_msgs::Bool> sub_enable("enable" , &enable_callback);
ros::Subscriber<std_msgs::Bool> sub_reset("reset" , &reset_callback);
ros::Subscriber<std_msgs::Bool> sub_do_once("do_once" , &do_once_callback);

ros::Subscriber<std_msgs::Float32> sub_do_once_test("do_once_test" , &do_once_test_callback);

//-----------------------------------------------------------------------------------------------------------

void init_pub_data(){
    
    throw_is_finished.data = false;
}

void ros_spin(){
    
    //blink_led();
    pub_throw_is_finished.publish(&throw_is_finished);
    pub_cur_deg.publish(&cur_deg);
    pub_cur_rpm.publish(&cur_rpm);

    nh.spinOnce();
}

void fast_loop(){
    
    cal_deg();
    cal_rpm();
    
    
    }
//---------------------------------------------------------
int main()
{
    //F401_init84(0);
    init_pub_data();
    pwmSignal.period_ms(20);                                //50Hz
    pwmSignal = 0.0f;
    pwmSignal.write(0.1);                                       //setting duty cycle
    enablePin = 1;
    stopPin = 1;
    button.rise(&reset_encoder);
    
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
  
    //publisher
    nh.advertise(pub_throw_is_finished);
    nh.advertise(pub_cur_deg);
    nh.advertise(pub_cur_rpm);
 
    //subscriber
    nh.subscribe(sub_stop_motor);
    nh.subscribe(sub_tar_deg);
    nh.subscribe(sub_release_grab);
    nh.subscribe(sub_reset);
    nh.subscribe(sub_tar_rpm);
    nh.subscribe(sub_enable);
    nh.subscribe(sub_do_once);
    nh.subscribe(sub_do_once_test);      

    timer.start();
    
    rosloop.attach(&ros_spin , 0.02);
    //fastloop.attach(&fast_loop, 0.001);
    write_time.start();
    
    while(1) {
        
        cal_deg();
        cal_rpm();
        
        if(do_once_flag && !finish_do_once_flag){

            if(return_init_posi_flag && !finish_return_init_posi_flag){
                return_init_posi(init_posi);
            }

            if(write_pwm_flag && !finish_write_pwm_flag){
                write_pwm(tar_rpm);
            }

            if(release_grab_flag && !finish_release_grab_flag){
                release_grab();
            }

            if(return_reset_posi_flag && !finish_return_reset_posi_flag){
                return_reset_posi(45.0f);
            }               
            
                
        }

        
        
    }
}