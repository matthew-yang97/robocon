#include "mbed.h"
#include "TCS3200.h"
#include "MUX.h"

//setup functions --------------

TCS3200 sensor(PA_8, PA_11, PB_5, PB_4, PA_1);  //Create a TCS3200 object 
 //            S0    S1    S2     S3     OUT

MUX mux(PB_1, PF_0, PF_1);

I2CSlave slave(PB_7, PB_6);

Ticker ticker50hz ;
Ticker ticker100hz ;

//struct creation ---------------

struct color{
    long value;
    long max;
    long min;
    long ref;
    float level;
};

union data {
    float f;
    char c[4];
};

//color sensor function ----------------
void init_color_sensor(){

	sensor.SetMode(TCS3200::SCALE_2);

		
	
}



//ros function ---------------------
void rosloop(){
	

}





//main function --------------


int main() {
   
   sensor.SetMode(TCS3200::SCALE_2); 
   const int ColorNUM = 6;
   color ColorArray[ColorNUM];
   
   data coord;
   
   for(int i = 0; i < ColorNUM; i++){
        readColor(ColorArray, i, mux, sensor);
        ColorArray[i].max = ColorArray[i].min = ColorArray[i].value;
    }    
    
   char buf[10];
   slave.address(0xA0);
   
   while (1) {
       coord.f = getCoord(ColorArray, ColorNUM, mux, sensor); 
       int i = slave.receive();
       switch (i) {
           case I2CSlave::ReadAddressed:
               slave.write(coord.c, sizeof(float)); 
               break;
           case I2CSlave::WriteGeneral:
               slave.read(buf, 10);
               printf("Read G: %s\n", buf);
               break;
           case I2CSlave::WriteAddressed:
               slave.read(buf, 10);
               printf("Read A: %s\n", buf);
               break;
       }
       for(int i = 0; i < 10; i++) buf[i] = 0;    // Clear buffer
   }
}
