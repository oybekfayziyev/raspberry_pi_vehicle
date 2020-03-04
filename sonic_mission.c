#include<stdio.h>
#include<wiringPi.h>

void sonic_mission3(){

    if(wiringPiSetup() == -1)
        return 0;
    
    while(1){
        Lvalue = digitalRead(LEFT_IR_PIN);
        Rvalue = digitalRead(RIGTH_IR_PIN);

        distance = getDistance();
        goForward(0);
        printf("Distance %dc\n ",distance));
        
        if(Lvalue == 1 || Rvalue == 1 && distance == 100){
            forward(10);            
        }

        if(Lvalue == 1 || Rvalue == 1 && distance == 20){
            stopDCMotor();            
        }  
    }
}