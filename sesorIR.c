#include<stdio.h>

# define LEFT_IR_PIN    27
# define RIGTH_IR_PIN   28

void initIRSersor(){
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGTH_IR_PIN, INPUT);
}

int main(void){

    int Rvalue, Lvalue;

    if(wiringPiSetup() == -1)
        return 0;
    
    initIRSersor();

    while(1){
        Lvalue = digitalRead(LEFT_IR_PIN);
        Rvalue = digitalRead(RIGTH_IR_PIN);

        if(Lvalue == 1 && Rvalue == 0){
            printf("Go to right");
        }
        else if(Lvalue == 0 && Rvalue == 1){
            printf("Go to Left");
        }
        else if(Lvalue == 1 && Rvalue == 1){
            printf('stop');
        }
        else if(Lvalue == 0 && Rvalue == 0){
            printf('go forward');
        }
    }
}