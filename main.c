#include<stdio.h>
#include<wiringPi.h>

# define IN1_PIN    1
# define IN4_PIN    4
# define IN5_PIN    5
# define IN6_PIN    6

# define MAX_SPEED  50
# define MIN_SPPED  0

void initDCMotor();
void goForward();
void goBackward();
void turnLeft();
void turnRight();
void stopDCMotor();
int cntr = 1
int main(void){
    if(wiringPiSetup() == -1)
        return 0;
    
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(IN5_PIN, OUTPUT);
    pinMode(IN6_PIN, OUTPUT);

    initDCMotor();

    while(cntr > 0){
        goForward();
        delay(500);
    }

}
// initalization DC Motor
void initDCMotor(){
    digitalWrite(IN1_PIN,HIGH);
    digitalWrite(IN4_PIN,HIGH);
    digitalWrite(IN5_PIN,HIGH);
    digitalWrite(IN6_PIN,HIGH);
    printf("Initialization motor\n");
}
// go forward
void goForward(){
    digitalWrite(IN1_PIN,HIGH);
    digiralWrite(IN4_PIN,LOW);
    digitalWrite(IN5_PIN,HIGH);
    digitalWrite(IN6_PIN,LOW);
    printf("GO Forward\n");
}
// go back
void goBackward(){
    digitalWrite(IN1_PIN,LOW);
    digitalWrite(IN4_PIN,HIGH);
    digitalWrite(IN5_PIN,LOW);
    digitalWrite(IN6_PIN,HIGH);
    printf("GO Back\n");
}
// turn left
void turnLeft(){
    digitalWrite(IN1_PIN,LOW);
    digitalWrite(IN4_PIN,HIGH);
    digitalWrite(IN5_PIN,HIGH);
    digitalWrite(IN6_PIN,LOW);
    printf("Turn Left\n");
}
// turn right
void turnRight(){
    digitalWrite(IN1_PIN,HIGH);
    digitalWrite(IN4_PIN,LOW);
    digitalWrite(IN5_PIN,LOW);
    digitalWrite(IN6_PIN,HIGH);
    printf("Turn Right\n");
}
// stop DC motor
void stopDCMotor(){
    digitalWrite(IN1_PIN,LOW);
    digitalWrite(IN4_PIN,LOW);
    digitalWrite(IN5_PIN,LOW);
    digitalWrite(IN6_PIN,LOw);
    printf("Stop DC Motor\n")
}
// initialization DC Motor in smooth motion
void initDCMotorSmooth(){
    pinMode(IN1_PIN, SOFT_PWM_OUTPUT);
    pinMode(IN4_PIN, SOFT_PWM_OUTPUT);
    pinMode(IN5_PIN, SOFT_PWM_OUTPUT);
    pinMode(IN6_PIN, SOFT_PWM_OUTPUT);
    // creates a PWM pin software control
    softPwmCreate(IN1_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(IN4_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(IN5_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(IN6_PIN, MIN_SPEED, MAX_SPPED);
}
// smooth right
void smoothRight(){
    // updates the PWM value on the given pin
    softPwmWrite(IN1_PIN, MAX_SPEED);
    softPwmWrite(IN4_PIN, MIN_SPPED);
    softPwmWrite(IN5_PIN, (MAX_SPEED) / 8);
    softPwmWrite(IN6_PIN, MIN_SPEED);
    printf("Smooth Right");
}
// smooth left
void smoothLeft(){
    softPwmWrite(IN1_PIN, MAX_SPEED / 8);
    softPwmWrite(IN4_PIN, MIN_SPEED);
    softPwmWrite(IN5_PIN, MAX_SPEED);
    softPwmWrite(IN6_PIN, MIN_SPEED);
}