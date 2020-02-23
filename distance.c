#include<stdio.h>
#include<wiringPi.h>

# define TRIG_PIN   28
# define ECHO_PIN   29

int main(void){
    
    if(wiringPiSetup() == -1 )
        return 0;
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    while(1){
        printf("Distance %dcm \n", getDistance());
        delay(100);
    }
}
void getDistance(){
    int start_time = 0, end_time = 0;
    float distance = 0;

    // regularize trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delay(500);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    //regularize echo pin
    while(digitalRead() == 0);
    start_time = micros();

    while(digitalRead() == 1);
    end_time = micros();

    distance = (end_time - start_time) / 29./ 2.;

    return (int) distance


}