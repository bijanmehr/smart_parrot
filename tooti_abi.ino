
#include "Filters.h"


/*   PIN DEFINES    */
#define MOTOR_BODY_PWM 10
#define MOTOR_BODY_IN1 7
#define MOTOR_BODY_IN2  8

#define HEAD_MOTOR_PWM 9
#define HEAD_MOTOR_IN1 12
#define HEAD_MOTOR_IN2 13


#define ZERO_CROSSING 11


#define ENCODER_1 2
#define ENCODER_2 3


#define TONGUE 4
#define OPEN_EYE 5
#define CLOSE_EYE 6

#define HEAD_TOUCH A0


 /*     uncomment this when you whant to measure the best threshold for head touch
 *     it just plot the head touch sensor value and you can open the plotter to see it  
 *     ----> when you uncomment this other things wont work <----
 */
//#define JUST_PLOT_HEAD_TOUCH

/*    filters for the head touch sensors      */
FilterOnePole lowpassFilter( LOWPASS, 1 ); // lowpass filter in 1 hz  for reading the head_touch sensor 
RunningStatistics lowpassFilterStats;                   

uint8_t counter = 0;

uint32_t time_out = 2000;
uint32_t befor_loop_time = 0;

uint16_t head_touch_sensor_threshold = 750;
uint16_t head_touch_sensor_threshold_offset = 150;
bool head_touch_flag = true;

bool tongue_flag = false;

void open_eye(uint8_t pwm,bool no_error = false);


void interrput_encoder_1(){
  counter++;
}



void interrput_encoder_2(){

}


void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_BODY_PWM,OUTPUT);
  pinMode(MOTOR_BODY_IN1,OUTPUT);
  pinMode(MOTOR_BODY_IN2,OUTPUT);

  pinMode(HEAD_MOTOR_PWM,OUTPUT);
  pinMode(HEAD_MOTOR_IN1,OUTPUT);
  pinMode(HEAD_MOTOR_IN2,OUTPUT);

  pinMode(ZERO_CROSSING,INPUT);

  pinMode(HEAD_TOUCH,INPUT);
  
  pinMode(ENCODER_1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1),interrput_encoder_1,FALLING);
  
  pinMode(ENCODER_2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2),interrput_encoder_1,FALLING);
  
  /*    this motor just move in 1 direction  */
  digitalWrite(MOTOR_BODY_IN1,HIGH);
  digitalWrite(MOTOR_BODY_IN2,LOW);


  open_eye(130); // for blinking pin13 on startup 

 
  lowpassFilterStats.setWindowSecs( 0.1 );
  
  Serial.begin(9600);
  Serial.setTimeout(50);  

  #ifndef JUST_PLOT_HEAD_TOUCH
    introduction();
  #endif
}

void loop() {

  #ifdef JUST_PLOT_HEAD_TOUCH
    while(1){
      uint16_t pure_data = analogRead(HEAD_TOUCH);
      lowpassFilter.input(pure_data);
      uint16_t filtered_data = lowpassFilter.output();
      lowpassFilterStats.input( filtered_data );
      Serial.print(pure_data);
      Serial.print(",");
      Serial.print(filtered_data);
      Serial.print(",");
      Serial.println(lowpassFilterStats.mean());
    }
  #endif

  check_head_buttons();
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil(' ');
    String value = Serial.readStringUntil('\n');
    command.remove(0,1); // remove 'G'
    value.remove(0,1); // remove 'S'
    switch(command.toInt()){
      
      case 1: // G1  -> dance
        dance(value.toInt());
        break;

      case 2: // G2  -> eye blink
        eye_blink(value.toInt());
        break;
        
       case 3: // G3  -> open(S255) ,close(S0) 
        mouth(value.toInt());
        break;

       case 4: // G4  -> open eye
        open_eye(value.toInt());
        break;

       case 5:// G5  -> close eye 
        close_eye(value.toInt());
        break;



       case 6:// G6  -> set the timeout value
        time_out = value.toInt();
        break;

       case 7:// G7  -> set the head_touch_sensor_threshold 
        head_touch_sensor_threshold = value.toInt();
        break;
        
       case 8:// G8  -> set head_touch_sensor_threshold_offset
        head_touch_sensor_threshold_offset = value.toInt();
        break;
    }
  }

}


void introduction(){
  Serial.println("parrot");
}


void open_eye(uint8_t pwm,bool no_error){
  
  digitalWrite(HEAD_MOTOR_IN1,LOW);
  digitalWrite(HEAD_MOTOR_IN2,HIGH);

  befor_loop_time = millis();
  while(!digitalRead(OPEN_EYE)){
  analogWrite(HEAD_MOTOR_PWM,pwm);
  if((millis() - befor_loop_time) > time_out){
    analogWrite(HEAD_MOTOR_PWM,0);
    if(!no_error) Serial.println("NOT DONE!");
    return;
    }
  }
  analogWrite(HEAD_MOTOR_PWM,0);
  if(!no_error) Serial.println("DONE!"); 
}

void close_eye(uint8_t pwm){ // prefer 130
  
  digitalWrite(HEAD_MOTOR_IN1,LOW);
  digitalWrite(HEAD_MOTOR_IN2,HIGH);

  befor_loop_time = millis();
  while(!digitalRead(CLOSE_EYE)){
  analogWrite(HEAD_MOTOR_PWM,pwm);
  if((millis() - befor_loop_time) > time_out){
    analogWrite(HEAD_MOTOR_PWM,0);
    Serial.println("NOT DONE!");
    return;
    }
  }
  analogWrite(HEAD_MOTOR_PWM,0);
  Serial.println("DONE!");
}

void eye_blink(uint8_t pwm){
  close_eye(pwm);
  delay(500);
  open_eye(pwm, true);
}

void mouth(uint8_t pwm){
  
  digitalWrite(HEAD_MOTOR_IN1,HIGH);
  digitalWrite(HEAD_MOTOR_IN2,LOW);
  analogWrite(HEAD_MOTOR_PWM,pwm);
  Serial.println("DONE!");
}

void dance(uint8_t cycle){    // THIS FUNC DONT HAVE TIMEOUT BECAUSE IT TAKE LONGE TIME TO DO THE JOB
  for(uint8_t i = 0 ; i < cycle ; i++){
    while(!digitalRead(ZERO_CROSSING)){
      analogWrite(MOTOR_BODY_PWM,255);
    }  
    analogWrite(MOTOR_BODY_PWM,0);
    while(digitalRead(ZERO_CROSSING)); // for get the debounce of the key
  
  }
  Serial.println("DONE!");
}


void check_head_buttons(){
  if(digitalRead(TONGUE) && tongue_flag == false){
    delay(200);
    Serial.println("tongue pressed!");
    tongue_flag = true;
  }
  else if(!digitalRead(TONGUE) && tongue_flag == true){
    Serial.println("tongue released!");
    tongue_flag = false;
  }
  
  lowpassFilter.input(analogRead(HEAD_TOUCH));
  lowpassFilterStats.input( lowpassFilter.output() );
  uint16_t output_data = lowpassFilterStats.mean();
  if(output_data <= head_touch_sensor_threshold && head_touch_flag == false){
    Serial.println("head touched!");  
    head_touch_flag = true;  
  }
  else if(output_data >= head_touch_sensor_threshold + head_touch_sensor_threshold_offset){
    head_touch_flag = false;
  }
}
