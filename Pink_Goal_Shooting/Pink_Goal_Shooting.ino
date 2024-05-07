#include <Pixy2.h>
#include <SPI.h>
#include <DualMAX14870MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Pixy2 pixy;
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler;

// PID constants
float Kp_t = 2.5; // Proportional constant
float Ki_t = 0; // Integral constant
float Kd_t = 0.2; // Derivative constant

// PID constants
float Kp_t_2 = 5; // Proportional constant
float Ki_t_2 = 0; // Integral constant
float Kd_t_2 = 0.2; // Derivative constant

float Kp_s = 1.5;
float Ki_s = 0;
float Kd_s = 0.1;

float Kp_s_2 = 0.5;
float Ki_s_2 = 0;
float Kd_s_2 = 0;

// Center point of the object
double setPoint = 320/2; //158
double threshold_turn = 15.8; // 15.8 10%

double input, output;
double integral = 0;
double derivative;
double previous_error = 0;
double lastAngle = 0;

// Refresh rate
unsigned long refreshRate = 100; // Refresh rate in milliseconds
unsigned long lastTime = 0;

int state;
int lastLight = 17;

int temp2;
int temp3;
double usualSpeed = 125;

const int pingPin = 25;
unsigned long pulseDuration;
float distance;

//double x_g_1 = 217;
double x_g_1 = 24;
double y_g_1 = 60;
double theta = 0;
bool ballDetected = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  // UART communication with computer
  Serial1.begin(115200); // UART communication with XBee module (Pin 18 & 19 of Arduino Mega)
  motors.enableDrivers(); //for the motors
  pixy.init();
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  lastAngle = euler.x();
  state = 1;
  pinMode(17, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  Serial.print("New loop at state ");
  Serial.println(state);
  switch(state){
    case 1:
      digitalWrite(lastLight, LOW);
      digitalWrite(17, HIGH);
      lastLight = 17;
      Serial.println("In state 1");
      pixy.ccc.getBlocks();
      if(pixy.ccc.numBlocks){
        if(pixy.ccc.blocks[0].m_signature == 6){
          int x = pixy.ccc.blocks[0].m_x; // 0~316
          if(x < setPoint + threshold_turn && x > setPoint - threshold_turn){
            stop();
            state = 2;
            delay(1000);
            break;
          } else{
            aPID_TURNING(Kp_t, Ki_t, Kd_t, setPoint);
            Serial.println("Complete");
            state = 2;
            delay(1000);
            break;
          }
        }
        else{
          Serial.println("Not detect sig 7");
          motors.setM1Speed(100);
          motors.setM2Speed(100);
          state = 6;
          break;
        }
      }
      else{
        Serial.println("Not detect any block");
        motors.setM1Speed(100);
        motors.setM2Speed(100);
        state = 6;
        break;
      }
    case 2:
      digitalWrite(lastLight, LOW);
      digitalWrite(16, HIGH);
      lastLight = 16;
      Serial.println("In state 2");
      temp2 = aPID_STRAIGHT(Kp_s, Ki_s, Kd_s, setPoint);
      if(temp2 == 0){
        stop();
        state = 1;
        break;
      }
      state = 3;
      stop();
      delay(1000);
      break;
    case 3:
      digitalWrite(lastLight, LOW);
      digitalWrite(15, HIGH);
      lastLight = 15;
      Serial.println("In state 3");
      Serial1.print('?');
      if (Serial1.available()) {
        Serial.println("Got signal");
        String data = Serial1.readStringUntil('\n'); // Read the line from XBee
        String output = parseCoordinates(data);  // Parse the coordinates
        String x_coord = output.substring(0,3);
        String y_coord = output.substring(3,6);
        int x = x_coord.toInt();
        int y = y_coord.toInt();
        if(x == 0 & y == 0){
          break;
        }
        theta = atan2(y_g_1 - y, x_g_1 - x) * 180 / 3.1415;
        theta = -1 * theta;
        if(theta < 0){
          theta = theta + 360;
        }
        Serial.print("x: ");
        Serial.print(x);
        Serial.print("y: ");
        Serial.print(y);
        Serial.print("Theta: ");
        Serial.println(theta);
        temp3 = aPID_TURNING2(Kp_t_2, Ki_t_2, Kd_t_2, theta, y);
        if (temp3 == 1){
          state = 4;
          delay(2000);
        } else{
          state = 1;
        }
        delay(500);
        break;
      }else{
        Serial.println("No signal");
        break;
      }
    case 4:
      digitalWrite(lastLight, LOW);
      digitalWrite(14, HIGH);
      lastLight = 14;
      pixy.ccc.getBlocks();
      if(pixy.ccc.numBlocks){
        if(pixy.ccc.blocks[0].m_signature != 6 || pixy.ccc.blocks[0].m_width < 70){
          state = 1;
          break;
        }
      } else{
        state = 1;
        break;
      }
      Serial.println("In state 4");
      aPID_STRAIGHT2(Kp_s_2, Ki_s_2, Kd_s_2, theta);
      stop();
      delay(500);
      state = 5;
      break;
    case 5:
      Serial.println("In state 5");
      stop();
      break;
    case 6:
      digitalWrite(lastLight, LOW);
      digitalWrite(13, HIGH);
      lastLight = 13;
      Serial.println("In state 6");
      while(1){
        pixy.ccc.getBlocks();
        if(pixy.ccc.numBlocks){
          if(pixy.ccc.blocks[0].m_signature == 6){
            if(pixy.ccc.blocks[0].m_x <= 200 && pixy.ccc.blocks[0].m_x >= 120){
              stop();
              state = 1;
              break;
            }
          }
        }
      }
  }

}

void stop(){
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

void aPID_TURNING(double Kp, double Ki, double Kd, double setpoint) {
  double integral = 0; //cumulative der
  double derivative; //change in error over change in time
  double old_err = 0; //error from previous time step
  double settleTime = 300;
  unsigned long settleCount = 0; //counter for settling time
  unsigned long oldSettleCount = 0; //counter for settling time
  unsigned long rr = 100; //refresh rate, time it takes between PID running
  unsigned long oldTime = 0; //time the last pid ran
  unsigned long now = millis();  //get current time
  double output; //output value of PID to motor
  double input;
  double speed; //speed output of the motors
  while (1) {
    now = millis();
    if (now - oldTime >= rr) { //if enough time has passed since the last pid call
      pixy.ccc.getBlocks();
      if(pixy.ccc.numBlocks){
        if(pixy.ccc.blocks[0].m_signature == 6){
          int x = pixy.ccc.blocks[0].m_x; // 0~316
          input = x;
          oldTime = now; //update oldTime
          double error = setpoint - input; //find error
          if (abs(error) < threshold_turn) { // if euler.x() is pretty close to the setpoint, and it is within the threshold for the settling time
            settleCount = millis(); //update the current settle time counter
            if (settleCount - oldSettleCount >= settleTime) { //if the error has been within the settle threshold for enough time, exit the turning PID
              // stop and exit the function
              motors.setM1Speed(0);
              motors.setM2Speed(0);
              return;
            }
          }
          else { //if we ever fall outside the threshold
            oldSettleCount = millis(); //resets the starting counter for settleCount
          }
          integral = integral + (error * (rr / 1000.0)); //calculate integral
          derivative = (error - old_err) / (rr / 1000.0); //calc deriv
          output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
          old_err = error; //updates old error to current error
          speed = constrain(output, -200, 200);
          // Serial.println(speed);
          motors.setM1Speed(speed); //wheels fed same speed, turn in opposite directions
          motors.setM2Speed(speed);
        }
      }
    }
  }
}

int aPID_STRAIGHT(double Kp, double Ki, double Kd, double setpoint) {
  double integral = 0; //cumulative der
  double derivative; //change in error over change in time
  double old_err = 0; //error from previous time step
  double settleTime = 300;
  unsigned long settleCount = 0; //counter for settling time
  unsigned long oldSettleCount = 0; //counter for settling time
  unsigned long rr = 100; //refresh rate, time it takes between PID running
  unsigned long oldTime = 0; //time the last pid ran
  unsigned long now = millis();  //get current time
  double output; //output value of PID to motor
  double input;
  double speed; //speed output of the motors
  double speedAdjust;
  double temp = 1;
  while (1) {
    now = millis();
    if (now - oldTime >= rr) { //if enough time has passed since the last pid call
      pixy.ccc.getBlocks();
      if(pixy.ccc.numBlocks){
        if(pixy.ccc.blocks[0].m_signature == 6){
          int x = pixy.ccc.blocks[0].m_x; // 0~316
          input = x;
          oldTime = now; //update oldTime
          double error = setpoint - input; //find error
          oldSettleCount = millis();
          // if(pixy.ccc.blocks[0].m_width > 70){
          //   temp = 0.8;
          // }
          // if (getAverageDistance() < 20) { // if euler.x() is pretty close to the setpoint, and it is within the threshold for the settling time
          //   settleCount = millis(); //update the current settle time counter
          //   if (settleCount - oldSettleCount >= settleTime) { //if the error has been within the settle threshold for enough time, exit the turning PID
          //     // stop and exit the function
          if(pixy.ccc.blocks[0].m_width > 85 ){
              motors.setM1Speed(0);
              motors.setM2Speed(0);
              Serial.println("-----------");
              //Serial.println(getAverageDistance());
              Serial.println("EXITTTT");
              return 1;
            // }
          }
          else { //if we ever fall outside the threshold
             //resets the starting counter for settleCount
          }
          integral = integral + (error * (rr / 1000.0)); //calculate integral
          derivative = (error - old_err) / (rr / 1000.0); //calc deriv
          output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
          old_err = error; //updates old error to current error
          speedAdjust = constrain(output, -50, 50);
          motors.setM1Speed((usualSpeed + speedAdjust)*temp); //wheels fed same speed, turn in opposite directions
          motors.setM2Speed((-1 * usualSpeed + speedAdjust)*temp);
        } else{
          if(millis() - oldSettleCount > settleTime){
            return 0;
          }
        }
      } else{
        if(millis() - oldSettleCount > settleTime){
          return 0;
        }
      }
    }
  }
}

int aPID_TURNING2(double Kp, double Ki, double Kd, double setpoint, int y) {
  double integral = 0; //cumulative der
  double derivative; //change in error over change in time
  double old_err = 0; //error from previous time step
  double settleTime = 300;
  unsigned long settleCount = 0; //counter for settling time
  unsigned long oldSettleCount = 0; //counter for settling time
  unsigned long settleCount2 = 0; //counter for settling time
  unsigned long oldSettleCount2 = 0; //counter for settling time
  unsigned long rr = 100; //refresh rate, time it takes between PID running
  unsigned long oldTime = 0; //time the last pid ran
  unsigned long now = millis();  //get current time
  double output; //output value of PID to motor
  double input;
  double speed; //speed output of the motors
  while (1) {
    now = millis();
    if (now - oldTime >= rr) { //if enough time has passed since the last pid call
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get the new angle
      input = euler.x();
      oldTime = now; //update oldTime
      double error = 0;
      // if(y < 60){
      //   if(input >= 180 && input < 360){
      //     error = input - setpoint ;
      //   } else{
      //     error = setpoint - input;
      //   }
      // } else if(y > 60){
      //   if(input >= 180 && input < 360){
      //     error = input - setpoint;
      //   } else{
      //     error = 180 - setpoint - input;
      //   }
      // } else{
      //   if(input >= 180 && input < 360){
      //     error = input -setpoint;
      //   } else{
      //     error = setpoint - input;
      //   }
      // }
      error = setpoint - input;
      // if(y>60 && input>=180 && input <= 360){
      //   error = error - 2 * input + 450;
      // }
      Serial.println("------------------");
      Serial.print("Setpoint");
      Serial.println(setpoint);
      Serial.print("Input");
      Serial.println(input);
      Serial.print("Error");
      Serial.println(error);
      Serial.println();
      // if(getAverageDistance() < 4.5 ||  getAverageDistance() > 12){
      //   motors.setM1Speed(0);
      //   motors.setM2Speed(0);
      //   return 0;
      // }
      pixy.ccc.getBlocks();
      if(pixy.ccc.numBlocks){
        if(pixy.ccc.blocks[0].m_signature == 6){
          ballDetected = 1;
        } else{
          ballDetected = 0;
        }
      }else{
        ballDetected = 0;
      }
      if(ballDetected == 0){
        settleCount2 = millis(); //update the current settle time counter
        if (settleCount2 - oldSettleCount2 >= settleTime) { //if the error has been within the settle threshold for enough time, exit the turning PID
          // stop and exit the function
          motors.setM1Speed(0);
          motors.setM2Speed(0);
          return 0;
        }
      }else{
        oldSettleCount2 = millis();
      }
      if (abs(error) < 10) { // if euler.x() is pretty close to the setpoint, and it is within the threshold for the settling time
        settleCount = millis(); //update the current settle time counter
        if (settleCount - oldSettleCount >= settleTime) { //if the error has been within the settle threshold for enough time, exit the turning PID
          // stop and exit the function
          motors.setM1Speed(0);
          motors.setM2Speed(0);
          return 1;
        }
      }
      else { //if we ever fall outside the threshold
        oldSettleCount = millis(); //resets the starting counter for settleCount
      }
      integral = integral + (error * (rr / 1000.0)); //calculate integral
      derivative = (error - old_err) / (rr / 1000.0); //calc deriv
      output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
      old_err = error; //updates old error to current error
      speed = constrain(output, -100, 100);
      // Serial.println(speed);
      motors.setM1Speed(speed); //wheels fed same speed, turn in opposite directions
      motors.setM2Speed(speed);
      //Serial.println(error);
    }
  }
}

void aPID_STRAIGHT2(double Kp, double Ki, double Kd, double setpoint) {
  double integral = 0; //cumulative der
  double derivative; //change in error over change in time
  double old_err = 0; //error from previous time step
  double settleTime = 300;
  unsigned long settleCount = 0; //counter for settling time
  unsigned long oldSettleCount = 0; //counter for settling time
  unsigned long rr = 100; //refresh rate, time it takes between PID running
  unsigned long oldTime = 0; //time the last pid ran
  unsigned long now = millis();  //get current time
  double output; //output value of PID to motor
  double input;
  double speed; //speed output of the motors
  double speedAdjust;
  while (1) {
    now = millis();
    if (now - oldTime >= rr) { //if enough time has passed since the last pid call
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get the new angle
      input = euler.x();
      oldTime = now; //update oldTime
      double error = 0;
      // if (input > 190) { //190 instead of 180 so it doesn't flip flop between 178 and -178
      //   input = input - 360;
      // }
      error = setpoint - input;
      Serial.print("Setpoint");
      Serial.println(setpoint);
      Serial.print("Input");
      Serial.println(input);
      Serial.print("Error");
      Serial.println(error);
      Serial.println();
      Serial1.print('?');
      if (Serial1.available()) {
        String data = Serial1.readStringUntil('\n'); // Read the line from XBee
        String output = parseCoordinates(data);  // Parse the coordinates
        String x_coord = output.substring(0,3);
        String y_coord = output.substring(3,6);
        int x = x_coord.toInt();
        int y = y_coord.toInt();
        //if (x > 160){
        //x < 30
        if (x < 30){
          settleCount = millis(); //update the current settle time counter
          //if (settleCount - oldSettleCount >= settleTime) { //if the error has been within the settle threshold for enough time, exit the turning PID
            // stop and exit the function
            motors.setM1Speed(0);
            motors.setM2Speed(0);
            return;
          //}
          //else{
            //oldSettleCount = millis(); //resets the starting counter for settleCount
          //}
        }
      }
      integral = integral + (error * (rr / 1000.0)); //calculate integral
      derivative = (error - old_err) / (rr / 1000.0); //calc deriv
      output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
      old_err = error; //updates old error to current error
      speedAdjust = constrain(output, -75, 75);
      // Serial.println(speed);
      motors.setM1Speed(175 + speedAdjust); //wheels fed same speed, turn in opposite directions
      motors.setM2Speed(-1 * 175 + speedAdjust);
    }
  }
}

void measureDistance() {

  //set pin as output so we can send a pulse
  pinMode(pingPin, OUTPUT);
  //set output to LOW
  digitalWrite(pingPin, LOW);
  delayMicroseconds(5);

  //now send the 5uS pulse out to activate the PING
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  //now we need to change the digital pin
  //to input to read the incoming pulse
  pinMode(pingPin, INPUT);

  //finally, measure the length of the incoming pulse
  pulseDuration = pulseIn(pingPin, HIGH);
  distance = (pulseDuration * 0.0001 * 343) / 2; //conversion for the distance

}

float getAverageDistance() {
  int numReadings = 5;
  float sumDistance = 0;
  for (int i = 0; i < numReadings; ++i) {
    measureDistance();  // Assuming this function updates 'distance'
    sumDistance += distance;
    delay(10);  // Add a small delay between readings
  }
  return sumDistance / numReadings;
}

String parseCoordinates(String data) {
  // Example input: M,TTTT,XXX,YYY
  int firstCommaIndex = data.indexOf(',');
  int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
  int thirdCommaIndex = data.indexOf(',', secondCommaIndex + 1);

  if (thirdCommaIndex != -1) { // Make sure all commas are found
    String xCoord = data.substring(secondCommaIndex + 1, thirdCommaIndex);
    String yCoord = data.substring(thirdCommaIndex + 1);
    return xCoord + yCoord;
  } else {
    Serial.println("Error: Data format incorrect");
  }
}