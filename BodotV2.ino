#include <Arduino.h>
#include <NewPing.h>

void manuelMode();
void autoMode(unsigned int minDistance);
void findWay();
void brake();
void turn(bool left);
void forward(bool back);
int getSerialData();

// Sonar led signal pins
const int RIGHT_LED_PIN         = 8;
const int FRONT_LED_PIN         = 9;
const int LEFT_LED_PIN          = 10;

// Control pins
const int CONTROL_PIN_1         = 22;
const int CONTROL_PIN_2         = 24;
const int CONTROL_PIN_3         = 28;

const int MODE_LED_PIN          = 23;

// Left back motor
const int MOTOR_LB_ENABLE_PIN   = 2;
const int MOTOR_LB_DIR_PIN1     = 36;
const int MOTOR_LB_DIR_PIN2     = 37;

// Right back motor
const int MOTOR_RB_ENABLE_PIN   = 3;
const int MOTOR_RB_DIR_PIN1     = 32;
const int MOTOR_RB_DIR_PIN2     = 33;

// Left front motor
const int MOTOR_LF_ENABLE_PIN   = 4;
const int MOTOR_LF_DIR_PIN1     = 34;
const int MOTOR_LF_DIR_PIN2     = 35;

// Right front motor
const int MOTOR_RF_ENABLE_PIN   = 5;
const int MOTOR_RF_DIR_PIN1     = 30;
const int MOTOR_RF_DIR_PIN2     = 31;

const int LEFT_ECHO_PIN         = 38;
const int LEFT_TRIGGER_PIN      = 39;

const int FRONT_ECHO_PIN        = 40;
const int FRONT_TRIGGER_PIN     = 41;

const int RIGHT_ECHO_PIN        = 42;
const int RIGHT_TRIGGER_PIN     = 43;

const int SONAR_MAX_DISTANCE    = 200;

int frontMinDistance          = 40;
int frontMaxDistance          = 60;

int lefMinDistance           = 30;
int lefMaxDistance           = 45;

int rightMinDistance          = 30;
int rightMaxDistance          = 45;

float accelerationRate         = 2;

NewPing l_sonar(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, SONAR_MAX_DISTANCE);
NewPing f_sonar(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, SONAR_MAX_DISTANCE);
NewPing r_sonar(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, SONAR_MAX_DISTANCE);

unsigned long printInfoPrevMillis     = 0;
unsigned long printInfoInterval       = 1000;

unsigned long leftSonarPrevMillis     = 0;
unsigned long frontSonarPrevMillis    = 0;
unsigned long rightSonarPrevMillis    = 0;

int prevFrontSonarValue               = 0;
int prevLeftSonarValue                = 0;
int prevRightSonarValue               = 0;

// Used for acceleration of the car (we should slowly increase velocity)
unsigned long accelerationPrevMillis  = 0;
const long accelerationInterval       = 2000;

const long leftSonarInterval          = 80;
const long frontSonarInterval         = 75;
const long rightSonarInterval         = 80;

unsigned int leftDistance             = 0;
unsigned int frontDistance            = 0;
unsigned int rightDistance            = 0;

unsigned int prevVelocity             = 0;
unsigned int velocity                 = 0;

unsigned char prevDirection           = 'S';

int prevSerialDataValue               = 0;

bool isAutoMode                       = false;
bool booting                          = true;
bool debug                            = false;

unsigned int turnSpeed                = 0;
unsigned int minSpeed                 = 0;
unsigned int maxSpeed                 = 0;

float manuelSpeedLevel                = 0;

long sonarPingTime                    = 0;

void setup(){
    Serial.begin(9600);
    Serial1.begin(9600);

    pinMode(RIGHT_LED_PIN, OUTPUT);
    pinMode(FRONT_LED_PIN, OUTPUT);
    pinMode(LEFT_LED_PIN, OUTPUT);
    pinMode(MODE_LED_PIN, OUTPUT);

    // Right motors
    pinMode(MOTOR_RF_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_RB_ENABLE_PIN, OUTPUT);
  
    // Left motors
    pinMode(MOTOR_LF_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_LB_ENABLE_PIN, OUTPUT);
  
    // Right motors
    pinMode(MOTOR_RF_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_RF_DIR_PIN2, OUTPUT);
    pinMode(MOTOR_RB_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_RB_DIR_PIN2, OUTPUT);
  
    // Left motors
    pinMode(MOTOR_LF_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_LF_DIR_PIN2, OUTPUT);
    pinMode(MOTOR_LB_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_LB_DIR_PIN2, OUTPUT);
  
    pinMode(LEFT_TRIGGER_PIN, OUTPUT);
    pinMode(FRONT_TRIGGER_PIN, OUTPUT);
    pinMode(RIGHT_TRIGGER_PIN, OUTPUT);
  
    pinMode(LEFT_ECHO_PIN, INPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(RIGHT_ECHO_PIN, INPUT);

    pinMode(CONTROL_PIN_1, INPUT_PULLUP);
    pinMode(CONTROL_PIN_2, INPUT_PULLUP);
    pinMode(CONTROL_PIN_3, INPUT_PULLUP);
  
    brake();
}

void loop(){
    unsigned long currentMillis = millis();
    
    unsigned int minDistance = pingSonar();

    if (booting){     
      digitalWrite(RIGHT_LED_PIN, HIGH);
      delay(1000);

      minDistance = pingSonar();
      
      digitalWrite(FRONT_LED_PIN, HIGH);
      delay(1000);

      minDistance = pingSonar();
      
      digitalWrite(LEFT_LED_PIN, HIGH);
      delay(1000);

      minDistance = pingSonar();

      bool high = true;
      for (int i=0; i<8 ; i++){
          digitalWrite(RIGHT_LED_PIN, high ? HIGH : LOW);
          digitalWrite(FRONT_LED_PIN, high ? HIGH : LOW);
          digitalWrite(LEFT_LED_PIN, high ? HIGH : LOW);
          delay(250);
          high = !high;
      }
      
      booting = false;
    }

    int control1Value = digitalRead(CONTROL_PIN_1);
    int control2Value = digitalRead(CONTROL_PIN_2);
    int control3Value = digitalRead(CONTROL_PIN_3);    
      
    // Mode (auto? manuel?)
    if (control1Value == HIGH) {
        isAutoMode = true;
        
        digitalWrite(MODE_LED_PIN, HIGH);
    } else {
        isAutoMode = false;

        digitalWrite(MODE_LED_PIN, LOW);
    }

    // Speed control settings
    if (control2Value == LOW && control3Value == LOW) {
        minSpeed = 40;
        turnSpeed = 60;
        maxSpeed = 80;
        frontMinDistance = 30;
        frontMaxDistance = 50;
        lefMinDistance = 20;
        lefMaxDistance = 35;
        rightMinDistance = 20;
        rightMaxDistance = 35;
        accelerationRate = 3;
    } else if (control2Value == LOW && control3Value == HIGH) {
        minSpeed = 50;
        turnSpeed = 65;
        maxSpeed = 100;
        frontMinDistance = 33;
        frontMaxDistance = 53;
        lefMinDistance = 22;
        lefMaxDistance = 37;
        rightMinDistance = 22;
        rightMaxDistance = 37;
        accelerationRate = 2.7;          
    } else if (control2Value == HIGH && control3Value == LOW) {
        minSpeed = 70;
        turnSpeed = 75;
        maxSpeed = 120;
        frontMinDistance = 36;
        frontMaxDistance = 56;
        lefMinDistance = 24;
        lefMaxDistance = 39;
        rightMinDistance = 24;
        rightMaxDistance = 39;
        accelerationRate = 2.4;             
    } else if (control2Value == HIGH && control3Value == HIGH) {
        minSpeed = 80;
        turnSpeed = 75;
        maxSpeed = 200;
        frontMinDistance = 39;
        frontMaxDistance = 59;
        lefMinDistance = 26;
        lefMaxDistance = 41;
        rightMinDistance = 26;
        rightMaxDistance = 41;
        accelerationRate = 2;                
    }

    prevVelocity = minSpeed;
    
    velocity = minSpeed;

    manuelSpeedLevel = (maxSpeed - minSpeed) / 10;    
    
    if (isAutoMode){
      autoMode(minDistance);
    } else {
      manuelMode();
    }

    if (debug) {
      printInfo();
    }
}

void autoMode(unsigned int minDistance){
    unsigned long currentMillis = millis();

    // Set velocity according to calculated distances (left, front, right)
    velocity = map(minDistance, 0, SONAR_MAX_DISTANCE, minSpeed, maxSpeed);

    // Do we need to accelerate?
    if (currentMillis - accelerationPrevMillis >= accelerationInterval) {
      if (prevVelocity < velocity){
        accelerationPrevMillis = currentMillis;

        // Accelerate 
        velocity = prevVelocity + ((prevVelocity * accelerationRate) / 100);
      }
    }

    // Check min, max speed thresholds
    if (velocity > maxSpeed){
      velocity = maxSpeed;
    } else if (velocity < minSpeed){
      velocity = minSpeed;
    }

    // Set min distances according to calculated velocity
    unsigned int mappedFrontMinDistance = map(velocity, minSpeed, maxSpeed, frontMinDistance, frontMaxDistance);
    unsigned int mappedLeftMinDistance = map(velocity, minSpeed, maxSpeed, lefMinDistance, lefMaxDistance);
    unsigned int mappedRightMinDistance = map(velocity, minSpeed, maxSpeed, rightMinDistance, rightMaxDistance);

    updateLedStatus(mappedFrontMinDistance, mappedLeftMinDistance, mappedRightMinDistance);
    
    if (frontDistance < mappedFrontMinDistance || 
        leftDistance < mappedLeftMinDistance || 
        rightDistance < mappedRightMinDistance){
        findWay();
    } else {
        forward(false);
    }

    prevVelocity = velocity;  
}

void manuelMode(){ 
    int data = getSerialData();
    
    // Car Bluetooth RC Android app is used here
    if (data == 48){
        velocity = minSpeed;
    } else if (data == 48){
        velocity = (int)(minSpeed + manuelSpeedLevel);
    } else if (data == 50){
        velocity = (int)(minSpeed + manuelSpeedLevel * 2);
    } else if (data == 51){
        velocity = (int)(minSpeed + manuelSpeedLevel * 3);
    } else if (data == 52){
        velocity = (int)(minSpeed + manuelSpeedLevel * 4);
    } else if (data == 53){
        velocity = (int)(minSpeed + manuelSpeedLevel * 5);
    } else if (data == 54){
        velocity = (int)(minSpeed + manuelSpeedLevel * 6);
    } else if (data == 55){
        velocity = (int)(minSpeed + manuelSpeedLevel * 7);
    } else if (data == 56){
        velocity = (int)(minSpeed + manuelSpeedLevel * 8);
    } else if (data == 57){
        velocity = (int)(minSpeed + manuelSpeedLevel * 9);
    } else if (data == 58){
        velocity = (int)(minSpeed + manuelSpeedLevel * 10);
    } else if (data == 70) {
        if (frontDistance  < frontMinDistance || 
            leftDistance < lefMinDistance ||
            rightDistance  < rightMinDistance){
            brake();
        } else {
            forward(false);
        }
    } else if (data == 66) {
        forward(true); 
    } else if (data == 76) {
        if (leftDistance < lefMinDistance){
            brake();
        } else {
            turn(true);
        }
    } else if (data == 82) {
        if (rightDistance  < rightMinDistance){
            brake();
        } else {
            turn(false);
        }
    } else {
        brake();
    }

    updateLedStatus(frontMinDistance, lefMinDistance, rightMinDistance);
}

int getSerialData(){
    int data = prevSerialDataValue;
    
    if (Serial1.available()){
        data = Serial1.read();
    
        while(Serial1.available() && !data && data == 0) {
            data = Serial1.read();
        }
  
        prevSerialDataValue = data;
    }

    return data;
}

int pingSonar(){
    unsigned long currentMillis = millis();
  
    unsigned int minDistance = 65535;
    
    if (currentMillis - frontSonarPrevMillis >= frontSonarInterval) {
      frontSonarPrevMillis = currentMillis;
  
      frontDistance = f_sonar.ping_cm();
      int retry = 0;
      while (frontDistance == NO_ECHO && retry < 5){
        frontDistance = f_sonar.ping_cm();

        delay(5);

        retry++;
      }

      if (frontDistance == NO_ECHO){
        frontDistance = prevFrontSonarValue;
      }
      
      if (frontDistance > SONAR_MAX_DISTANCE){
        frontDistance = SONAR_MAX_DISTANCE;
      }
  
      if (frontDistance < minDistance){
        minDistance = frontDistance;
      }

      prevFrontSonarValue = frontDistance;
    }
    
    if (currentMillis - leftSonarPrevMillis >= leftSonarInterval) {
      leftSonarPrevMillis = currentMillis;
  
      leftDistance = l_sonar.ping_cm();
      int retry = 0;
      while (leftDistance == NO_ECHO && retry < 5){
        leftDistance = l_sonar.ping_cm();

        delay(5);

        retry++;
      }

      if (leftDistance == NO_ECHO){
        leftDistance = prevLeftSonarValue;
      }
      
      if (leftDistance > SONAR_MAX_DISTANCE){
        leftDistance = SONAR_MAX_DISTANCE;
      }
  
      if (leftDistance < minDistance){
        minDistance = leftDistance;
      }

       prevLeftSonarValue = leftDistance;
    }
  
    if (currentMillis - rightSonarPrevMillis >= rightSonarInterval) {
      rightSonarPrevMillis = currentMillis;
  
      rightDistance = r_sonar.ping_cm();
      int retry = 0;
      while (rightDistance == NO_ECHO && retry < 5){
        rightDistance = r_sonar.ping_cm();

        delay(5);

        retry++;
      }

      if (rightDistance == NO_ECHO){
        rightDistance = prevRightSonarValue;
      }
        
      if (rightDistance > SONAR_MAX_DISTANCE){
        rightDistance = SONAR_MAX_DISTANCE;
      }
  
      if (rightDistance < minDistance){
        minDistance = rightDistance;
      }

      prevRightSonarValue = rightDistance;
    }

    sonarPingTime = millis() - currentMillis;
    
    return minDistance;
}

void findWay(){
    // Stop first
    brake();

    bool turnLeft = leftDistance > rightDistance;
    int data = 0;
    unsigned int mappedFrontMinDistance = map(velocity, minSpeed, maxSpeed, frontMinDistance, frontMaxDistance);
    unsigned int mappedLeftMinDistance = map(velocity, minSpeed, maxSpeed, lefMinDistance, lefMaxDistance);
    unsigned int mappedRightMinDistance = map(velocity, minSpeed, maxSpeed, rightMinDistance, rightMaxDistance);
    while ( (frontDistance < mappedFrontMinDistance || 
           leftDistance < mappedLeftMinDistance ||
           rightDistance < mappedRightMinDistance) && isAutoMode ){
          turn(turnLeft);
      
          pingSonar();
    
          updateLedStatus(mappedFrontMinDistance, mappedLeftMinDistance, mappedRightMinDistance);
    }
}

void updateLedStatus(unsigned int frontMinDistance, unsigned int leftMinDistance, unsigned int rightMinDistance){
    if (frontDistance < frontMinDistance){
        digitalWrite(FRONT_LED_PIN, HIGH);
    } else {
        digitalWrite(FRONT_LED_PIN, LOW);
    }
  
    if (leftDistance < leftMinDistance){
        digitalWrite(LEFT_LED_PIN, HIGH);
    } else {
        digitalWrite(LEFT_LED_PIN, LOW);
    }
  
    if (rightDistance < rightMinDistance){
        digitalWrite(RIGHT_LED_PIN, HIGH);
    } else {
        digitalWrite(RIGHT_LED_PIN, LOW);
    }  
}

void forward(bool back){
    if (back) {
      velocity = minSpeed;
    }
  
    // Update direction only if previous direction has changed
    if (!((back && prevDirection == 'B') || (!back && prevDirection == 'F')) || prevDirection == 'S'){
      digitalWrite(MOTOR_RF_DIR_PIN1, back ? HIGH : LOW);
      digitalWrite(MOTOR_RF_DIR_PIN2, back ? LOW : HIGH);
      
      digitalWrite(MOTOR_LF_DIR_PIN1, back ? HIGH : LOW);
      digitalWrite(MOTOR_LF_DIR_PIN2, back ? LOW : HIGH);
      
      digitalWrite(MOTOR_RB_DIR_PIN1, back ? HIGH : LOW);
      digitalWrite(MOTOR_RB_DIR_PIN2, back ? LOW : HIGH);
      
      digitalWrite(MOTOR_LB_DIR_PIN1, back ? HIGH : LOW);
      digitalWrite(MOTOR_LB_DIR_PIN2, back ? LOW : HIGH); 
    }
  
    // Set speed
    analogWrite(MOTOR_RF_ENABLE_PIN, (int)(velocity * 1.1));
    analogWrite(MOTOR_RB_ENABLE_PIN, (int)(velocity));
    analogWrite(MOTOR_LF_ENABLE_PIN, (int)(velocity));
    analogWrite(MOTOR_LB_ENABLE_PIN, (int)(velocity));
  
    prevDirection = back ? 'B' : 'F';
}

void turn(bool left){
    // Update direction only if previous direction has changed
    if (!((left && prevDirection == 'L') || (!left && prevDirection == 'R')) || prevDirection == 'S'){
      digitalWrite(MOTOR_RF_DIR_PIN1, left ? LOW : HIGH);
      digitalWrite(MOTOR_RF_DIR_PIN2, left ? HIGH : LOW);
      digitalWrite(MOTOR_RB_DIR_PIN1, left ? LOW : HIGH);
      digitalWrite(MOTOR_RB_DIR_PIN2, left ? HIGH : LOW);
      
      digitalWrite(MOTOR_LF_DIR_PIN1, left ? HIGH : LOW);
      digitalWrite(MOTOR_LF_DIR_PIN2, left ? LOW : HIGH);
      digitalWrite(MOTOR_LB_DIR_PIN1, left ? HIGH : LOW);
      digitalWrite(MOTOR_LB_DIR_PIN2, left ? LOW : HIGH);
  
      // Set speed
      analogWrite(MOTOR_RF_ENABLE_PIN, turnSpeed);
      analogWrite(MOTOR_RB_ENABLE_PIN, turnSpeed);
      analogWrite(MOTOR_LF_ENABLE_PIN, turnSpeed);
      analogWrite(MOTOR_LB_ENABLE_PIN, turnSpeed);
    } 
  
    prevDirection = left ? 'L' : 'R';
}

void brake(){
    analogWrite(MOTOR_RF_ENABLE_PIN, 0);
    analogWrite(MOTOR_RB_ENABLE_PIN, 0);
    analogWrite(MOTOR_LF_ENABLE_PIN, 0);
    analogWrite(MOTOR_LB_ENABLE_PIN, 0);
      
    digitalWrite(MOTOR_RF_DIR_PIN1, LOW);
    digitalWrite(MOTOR_LF_DIR_PIN1, LOW);
    digitalWrite(MOTOR_RF_DIR_PIN2, LOW);
    digitalWrite(MOTOR_LF_DIR_PIN2, LOW);
    digitalWrite(MOTOR_RB_DIR_PIN1, LOW);
    digitalWrite(MOTOR_LB_DIR_PIN1, LOW);
    digitalWrite(MOTOR_RB_DIR_PIN2, LOW);
    digitalWrite(MOTOR_LB_DIR_PIN2, LOW);
    
    prevDirection = 'S';
}

void printInfo(){
    unsigned long currentMillis = millis();
    
    if (currentMillis -  printInfoPrevMillis >= printInfoInterval) {
      printInfoPrevMillis = currentMillis;
      
      Serial.print("V: ");
      Serial.print(velocity);
      Serial.print("\t"); 
      Serial.print("PV: ");
      Serial.print(prevVelocity);
      Serial.print("\t"); 
      Serial.print("Auto: ");
      Serial.print(isAutoMode);
      Serial.print("\t");
      Serial.print("SD: ");
      Serial.println(prevSerialDataValue);      
    
      Serial.print("L: ");
      Serial.print(leftDistance);
      Serial.print("\t"); 
      Serial.print("F: ");
      Serial.print(frontDistance);
      Serial.print("\t"); 
      Serial.print("R: ");
      Serial.print(rightDistance);
      Serial.print("\t"); 
      Serial.print("SP: ");
      Serial.println(sonarPingTime);   

      int c1 = digitalRead(CONTROL_PIN_1);
      int c2 = digitalRead(CONTROL_PIN_2);
      int c3 = digitalRead(CONTROL_PIN_3);  
    
      Serial.print("C1: ");
      Serial.print(c1);
      Serial.print("\t"); 
      Serial.print("C2: ");
      Serial.print(c2);
      Serial.print("\t"); 
      Serial.print("C3: ");
      Serial.println(c3);
    }   
}