#include <Arduino.h>
#include <NewPing.h>

void findWay(unsigned int frontDistance, unsigned int leftDistance, unsigned int rightDistance);
void brake();
void turn(bool left);
void forward(bool back);

// Right front motor
int MOTOR_RF_ENABLE_PIN   = 2;
int MOTOR_RF_DIR_PIN1     = 30;
int MOTOR_RF_DIR_PIN2     = 31;

// Right back motor
int MOTOR_RB_ENABLE_PIN   = 3;
int MOTOR_RB_DIR_PIN1     = 32;
int MOTOR_RB_DIR_PIN2     = 33;

// Left front motor
int MOTOR_LF_ENABLE_PIN   = 4;
int MOTOR_LF_DIR_PIN1     = 34;
int MOTOR_LF_DIR_PIN2     = 35;

// Left back motor
int MOTOR_LB_ENABLE_PIN   = 5;
int MOTOR_LB_DIR_PIN1     = 36;
int MOTOR_LB_DIR_PIN2     = 37;

int LEFT_ECHO_PIN         = 38;
int LEFT_TRIGGER_PIN      = 39;

int FRONT_ECHO_PIN        = 40;
int FRONT_TRIGGER_PIN     = 41;

int RIGHT_ECHO_PIN        = 42;
int RIGHT_TRIGGER_PIN     = 43;

#define TURN_SPEED          75
#define MIN_SPEED           50
#define MAX_SPEED           120
#define MAX_DISTANCE        200
#define FRONT_MIN_DISTANCE  15
#define FRONT_MAX_DISTANCE  30
#define LEFT_MIN_DISTANCE   10
#define LEFT_MAX_DISTANCE   20
#define RIGHT_MIN_DISTANCE  10
#define RIGHT_MAX_DISTANCE  20
#define ACCELERATION_RATE   5

NewPing l_sonar(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing f_sonar(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing r_sonar(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

unsigned long bootupPrevMillis      = 0;
unsigned long bootupInterval        = 7500;

unsigned long leftSonarPrevMillis   = 0;
unsigned long frontSonarPrevMillis  = 0;
unsigned long rightSonarPrevMillis  = 0;

unsigned long accelerationPrevMillis  = 0;
const long accelerationInterval       = 1000;

const long leftSonarInterval        = 70;
const long frontSonarInterval       = 60;
const long rightSonarInterval       = 70;

unsigned int leftDistance           = 0;
unsigned int frontDistance          = 0;
unsigned int rightDistance          = 0;

unsigned int prevVelocity         = MIN_SPEED;
unsigned int velocity             = MIN_SPEED;

void setup(){
  Serial.begin(9600);

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

  brake();
}

void loop(){
  unsigned long currentMillis = millis();

  unsigned int minDistance = 65535;
  
  if (currentMillis - bootupPrevMillis >= bootupInterval) {
    if (currentMillis - frontSonarPrevMillis >= frontSonarInterval) {
      frontSonarPrevMillis = currentMillis;
  
      frontDistance = f_sonar.ping_cm();
  
      if (frontDistance == 0 || frontDistance > MAX_DISTANCE){
        frontDistance = MAX_DISTANCE;
      }
  
      if (frontDistance < minDistance){
        minDistance = frontDistance;
      }
    }
    
    if (currentMillis - leftSonarPrevMillis >= leftSonarInterval) {
      leftSonarPrevMillis = currentMillis;
  
      leftDistance = l_sonar.ping_cm();
  
      if (leftDistance == 0 || leftDistance > MAX_DISTANCE){
        leftDistance = MAX_DISTANCE;
      }
  
      if (leftDistance < minDistance){
        minDistance = leftDistance;
      }    
    }
  
    if (currentMillis - rightSonarPrevMillis >= rightSonarInterval) {
      rightSonarPrevMillis = currentMillis;
  
      rightDistance = r_sonar.ping_cm();
  
      if (rightDistance == 0 || rightDistance > MAX_DISTANCE){
        rightDistance = MAX_DISTANCE;
      }
  
      if (rightDistance < minDistance){
        minDistance = rightDistance;
      }   
    }

    // Set velocity according to calculated distances (left, front, right)
    velocity = map(minDistance, 0, MAX_DISTANCE, MIN_SPEED, MAX_SPEED);

    // This is for accelaration interval check
    currentMillis = millis();

    // Do we need to accelerate?
    if (currentMillis - accelerationPrevMillis >= accelerationInterval) {
      if (prevVelocity < velocity){
        accelerationPrevMillis = currentMillis;

        // Accelerate 
        velocity = prevVelocity + ((prevVelocity * ACCELERATION_RATE) / 100);
      }
    }

    // Check min, max speed thresholds
    if (velocity > MAX_SPEED){
      velocity = MAX_SPEED;
    } else if (velocity < MIN_SPEED){
      velocity = MIN_SPEED;
    }

    // Set min distances according to calculated velocity
    unsigned int frontMinDistance = map(velocity, MIN_SPEED, MAX_SPEED, FRONT_MIN_DISTANCE, FRONT_MAX_DISTANCE);
    unsigned int leftMinDistance = map(velocity, MIN_SPEED, MAX_SPEED, LEFT_MIN_DISTANCE, LEFT_MAX_DISTANCE);
    unsigned int rightMinDistance = map(velocity, MIN_SPEED, MAX_SPEED, RIGHT_MIN_DISTANCE, RIGHT_MAX_DISTANCE);
     
    if (frontDistance < frontMinDistance || leftDistance < leftMinDistance || rightDistance < rightMinDistance){
      findWay(frontDistance, leftDistance, rightDistance);
    } else {
      forward(false);
    }

    prevVelocity = velocity;
  } else {
    // bootup code here
    Serial.println("Booting up...");
  }
}

void findWay(unsigned int frontDistance, unsigned int leftDistance, unsigned int rightDistance){
  // Stop first
  brake();
  
  delay(150);

  // Go a little back (so wee need to consider sonra distances)
  forward(true);

  delay(500);

  int fDistance = frontDistance;
  int lDistance = leftDistance;
  int rDistance = rightDistance;

  // Since we have gone a little back, there should difference between the original distances and the
  // current ones. Thus consider them while finding a way out!
  int fDistanceDiff = 0;
  int lDistanceDiff = 0;
  int rDistanceDiff = 0;
  bool first = true;
  while (fDistance < FRONT_MIN_DISTANCE + fDistanceDiff || 
         lDistance < LEFT_MIN_DISTANCE + lDistanceDiff ||
         rDistance < RIGHT_MIN_DISTANCE + rDistanceDiff ){
    turn(lDistance > rDistance);
    
    delay(75);

    fDistance = f_sonar.ping_cm();
    if (fDistance == 0 || fDistance > MAX_DISTANCE){
      fDistance = MAX_DISTANCE;
    }
      
    lDistance = l_sonar.ping_cm();
    if (lDistance == 0 || lDistance > MAX_DISTANCE){
      lDistance = MAX_DISTANCE;
    }
    
    rDistance = r_sonar.ping_cm();
    if (rDistance == 0 || rDistance > MAX_DISTANCE){
      rDistance = MAX_DISTANCE;
    }

    if (first){
      fDistanceDiff = fDistance - frontDistance;
      if (fDistanceDiff < 0){
        fDistanceDiff = 0;
      }

      lDistanceDiff = lDistance - leftDistance;
      if (lDistanceDiff < 0){
        lDistanceDiff = 0;
      }

      rDistanceDiff = rDistance - rightDistance;
      if (rDistanceDiff < 0){
        rDistanceDiff = 0;
      }
    
      first = false;
    }

    delay(75);
  }
}

void forward(bool back){
  if (back) {
    velocity = MIN_SPEED;
  }
  
  Serial.print("V: ");
  Serial.println(velocity);
      
  digitalWrite(MOTOR_RF_DIR_PIN1, back ? HIGH : LOW);
  digitalWrite(MOTOR_LF_DIR_PIN1, back ? HIGH : LOW);
  digitalWrite(MOTOR_RF_DIR_PIN2, back ? LOW : HIGH);
  digitalWrite(MOTOR_LF_DIR_PIN2, back ? LOW : HIGH);
  digitalWrite(MOTOR_RB_DIR_PIN1, back ? HIGH : LOW);
  digitalWrite(MOTOR_LB_DIR_PIN1, back ? HIGH : LOW);
  digitalWrite(MOTOR_RB_DIR_PIN2, back ? LOW : HIGH);
  digitalWrite(MOTOR_LB_DIR_PIN2, back ? LOW : HIGH);

  analogWrite(MOTOR_RF_ENABLE_PIN, (int)(velocity));
  analogWrite(MOTOR_RB_ENABLE_PIN, (int)(velocity));
  analogWrite(MOTOR_LF_ENABLE_PIN, (int)(velocity));
  analogWrite(MOTOR_LB_ENABLE_PIN, (int)(velocity));
}

void turn(bool left){
  analogWrite(MOTOR_RF_ENABLE_PIN, TURN_SPEED);
  analogWrite(MOTOR_RB_ENABLE_PIN, TURN_SPEED);
  analogWrite(MOTOR_LF_ENABLE_PIN, TURN_SPEED);
  analogWrite(MOTOR_LB_ENABLE_PIN, TURN_SPEED);

  delay(50);

  digitalWrite(MOTOR_RF_DIR_PIN1, left ? LOW : HIGH);
  digitalWrite(MOTOR_RF_DIR_PIN2, left ? HIGH : LOW);
  digitalWrite(MOTOR_RB_DIR_PIN1, left ? LOW : HIGH);
  digitalWrite(MOTOR_RB_DIR_PIN2, left ? HIGH : LOW);
  digitalWrite(MOTOR_LF_DIR_PIN1, left ? HIGH : LOW);
  digitalWrite(MOTOR_LF_DIR_PIN2, left ? LOW : HIGH);
  digitalWrite(MOTOR_LB_DIR_PIN1, left ? HIGH : LOW);
  digitalWrite(MOTOR_LB_DIR_PIN2, left ? LOW : HIGH);
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
  
  delay(20);
}
