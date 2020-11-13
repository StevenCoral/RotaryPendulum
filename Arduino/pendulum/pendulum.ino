#include <Encoder.h>
//#include <Math.h>

#define DT 0.005
#define MOTOR_PIN 3
#define DIR_PIN 12
#define SUPPLY_VOLTAGE 5

#define MOTOR_GEAR 5.75
#define POSITIVE_DEADZONE 70
#define NEGATIVE_DEADZONE 70

#define PEND_LENGTH 0.129
#define PEND_MASS 0.024
#define SWING_COEFF 100

#define MOTOR_CPR 500
#define MOTOR_ENC_A 19
#define MOTOR_ENC_B 18

#define PEND_CPR 512
#define PEND_ENC_A 20
#define PEND_ENC_B 21



class EncVel
{
  public:
  float dt;
  float velocityRaw=0, velocityFiltered=0;
  float positionPrevious=0, positionRadians=0;
  float encToRadGear;  
  float filter=0.9;
  
  EncVel(int baseCPR, float gearRatio, float deltaTime);
  void computeVelocity(long encoderPos);
};

EncVel::EncVel(int baseCPR, float gearRatio, float deltaTime)
{
  encToRadGear = TWO_PI/float(baseCPR*4)/gearRatio;
  dt = deltaTime;
}

void EncVel::computeVelocity(long encoderPos)
{
  positionRadians = encoderPos * encToRadGear;
  velocityRaw = (positionRadians-positionPrevious)/dt;
  velocityFiltered = velocityFiltered * filter + velocityRaw * (1-filter);
  if (abs(velocityFiltered)<0.001) 
    velocityFiltered = 0;
  positionPrevious = positionRadians;
}

inline int sign(float argument)
{
  if (argument<0) return (-1);
  else return 1;
}


Encoder motorEnc(MOTOR_ENC_A, MOTOR_ENC_B);
Encoder pendulumEnc(PEND_ENC_A, PEND_ENC_B);

EncVel motorVel(MOTOR_CPR, MOTOR_GEAR, DT);
EncVel pendulumVel(PEND_CPR, 1, DT);

// State vector is: [theta, alpha, thetaDot, alphaDot]
const float controlGain[4] = {-0.6325 ,  13.4865  , -0.7251   , 1.3025};  //{-0.7071  , 14.8844 ,  -0.7914  ,  1.4432};
const long dtMicros = DT*1e6;
const float pendulumMOI = PEND_MASS * PEND_LENGTH*PEND_LENGTH / 12;  // Moment of inertia around its edge
const float energyReference = PEND_MASS * PEND_LENGTH * 9.81;
unsigned long now = micros();
int printCount=1, maxOutput=0;
bool stateTransition = true, shouldPrint = false;
float voltageToPWM = 255.0 / SUPPLY_VOLTAGE;
float outputVoltage = 0;

void setup() {
  TCCR3B = TCCR3B & B11111000 | B00000001;
  Serial.begin(57600);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  
  // HIGH = forward LOW = Backwards (when motor cable exposed part is facing inwards from the motor shield):
  digitalWrite(DIR_PIN, HIGH);
  pendulumEnc.write(PEND_CPR*2);  // 180 degees down, must begin at rest!
  now = micros();
}

void loop() {

  if (micros()-now > dtMicros)
  {
    unsigned long tic = micros();
        
    long motorPos = motorEnc.read();
    motorVel.computeVelocity(motorPos);

    long pendulumPos = pendulumEnc.read();
    pendulumVel.computeVelocity(pendulumPos);
    float pendulumPosFixed = pendulumVel.positionRadians;
    if (pendulumVel.positionRadians > PI)
      pendulumPosFixed -= TWO_PI;  // Try to fix multiple circles!
    if (pendulumVel.positionRadians < -PI)
      pendulumPosFixed += TWO_PI;

    float energyLevel = pendulumMOI * pendulumVel.velocityFiltered * pendulumVel.velocityFiltered / 2 + 
                        PEND_MASS * PEND_LENGTH * 9.81 * (1 - cos(pendulumPosFixed-PI)) / 2;

    float energyError = energyReference - energyLevel;

    if (abs(pendulumPosFixed) < 0.35)  // 0.35 rads ~ 20 deg
    {
      if (stateTransition)
      {
        motorVel.velocityFiltered = 0;
        motorVel.positionPrevious = 0;
        motorEnc.write(0);
        pendulumVel.velocityFiltered = 0;
        stateTransition = false;
      }
      // u = K * y
      outputVoltage = controlGain[0]*motorVel.positionRadians + 
                      controlGain[1]*pendulumPosFixed + 
                      controlGain[2]*motorVel.velocityFiltered + 
                      controlGain[3]*pendulumVel.velocityFiltered ;
      maxOutput = 255;
    }
    else
    {
      outputVoltage = -SWING_COEFF * energyError * sign(pendulumVel.velocityFiltered * cos(pendulumPosFixed));
      maxOutput = 140;
      stateTransition = true;
    }

    int controlOutput = outputVoltage*voltageToPWM;    

    if (controlOutput > 1)
    {
      digitalWrite(DIR_PIN, HIGH);
      controlOutput += POSITIVE_DEADZONE;
    }           
    if (controlOutput < -1)
    {
      digitalWrite(DIR_PIN, LOW);
      controlOutput -= NEGATIVE_DEADZONE;
    }
    
    controlOutput = constrain(controlOutput, -maxOutput, maxOutput);
    analogWrite(MOTOR_PIN, abs(controlOutput));
    
    if (shouldPrint && printCount > 10)
    {
//      Serial.print("theta: ");
//      Serial.print(motorVel.positionRadians, 2);
//      Serial.print("  alpha: ");
//      Serial.print(pendulumPosFixed, 2);
            
//      Serial.print("  thetaDot: ");
//      Serial.print(motorVel.velocityFiltered, 2);
//      Serial.print("  alphaDot: ");
//      Serial.print(pendulumVel.velocityFiltered, 2);

//      Serial.print("  energy: ");
//      Serial.print(energyLevel, 4);
//      Serial.print("  energyError: ");
//      Serial.print(energyError, 4);
//      Serial.print("  sign: ");
//      Serial.print(sign(pendulumVel.velocityFiltered * cos(pendulumPosFixed)));
      
//      Serial.print("  voltage: ");
//      Serial.print(outputVoltage);
//      Serial.print("  pwm: ");
//      Serial.println(controlOutput);      
      printCount=1;
    }
    else printCount++;
    now = micros();
  }  
}
