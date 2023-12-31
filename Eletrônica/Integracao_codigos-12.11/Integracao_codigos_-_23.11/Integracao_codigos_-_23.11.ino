#include <Wire.h>
#include <stdio.h>
#include <MPU6050_tockn.h>
#include <QTRSensors.h>
#define TRIG_PIN_LEFT 17
#define ECHO_PIN_LEFT 16
#define TRIG_PIN_RIGHT 02
#define ECHO_PIN_RIGHT 15
#define TRIG_PIN_FRONT 18
#define ECHO_PIN_FRONT 05
#define MOTOR_IN_A1 32 // RIGHT
#define MOTOR_IN_A2 33 // RIGHT
#define MOTOR_IN_B1 12 // LEFT
#define MOTOR_IN_B2 27 // LEFT
#define MOTOR_MAX_SPEED 3.1415
#define NUMBER_OF_SAMPLES 15
#define LINHA_1 13
#define LINHA_3 14
#define LINHA_6 26
#define LINHA_8 25
#define HC_SR04_DISTANCE_CONVERSION_FACTOR 17
#define MOTOR_TEST
#define LED_BUILTIN  2

QTRSensors qtr;
MPU6050 mpu6050(Wire);
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

struct Motor {
  int pin1;
  int pin2;
  float vcc;
  float maxSpeed;
  
  Motor(int pPin1, int pPin2, float pMaxSpeed) {
    pin1 = pPin1;
    pin2 = pPin2;
    maxSpeed = pMaxSpeed;
  }

  void setupMotor() {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }

  void setMotorSpeed(float vel);
};

void Motor::setMotorSpeed(float vel) {
  //Serial.print("Vel motor: ");
  //Serial.println(vel);
  float velocity = vel / maxSpeed;
  bool forward = velocity > 0;
  if (abs(velocity) > 1) velocity = 1;
  int pwmOUT = round(abs(velocity) * 255);
  if (forward) {  //forward
    analogWrite(pin1, pwmOUT);      
    analogWrite(pin2, 0);
  } else {  // reverse
    analogWrite(pin1, 0);
    analogWrite(pin2, pwmOUT);
  }
}

struct SensorDistance{
  int pinTrigger;
  int pinEcho;
  SensorDistance(int p_pinTrigger, int p_pinEcho){
    pinTrigger = p_pinTrigger;
    pinEcho = p_pinEcho;
  }
  void setup(){
    pinMode(pinTrigger, OUTPUT);
    pinMode(pinEcho, INPUT);
  }
  float readOneSample(){
    //send trigger
    digitalWrite(pinTrigger, LOW);
    delayMicroseconds(2);
    
    digitalWrite(pinTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigger, LOW);
    
    long duration = pulseIn(pinEcho, HIGH);
    //adjust and return distance read.
    return duration * HC_SR04_DISTANCE_CONVERSION_FACTOR/1000.0; 
  }
  float readSampleAverageDist(int numberOfSamplesToRead){
    float distAvg = 0;
    //Accumulate distances read
    for (int i = 0; i < numberOfSamplesToRead; i++)
      distAvg += readOneSample();
    //return the average
    if (distAvg/numberOfSamplesToRead >= 3 && distAvg/numberOfSamplesToRead <= 40){
      return distAvg/numberOfSamplesToRead;
    }else{
      if (distAvg/numberOfSamplesToRead < 3){
        return 3;
      }
      if (distAvg/numberOfSamplesToRead > 40){
        return 40;
      }
    }
  }
};

Motor MotorA(MOTOR_IN_A1, MOTOR_IN_A2, MOTOR_MAX_SPEED); // RIGHT
Motor MotorB(MOTOR_IN_B1, MOTOR_IN_B2, MOTOR_MAX_SPEED); // LEFT
SensorDistance sensorRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT); 
SensorDistance sensorLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
SensorDistance sensorFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

float getAngle(){
  mpu6050.update();
  float angleZ = mpu6050.getAngleZ();
  return(angleZ*M_PI/180);
}
 
int sensorLinha(){
  qtr.readLineBlack(sensorValues);
  /*
  Serial.print(sensorValues[0]);
  Serial.print('\t');
  Serial.print(sensorValues[5]);
  Serial.print('\t');
  Serial.print(sensorValues[7]);
  Serial.print('\t');
  Serial.println();
  */
  if (sensorValues[0] > 950 || sensorValues[5] >950 || sensorValues[7] > 950){
    return 1;
  }else{
    return 0;
  }
}

void linhaSetup(){
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){LINHA_1, 0, 0, 0, 0, LINHA_6, 0, LINHA_8}, SensorCount);
  qtr.setEmitterPin(14);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are done with calibration
}

void setup() {
  Serial.begin(9600);
  //linhaSetup();
  //delay(5000);
  
  sensorLeft.setup();
  sensorRight.setup();
  sensorFront.setup();
  
  MotorA.setupMotor();
  Serial.println("Primeiro");
  delay(500);
  MotorB.setupMotor();
  Serial.println("Segundo");
  delay(500);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);        
}


unsigned long initialTime = millis();
unsigned long finalTime = millis();
const int CURVE = 0;
const int RUN = 1;
const int ANTICOLISION = 2;
//const float M_PI 3.141592653589793238462643;

float t = 0.0, turning_delay=99.0, identified_curve_time=0.0;
float max_speed = 8.37758;
float base_speed = 0.0;
float right_speed=0.0, left_speed=0.0;
float error = 0.0, sum_error=0.0, error_sensors=0.0, sum_error_sensors=0.0;
float Kp=15.0, Ki=0.0;
float proportional=0.0, integral=0.0;
float new_direction = 0.0;
int step = CURVE, next_step=CURVE; //0->Virando, 1->Indo pra frente por X seg, 2->Anticolisão (linha reta)

float dsValues[3] = {0.0, 0.0, 0.0};//em centímetros
float orientation = 0.0;//orientação do robo no plano z
float offset = 0;//pra corrigir o giroscopio
bool done = false;//caso chegou no final do labirinto
float target = 0, previous_target=0.0;//previous não usando pra nada ainda nesse código
float run_delay = 1.0;
int countCurve = 0;


void consolePrint(double t_, double turning_delay_, double identified_curve_time_, double step_, double next_step_, double new_direction_, double error_, double error_sensors_, double Kp_){
    Serial.println("-----------------------------------------------------------------");
    Serial.print("t: " + (String)t_ + ", t_delay: " + (String)turning_delay_ + ", t_id: " + (String)identified_curve_time_);
    if(step_ == CURVE){
      Serial.print(", step: CURVE");
    }else if(step_ == RUN){
      Serial.print(", step: RUN");
    }else if(step_ == ANTICOLISION){
      Serial.print(", step: ANTICOLISION");
    }
    if(next_step_ == CURVE){
      Serial.println(", next_step: CURVE");
    }else if(next_step_ == RUN){
      Serial.println(", next_step: RUN");
    }else if(next_step_ == ANTICOLISION){
      Serial.println(", next_step: ANTICOLISION");
    }
    Serial.println("Target: " + (String)target + ", P_target: " + (String)previous_target + " rad, orientation: " + (String)(orientation*180/M_PI) + " rad, new_dire: " + (String)(new_direction_*180/M_PI) + " rad");
    Serial.println("s1: " + (String)dsValues[0] + ", s2: " + (String)dsValues[1] + ", s3: " + (String)dsValues[2]);
    Serial.println("dif_angle: " + (String)abs(previous_target - target) + ", error_gyro: " + (String)error_ + ", dif_sensors: " + (String)error_sensors_ + ", Kp: " + (String)Kp_);
    Serial.println("left_speed: " + (String)left_speed + ", right_speed: " + (String)right_speed);
    
    }

float vel_max_curve=0.0;

void loop(){

  
  Serial.println("condição inicial1: ");

  while(!done){
    initialTime = millis();
    //fazendo a leitura nos sensores
    dsValues[0] = sensorLeft.readSampleAverageDist(3);
    dsValues[1] = sensorFront.readSampleAverageDist(3);
    dsValues[2] = sensorRight.readSampleAverageDist(3);
    //codicão inicial
    
    if(t == 0){
      step = ANTICOLISION;
      Serial.print("condição inicial: ");
      Serial.println(target);
      //target = selectDirection(target);
      Serial.print("target após selectDirection: ");
      Serial.println(target);
      consolePrint(t, turning_delay, identified_curve_time, step, next_step, new_direction, error, error_sensors, Kp);
    }

    //estados do robo
    switch (step) {
      case RUN:
        Serial.println("Inicio Run:");
        consolePrint(t, turning_delay, identified_curve_time, step, next_step, new_direction, error, error_sensors, Kp); 
        if(next_step == CURVE){
          run_delay = 0.95;
        }else{
          run_delay = 0.85;
        }
        
        if(turning_delay > run_delay){
          identified_curve_time = t;
          step = next_step;
          if (step = CURVE){
            countCurve = 0;
          }
        }
        Kp = 0.0;
        Ki = 0.0;
        
        if(dsValues[1] <= 4.0){
          base_speed = 0.0*max_speed;
        }else{     
          base_speed = 0.35*max_speed;
        }
        right_speed = base_speed;
        left_speed = base_speed;
        break;
      case CURVE:
        if(countCurve == 0){
          offset = getAngle();
          countCurve = 1;
        }
        orientation = getAngle() - offset;
        //corrigindo erro pra não ter mudanças bruscas
        error = target - orientation;
        if (error > M_PI) {
            error -= 2 * M_PI;
        } else if (error < -M_PI) {
            error += 2 * M_PI;
        }
                  
        Kp=8.0;
        Ki=0.0;
        base_speed = 0.0*max_speed;
        
        // Cálculo do erro integrador
        sum_error += error;
        integral = Ki * sum_error;
        
        proportional = Kp * error;
        right_speed = base_speed + proportional + integral;
        left_speed = base_speed - proportional - integral; 
        next_step = ANTICOLISION;

        //limitando velocidade em 0.45
        
        vel_max_curve=max_speed*0.6;
        if(left_speed > vel_max_curve){
          left_speed = vel_max_curve;
        }
        if(left_speed < -vel_max_curve){
          left_speed = -vel_max_curve;
        }
        if(right_speed > vel_max_curve){
          right_speed   = vel_max_curve;
        }
        if(right_speed < -vel_max_curve){
          right_speed = -vel_max_curve;
        }
        if(abs(error) < 0.2){
          step = RUN;
          turning_delay = 0;
          
          orientation=0;
          target = 0;//pois o angulo do giroscopio é resetado
          new_direction = 0;//-------------------- isso muda algo?
        }
        
        break;
      case ANTICOLISION:
        Kp = 0.4;
        Ki=0.0;
        base_speed = 0.3*max_speed;
        error_sensors = dsValues[0] - (22.3-10.5)/2; //7 == (distancia de labirinto - largura do robo) / 2
        
        // Cálculo do erro integrador
        sum_error_sensors += error_sensors;
        integral = Ki * sum_error_sensors;
        proportional = Kp * error_sensors;
        
        right_speed = base_speed + proportional + integral;
        left_speed = base_speed - proportional - integral; 
        
        vel_max_curve=max_speed*0.35;
        if(left_speed > vel_max_curve){
          left_speed = vel_max_curve;
        }
        if(left_speed < -vel_max_curve){
          left_speed = -vel_max_curve;
        }
        if(right_speed > vel_max_curve){
          right_speed   = vel_max_curve;
        }
        if(right_speed < -vel_max_curve){
          right_speed = -vel_max_curve;
        }


        if(dsValues[0] >= 15){
          new_direction = M_PI/2;
        }else if(dsValues[1] >= 15){
          new_direction = 0;
        }else if(dsValues[2] >= 15){
          new_direction = -M_PI/2;    
        }else{
          new_direction = M_PI;
        }
        
        next_step = CURVE;
        if(target != new_direction){
          turning_delay = 0;
          step = RUN;
          target = new_direction;
          right_speed = base_speed;
          left_speed = base_speed; 
        }   
        break;
      default:
      Serial.println("Estado: default");
        break;
    }
    
    //limitando a velocidade
    if(left_speed > max_speed){
      left_speed = max_speed;
    }
    if(left_speed < -max_speed){
      left_speed = -max_speed;
    }
    if(right_speed > max_speed){
      right_speed   = max_speed;
    }
    if(right_speed < -max_speed){
      right_speed = -max_speed;
    }

    //transformando os valores pra porcentagem
    right_speed = right_speed/max_speed;
    left_speed = left_speed/max_speed;


    //Passa os valores para os motores 
    MotorA.setMotorSpeed(right_speed*MOTOR_MAX_SPEED);
    MotorB.setMotorSpeed(left_speed*MOTOR_MAX_SPEED);

    
    consolePrint(t, turning_delay, identified_curve_time, step, next_step, new_direction, error, error_sensors, Kp);   

    finalTime = millis();
    t += (float)(finalTime - initialTime) / 1000.0;
    turning_delay += (float)(finalTime - initialTime)  / 1000.0;
    identified_curve_time += (float)(finalTime - initialTime) / 1000.0;

    //done = sensorLinha();
    
  }


}
