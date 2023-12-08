#include <Wire.h>
#include <stdio.h>
#include <MPU6050_tockn.h>
#include <QTRSensors.h>
#include <ShiftRegister74HC595.h>
#define TRIG_PIN_LEFT 18
#define ECHO_PIN_LEFT 05
#define TRIG_PIN_RIGHT 02
#define ECHO_PIN_RIGHT 15
#define TRIG_PIN_FRONT 17
#define ECHO_PIN_FRONT 16
#define MOTOR_IN_A1 33 // RIGHT
#define MOTOR_IN_A2 32 // RIGHT
#define MOTOR_CHANNEL_A 0
#define MOTOR_IN_B1 27 // LEFT
#define MOTOR_IN_B2 12 // LEFT
#define MOTOR_CHANNEL_B 1

#define NUMBER_OF_SAMPLES 15
#define LINHA_IR 34
#define LINHA_1 13 // marrom
//#define LINHA_3 34
#define LINHA_6 14 // Laranja
#define LINHA_8 35 // Amarelo
#define DISPLAY_1 26
#define DISPLAY_2 04
#define DISPLAY_3 25
#define HC_SR04_DISTANCE_CONVERSION_FACTOR 17
#define MOTOR_TEST
#define SDI 25
#define SCLK 4
#define LOAD 26
#define DIGITS 1
#define LED_BUILTIN  2

// create shift register object (number of shift registers, data pin, clock pin, latch pin)
ShiftRegister74HC595<2> sr (SDI, SCLK, LOAD); 
QTRSensors qtr;
MPU6050 mpu6050(Wire);
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int value,digit1,digit2,digit3,digit4; 
uint8_t  digits[] = {B11000000, //0
                      B11111001, //1 
                      B10100100, //2
                      B10110000, //3 
                      B10011001, //4
                      B10010010, //5
                      B10000010, //6 
                      B11111000, //7
                      B10000000, //8
                      B10010000  //9
                     };

void showNumber(int num)
{
    digit2=num % 10 ;
    digit1=(num / 10) % 10 ;
    //Send them to 7 segment displays
    uint8_t numberToPrint[]= {digits[digit2],digits[digit1]};
    sr.setAll(numberToPrint);  
}

struct Motor {
  int pin1;
  int pin2;
  int channel;
  float vcc;
  
  Motor(int pPin1, int pPin2, int pChannel) {
    pin1 = pPin1;
    pin2 = pPin2;
    channel = pChannel;
  }

  void setupMotor() {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }

  void setMotorSpeed(float vel);
};

void Motor::setMotorSpeed(float velocity) {
  if(velocity == 0){
    analogWrite(pin1, 0);      
    analogWrite(pin2, 0);
  }else{
    bool forward = velocity > 0;
    if (abs(velocity) > 1) velocity = 1;
    //int pwmOUT = round(abs(velocity) * 255);
    int pwmOUT = round(abs(velocity) * (255-75)) + 75;
    if (forward) {  //forward
      analogWrite(pin1, pwmOUT);      
      analogWrite(pin2, 0);
    } else {  // reverse
      analogWrite(pin1, 0);
      analogWrite(pin2, pwmOUT);
    }
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

Motor MotorA(MOTOR_IN_A1, MOTOR_IN_A2, MOTOR_CHANNEL_A); // RIGHT
Motor MotorB(MOTOR_IN_B1, MOTOR_IN_B2, MOTOR_CHANNEL_B); // LEFT
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
  if (sensorValues[0] > 950 || sensorValues[5] >950 || sensorValues[7] > 950){
    return true;
  }else{
    return false;
  }
}

void linhaSetup(){
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){LINHA_1, 0, 0, 0, 0, LINHA_6, 0, LINHA_8}, SensorCount);
  qtr.setEmitterPin(LINHA_IR);
  delay(50);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  for (uint16_t i = 0; i < 20; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are done with calibration
}

void setup() {
  Serial.begin(9600);
  //linhaSetup();
  //delay(10000);
  showNumber(99);
  sensorLeft.setup();
  sensorRight.setup();
  sensorFront.setup();
  
  MotorA.setupMotor();
  delay(100);
  MotorB.setupMotor();

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);        
}


unsigned long initialTime = millis();
unsigned long finalTime = millis();
const int CURVE = 2;
const int RUN = 1;
const int ANTICOLISION = 0;
const float DISTANCE_FOR_CURVE = 18.0;
const bool isLeftHandle = true;


float t = 0.0, turning_delay=99.0, identified_curve_time=0.0;
float base_speed = 0.0;
float right_speed=0.0, left_speed=0.0;
float error = 0.0, sum_error=0.0, error_sensors=0.0, sum_error_sensors=0.0,previous_error=0.0;
float Kp=15.0, Ki=0.0, Kd = 0.0001;
float proportional=0.0, integral=0.0, derivativo=0.0;
float new_direction = 0.0;
int step = ANTICOLISION, next_step=CURVE; //0->Virando, 1->Indo pra frente por X seg, 2->Anticolisão (linha reta)

float dsValues[3] = {0.0, 0.0, 0.0};//em centímetros
float orientation = 0.0;//orientação do robo no plano z
float offset = 0;//pra corrigir o giroscopio
bool done = false;//caso chegou no final do labirinto
float target = 0, previous_target=0.0;//previous não usando pra nada ainda nesse código
float run_delay = 1.0;
int countCurve = 0, countRun = 0;
int count_anticolision=0;
//float delta = 0.01; 

void consolePrint(double t_, double turning_delay_, double identified_curve_time_, double step_, double next_step_, double new_direction_, double error_, double error_sensors_, double Kp_){
    //Serial.println("-----------------------------------------------------------------");
    Serial.print("t: " + (String)t_ /*+ ", t_delay: " + (String)turning_delay_ + ", t_id: " + (String)identified_curve_time_*/);
    
    if(step_ == CURVE){
      Serial.println(", step: CURVE");
    }else if(step_ == RUN){
      Serial.println(", step: RUN");
    }else if(step_ == ANTICOLISION){
      Serial.println(", step: ANTICOLISION");
    }
    /*
    if(next_step_ == CURVE){
      Serial.println(", next_step: CURVE");
    }else if(next_step_ == RUN){
      Serial.println(", next_step: RUN");
    }else if(next_step_ == ANTICOLISION){
      Serial.println(", next_step: ANTICOLISION");
    }
    */
    
    Serial.println(/*"Target: " + (String)target + " rad, orientation: " + (String)(orientation*180/M_PI) + */"New_dire: " + (String)(new_direction_*180/M_PI) + " rad");
    Serial.println("s1: " + (String)dsValues[0] + ", s2: " + (String)dsValues[1] + ", s3: " + (String)dsValues[2]);
    //Serial.println("dif_angle: " + (String)abs(previous_target - target) + ", error_gyro: " + (String)error_ + ", dif_sensors: " + (String)error_sensors_ + ", Kp: " + (String)Kp_);
    //Serial.println("left_speed: " + (String)left_speed + ", right_speed: " + (String)right_speed);
    
    }

float vel_max_curve=0.0;

void loop(){
  while(!done){
  
    initialTime = millis();
    //fazendo a leitura nos sensores
    dsValues[0] = sensorLeft.readSampleAverageDist(4);
    dsValues[1] = sensorFront.readSampleAverageDist(4);
    dsValues[2] = sensorRight.readSampleAverageDist(4);
    //codicão inicial
    if(t == 0){
      step = ANTICOLISION;
      Serial.println("");
      Serial.println("condição inicial: ");
    }
    //estados do robo
    switch (step) {
      case RUN:
        //Verificando se é pra virar mesmo (apenas na primeira iteração do estado)
        if(countRun == 0){
          if(isLeftHandle){
            if(dsValues[0] >= DISTANCE_FOR_CURVE){
              new_direction = M_PI/2;
            }else if(dsValues[1] >= DISTANCE_FOR_CURVE){
              new_direction = 0;
            }else if(dsValues[2] >= DISTANCE_FOR_CURVE){
              new_direction = -M_PI/2;    
            }else{
              new_direction = M_PI;
            }
          }else{
            if(dsValues[2] >= DISTANCE_FOR_CURVE){
              new_direction = -M_PI/2;
            }else if(dsValues[1] >= DISTANCE_FOR_CURVE){
              new_direction = 0;
            }else if(dsValues[0] >= DISTANCE_FOR_CURVE){
              new_direction = M_PI/2;    
            }else{
              new_direction = M_PI;
            }
          } 
        }
        
        //Verificando se é pra virar mesmo (apenas na primeira iteração do estado)
        if(target != new_direction and countRun == 0 and next_step == CURVE){
          countRun = 1;
          step = ANTICOLISION;
          next_step = CURVE;
          target = new_direction;
          right_speed = 0.0;
          left_speed = 0.0; 
        }else{
          Serial.println("Inicio Run:");
          if(next_step == CURVE){
            //run_delay = 1.4; era 1.4 mas alterei (05.12) para testar no labirinto em casa
            run_delay = 1.13;
          }else{
            //run_delay = 1.2; era 1.2 mas alterei (05.12) para testar no labirinto em casa
            run_delay = 1.0;
          }
          
          if(turning_delay > run_delay){
            identified_curve_time = t;
            step = next_step;
            if (step == CURVE){
              countCurve = 0;
              next_step = ANTICOLISION;
            }else{
              next_step = CURVE;
            }
          }
          Kp = 0.0;
          Ki = 0.0;
          
          if(dsValues[1] <= 4.2){
            base_speed = 0.0;
          }else{     
            base_speed = 0.02;
          }
          right_speed = base_speed;
          left_speed = base_speed;
        }

        //---------------------------------------- EM TESTE --------------------------------------------
        //se tem parede na esquerda ele já vai direto pro anticolisao se ele ta saindo da curva
        if(next_step = ANTICOLISION and dsValues[0] < DISTANCE_FOR_CURVE){
          step = next_step;
          next_step = CURVE;
        }
        
  
        
        break;
      case CURVE:
        //funcionamento normal do curve após de confirmar que precisar virar
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
  
          if(error < -0.035){
            right_speed = -0.02;
            left_speed = 0.02;
          }else if (error > 0.035){
            right_speed = 0.02;
            left_speed = -0.02;
          }else{
            right_speed = 0;
            left_speed = 0;
          }
  
          if(abs(error) < 0.035){
            step = RUN;
            turning_delay = 0;
            orientation=0;
            target = 0;//pois o angulo do giroscopio é resetado
            new_direction = 0;//-------------------- isso muda algo?
          }
        
        break;
      case ANTICOLISION:
       
        Kp = 0.035; // Tava em 0,013
        Ki = 0.0;
        Kd = 0.0025; //201 tava em 0,0011
        //Kp = 0.0;
        //Kd = 0.0;
        base_speed = 0.03;
        error_sensors = dsValues[0] - (25.3-10.5)/2; //7 == (distancia de labirinto - largura do robo) / 2 -> lab do gian é 25.3
        //error_sensors = dsValues[0] - dsValues[2]; //7 == (distancia de labirinto - largura do robo) / 2
        
        // Cálculo do erro integrador
        sum_error_sensors += error_sensors;
        integral = Ki * sum_error_sensors;
        proportional = Kp * error_sensors;
        derivativo = Kd * (error_sensors-previous_error);
  
        //ainda em teste (para o robo no anticolisão se tem parede na frente)
        if(dsValues[1] <= 4.0){
          base_speed = 0.0;
        }

        right_speed = base_speed + proportional + integral + derivativo;
        left_speed =  base_speed - proportional - integral - derivativo;    
        
        vel_max_curve=0.5;
        if(left_speed > vel_max_curve){
          left_speed = vel_max_curve;
        }
        if(left_speed <= 0){
          left_speed = 0.015;
        }
        if(right_speed > vel_max_curve){
          right_speed   = vel_max_curve;
        }
        if(right_speed <= 0){
          right_speed = 0.015;
        }
        
        /*
        if(isLeftHandle){
          if(dsValues[0] >= DISTANCE_FOR_CURVE){
            new_direction = M_PI/2;
          }else if(dsValues[1] >= DISTANCE_FOR_CURVE){
            new_direction = 0;
          }else if(dsValues[2] >= DISTANCE_FOR_CURVE){
            new_direction = -M_PI/2;    
          }else{
            new_direction = M_PI;
          }
        }else{
          if(dsValues[2] >= DISTANCE_FOR_CURVE){
            new_direction = -M_PI/2;
          }else if(dsValues[1] >= DISTANCE_FOR_CURVE){
            new_direction = 0;
          }else if(dsValues[0] >= DISTANCE_FOR_CURVE){
            new_direction = M_PI/2;    
          }else{
            new_direction = M_PI;
          }
        }
        */
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
  
    //Passa os valores para os motores 
    MotorA.setMotorSpeed(right_speed);
    MotorB.setMotorSpeed(left_speed);
    showNumber(step);
    
    //consolePrint(t, turning_delay, identified_curve_time, step, next_step, new_direction, error, error_sensors, Kp);   
  
    finalTime = millis();
    t += (float)(finalTime - initialTime) / 1000.0;
    turning_delay += (float)(finalTime - initialTime)  / 1000.0;
    identified_curve_time += (float)(finalTime - initialTime) / 1000.0;
  
    previous_error= error_sensors;
    //done = sensorLinha();
    delay(5);
  }
    MotorA.setMotorSpeed(0);
    MotorB.setMotorSpeed(0);
    showNumber(66);
    delay(60000);
}
