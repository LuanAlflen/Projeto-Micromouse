#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>
#include <webots/DistanceSensor.hpp>
#include <cmath>

#define TIME_STEP 16

using namespace webots;



Robot *robot = new Robot();
Motor *motor1 = robot->getMotor("motor_1");
Motor *motor2 = robot->getMotor("motor_2");
Gyro *gyro = robot->getGyro("gyro");
DistanceSensor *ds[3];


const int CURVE = 0;
const int RUN = 1;
const int ANTICOLISION = 2;
//const double DELAY = 1.1;

char dsNames[3][20] = {"distance_sensor1", "distance_sensor2", "distance_sensor3"};//esq, centro, dir
double dsValues[3];//em centímetros
double orientation = 0.0;
double target = 0, previous_target=0.0;//previous não usando pra nada ainda nesse código
double run_delay = 1.0;

void updateGyrol(int timeStep){
    orientation += (timeStep / 1000.0) * gyro->getValues()[2];
    if(orientation > M_PI){
      orientation -= 2*M_PI; 
    }
    if(orientation < -M_PI){
      orientation += 2*M_PI; 
    }
}

double selectDirection(double current_target){
  double direction = 0.0;
  if(dsValues[0] >= 10){
    direction = M_PI/2;
  }else if(dsValues[1] >= 10){
    direction = 0;
  }else if(dsValues[2] >= 10){
    direction = -M_PI/2;    
  }else{
    direction = M_PI;
  }
  
  double final_direction = direction + current_target;
  if(final_direction > M_PI){
    final_direction -= 2 * M_PI;
  }else if (final_direction < -M_PI){
    final_direction += 2 * M_PI;
  }
  return final_direction; 
}

void consolePrint(double t_, double turning_delay_, double identified_curve_time_, double step_, double next_step_, double new_direction_, double error_, double error_sensors_, double Kp_){

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "t: " << t_ << ", t_delay: " << turning_delay_ << ", t_id: " << identified_curve_time_;
    if(step_ == CURVE){
      std::cout << ", step: CURVE";
    }else if(step_ == RUN){
      std::cout << ", step: RUN";
    }else if(step_ == ANTICOLISION){
      std::cout << ", step: ANTICOLISION";
    }
    if(next_step_ == CURVE){
      std::cout << ", next_step: CURVE" << std::endl;
    }else if(next_step_ == RUN){
      std::cout << ", next_step: RUN" << std::endl;
    }else if(next_step_ == ANTICOLISION){
      std::cout << ", next_step: ANTICOLISION" << std::endl;
    }
    
    std::cout << "Target: " << target << ", P_target: " << previous_target << " rad, orientation: " << orientation << " rad, new_dire: " << new_direction_ << " rad"<< std::endl;
    std::cout << "s1: " << dsValues[0] << ", s2: " << dsValues[1] << ", s3: " << dsValues[2] << std::endl;
    std::cout << "dif_angle: " << abs(previous_target - target) << ", error_gyro: " << error_ << ", dif_sensors: " << error_sensors_ << ", Kp: " << Kp_ << std::endl;
  

}

int main() {

  //const double F = 2.0;   // frequency 2 Hz
  double t = 0.0, turning_delay=99.0, identified_curve_time=0.0;         // elapsed simulation time
  double max_speed = 8.37758;
  double base_speed = 0.0;
  double right_speed=0.0, left_speed=0.0;
  double error = 0.0, sum_error=0.0, error_sensors=0.0, sum_error_sensors=0.0;
  double Kp=15.0, Ki=0.0;
  double proportional=0.0, integral=0.0;
  double new_direction = 0.0;
  int step = CURVE, next_step=CURVE; //0->Virando, 1->Indo pra frente por X seg, 2->Anticolisão (linha reta)
  
   //fazendo o enable nos sensores
  for(int i = 0; i < 3; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);;
  }
  
  motor1->setPosition(INFINITY);
  motor1->setVelocity(0.0);
  motor2->setPosition(INFINITY);
  motor2->setVelocity(0.0);
  gyro->enable(TIME_STEP);

  

  while (robot->step(TIME_STEP) != -1) {
    //fazendo a leitura nos sensores
    for(int i = 0; i < 3; i++){
      dsValues[i] = ds[i]->getValue()/100;
    }
    
    //codicão inicial
    if(t == 0){
      step = ANTICOLISION;
    
      std::cout << "condição inicial: " << target << std::endl;
      std::cout << "target: " << target << std::endl;
      target = selectDirection(target);
      std::cout << "target após selectDirection: " << target << std::endl;
      consolePrint(t, turning_delay, identified_curve_time, step, next_step, new_direction, error, error_sensors, Kp);
    }
    
    
    //escolhendo o formato do PID (um pra curvas abruscas e outro pra andar em linha reta)
    if(step == RUN){
    
      if(next_step == CURVE){
        run_delay = 0.95;
      }else{
        run_delay = 0.85;
      }
      
      if(dsValues[0] <= 14 && next_step == ANTICOLISION){
        step = ANTICOLISION;
        next_step = CURVE;
        new_direction = 0;
      }
      
      if(turning_delay > run_delay){
        identified_curve_time = t;
        step = next_step;
      }
      Kp = 0.0;
      Ki = 0.0;
      
      if(dsValues[1] <= 4.0){
        base_speed = 0.0*max_speed;
      }else{     
        base_speed = 1.0*max_speed;
      }
      right_speed = base_speed;
      left_speed = base_speed;
      
    //--------------------------------------------------
    }else if (step == CURVE){
    
      updateGyrol(TIME_STEP);
      //corrigindo erro pra não ter mudanças bruscas
      error = target - orientation;
      if (error > M_PI) {
          error -= 2 * M_PI;
      } else if (error < -M_PI) {
          error += 2 * M_PI;
      }
                 
      Kp=50.0;
      Ki=0.0;
      base_speed = 0.0*max_speed;
      
      // Cálculo do erro integrador
      sum_error += error;
      integral = Ki * sum_error;
      
      proportional = Kp * error;
      right_speed = base_speed + proportional + integral;
      left_speed = base_speed - proportional - integral; 
      next_step = ANTICOLISION;
      if(abs(error) < 0.01){
        step = RUN;
        turning_delay = 0;
        
        orientation=0;
        target = 0;//pois o angulo do giroscopio é resetado
        new_direction = 0;//-------------------- isso muda algo?
      }
      
       
    //--------------------------------------------------
    }else if(step == ANTICOLISION){
      Kp = 1.2;
      Ki=0.0;
      base_speed = 0.8*max_speed;
      error_sensors = dsValues[0] - 7; //7 == (distancia de labirinto - largura do robo) / 2
      
      // Cálculo do erro integrador
      sum_error_sensors += error_sensors;
      integral = Ki * sum_error_sensors;
      proportional = Kp * error_sensors;
      
      right_speed = base_speed + proportional + integral;
      left_speed = base_speed - proportional - integral; 
      
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
      //std::cout << "--target: " << target << ", new_direction: " << new_direction << std::endl;
      if(target != new_direction){
        turning_delay = 0;
        step = RUN;
        target = new_direction;
        right_speed = base_speed;
        left_speed = base_speed; 
        // identified_curve_time = t - 1.0;
      }
      
      // if((t - identified_curve_time) > 1.0){
        // step = RUN;
      // }
      
         
    }
    
    // if(turning_delay > 1.1){
      // new_direction = selectDirection(target);
    // }
    
    
    // if(new_direction != target and reset_delay == 0){
      // turning_delay = 0.0;
      // reset_delay = 1.0;
    // }
    
    // if(new_direction != target and turning_delay > 1.1){
      // previous_target = target;
      // target = new_direction;
      // reset_delay = 0.0;
      // turning_delay = 0.0;
    // }
    
    
    
    //Passa os valores para os motores
    if(dsValues[1] <= 4 and target == 999.0/*new_direction*/){
      motor2->setVelocity(0);
      motor1->setVelocity(0);
    }else{
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
      motor2->setVelocity(left_speed);
      motor1->setVelocity(right_speed);
    }
    
    consolePrint(t, turning_delay, identified_curve_time, step, next_step, new_direction, error, error_sensors, Kp);   
    t += (double)TIME_STEP / 1000.0;
    turning_delay += (double)TIME_STEP / 1000.0;
    identified_curve_time += (double)TIME_STEP / 1000.0;
    
  }

  delete robot;
  return 0;
}