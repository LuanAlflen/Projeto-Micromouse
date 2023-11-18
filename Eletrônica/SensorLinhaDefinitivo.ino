//pino1 pino2 pino3 pino4 serão referentes as entradas analógicas de cada sensor do sensor de linha
 
int SensorLinha(){
  int sensor1 = analogRead(pino1);
  int sensor2 = analogRead(pino2);
  int sensor3 = analogRead(pino3);
  int sensor4 = analogRead(pino4);
  if((sensor1 > 950) || (sensor2 > 950) || (sensor3 > 950) || (sensor4 > 950)){
    return 1;
  } else {
    return 0;
  }
}