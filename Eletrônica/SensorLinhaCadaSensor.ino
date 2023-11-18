const int pino1 = A0; //PINO ANALÓGICO UTILIZADO PELO SENSOR
const int pino2 = A1;
const int pino3 = A4;
const int pino4 = A5;


void setup(){
  Serial.begin(9600); //INICIALIZAÇÃO DA SERIAL
   pinMode(pino1, INPUT); //DEFINE O PINO COMO ENTRADA
   pinMode(pino2, INPUT);
   pinMode(pino3, INPUT);
   pinMode(pino4, INPUT);
}
 
void loop(){
  delay(2000);
  int sensor1 = analogRead(pino1);
  int sensor2 = analogRead(pino2);
  int sensor3 = analogRead(pino3);
  int sensor4 = analogRead(pino4);
  if(sensor1 > 950){
    Serial.println("Sensor 1 detecta preto");
  }else{
    Serial.println("Sensor 1 não detecta preto");
  }
  if(sensor2 > 950){
    Serial.println("Sensor 2 detecta preto");
  }else{
    Serial.println("Sensor 2 não detecta preto");
  }
  if(sensor3 > 950){
    Serial.println("Sensor 3 detecta preto");
  }else{
    Serial.println("Sensor 3 não detecta preto");
  }
  if(sensor4 > 950){
    Serial.println("Sensor 4 detecta preto");
  }else{
    Serial.println("Sensor 4 não detecta preto");
  }
}