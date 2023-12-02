#include <ShiftRegister74HC595.h>
#include <SoftwareSerial.h>

#define SDIB 15
#define SCLKB 2
#define LOADB 4


// create shift register object (number of shift registers, data pin, clock pin, latch pin)
ShiftRegister74HC595<2> sr (SDIB, SCLKB, LOADB);

int led = 13, countA = 0, countB = 0, firstTime=0;
int value,
digit1,//dezena A
digit2,//unidade A
digit3,//dezena B
digit4; //unidade B

uint8_t digits[] = {
  B11000000, //0
  B11111001, //1
  B10100100, //2
  B10110000, //3
  B10011001, //4
  B10010010, //5
  B10000010, //6
  B11111000, //7
  B10000000, //8
  B10010000 //9
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Teste modulo display 7seg");

}

void loop() {
  // put your main code here, to run repeatedly:
  if(countA == 14){
    countA = 0;
  }
  countA = countA+1;
  showNumber(countA);
  delay(1000);
}

void showNumber(int num)
{
  digit2=num % 10 ;
  digit1=(num / 10) % 10 ;
  //Send them to segment displays
  uint8_t numberToPrint[]= {digits[digit1],digits[digit2]};
  sr.setAll(numberToPrint);
}
