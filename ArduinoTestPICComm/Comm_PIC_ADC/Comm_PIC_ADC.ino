#include <HardwareSerial.h>
#define CANALES_ADC 9
#define DATA_PIC (CANALES_ADC+1)
#define HEADER 10

char temp[10];
char tmp;
int16_t Mediciones[10];
HardwareSerial SerialPIC(2);
void setup() {
  // put your setup code here, to run once:
  SerialPIC.begin(57600);
  while(!SerialPIC);
  Serial.begin(9600);
}

void loop() {
temp[0]= '\0';
tmp=SerialPIC.read();
if(tmp == 'L'){
    temp[0]=tmp;
    for(int i=1; i < HEADER; i++){
      temp[i] = SerialPIC.read();
    }
  if(SerialPIC.available()>0){
    for(int i=0; i < DATA_PIC; i++){
      byte L = SerialPIC.read();
      byte H = SerialPIC.read();
      Mediciones[i] = (((int16_t)H) << 8) | L;
      }
  }
  Serial.print(temp);
  for(int i=0; i < DATA_PIC ; i++){
   Serial.print(Mediciones[i]);
   Serial.print(' ');
  }
  Serial.println();
 }
}
