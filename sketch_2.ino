#include <SoftwareSerial.h>

uint16_t value = 0;

#define rxPin 2
#define txPin 4
#define TRUE    1
#define FALSE   0
#define USART_BUFFER_SIZE 80
#define trigPin 10
#define echoPin 7

#define BRAKE_A 9
#define BRAKE_B 8
#define DIRECTION_A 12
#define DIRECTION_B 13
#define PWM_A 3
#define PWM_B 11

float duration, distance;  

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

typedef struct{
    uint8_t USART_RX    :1;
    uint8_t AUTONOMOUS  :1;
}system_flags;

system_flags MY_flags;

char USART_Buffer[USART_BUFFER_SIZE];
void ISR_USART_RX();
void clearUSARTBuffer();
void updateSensorData();

void driveChanalA(uint8_t direction, uint8_t speed);
void driveChanalB(uint8_t direction, uint8_t speed);

void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
    
  pinMode(A5, INPUT);

  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 
  

  mySerial.begin(9600);
  Serial.begin(9600);

  pinMode(DIRECTION_A,OUTPUT); 
  pinMode(DIRECTION_B,OUTPUT);
  pinMode(PWM_A,OUTPUT);
  pinMode(PWM_B,OUTPUT);
  pinMode(BRAKE_A,OUTPUT); 
  pinMode(BRAKE_B,OUTPUT);
  
  digitalWrite(BRAKE_A,HIGH);
  digitalWrite(BRAKE_B,HIGH);
}

void loop() {

  value = analogRead(A5);
  Serial.println(value);


  if(mySerial.available()){
    ISR_USART_RX();
  }



  if(MY_flags.USART_RX == TRUE){
    if(strcmp(USART_Buffer, "<UP>") == 0){
        Serial.println("GO UP");
		}
    if(strcmp(USART_Buffer, "<DOWN>") == 0){
        Serial.println("GO DOWN");
		}
    if(strcmp(USART_Buffer, "<RIGHT>") == 0){
        Serial.println("GO RIGHT");
    }
    if(strcmp(USART_Buffer, "<LEFT>") == 0){
        Serial.println("GO LEFT");
    if(strcmp(USART_Buffer, "<BRAKE>") == 0){
        Serial.println("GO BRAKE");
		}
	
    clearUSARTBuffer();
  
   //paša pievienots - ja sensors uztver, tad robots griežas
    if(MY_flags.USART_RX == TRUE){
      if(strcmp(USART_Buffer, "<UP>") == 0){
        Serial.println("GO UP");
        if(A5 > 22, strcmp(USART_Buffer, "<RIGHT>") == 0){
          Serial.println("GO RIGHT");
        } 
      }
    }





  }
  driveChanalA(1,255);
  driveChanalB(1,255);

  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW); 

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
  }
}

void ISR_USART_RX(){
  static uint8_t myISRCnt = 0;
    USART_Buffer[myISRCnt] = mySerial.read();
    //Check if first buffer element equals to '<' (0x3c)
    if (USART_Buffer[0] != '<'){
      //Clear USART read buffer
      for (uint8_t i = 0; i < USART_BUFFER_SIZE; i++){
        USART_Buffer[i] = 0;
      }
      myISRCnt= 255;
    }else if (USART_Buffer[myISRCnt] == '>'){
      USART_Buffer[myISRCnt + 1] = '\0';
      myISRCnt = 255;
      MY_flags.USART_RX = TRUE;
      
    }
    myISRCnt++;
}

void clearUSARTBuffer(){
  for (uint8_t i = 0; i < USART_BUFFER_SIZE; i++){
        USART_Buffer[i] = 0;
      }
}

void driveChanalA(uint8_t direction, uint8_t speed){
  //kodu ko funkcija dara
  if (speed == 0){
    digitalWrite(BRAKE_A,HIGH);
  } else {
    digitalWrite(BRAKE_A,LOW);
  }

  digitalWrite(DIRECTION_A,direction);
  analogWrite(PWM_A,speed);
}

void driveChanalB(uint8_t direction, uint8_t speed){
  //kodu ko funkcija dara
  if (speed == 0){
    digitalWrite(BRAKE_B,HIGH);
  } else {
    digitalWrite(BRAKE_B,LOW);
  }

  digitalWrite(DIRECTION_B,direction);
  analogWrite(PWM_B,speed);
  }

