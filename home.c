/**********************************************************************
* Filename    : I2CLCD1602.c
* Description : Use the LCD display data
* Author      : freenove
* modification: 2017/04/18
**********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <pcf8574.h>
#include <pcf8591.h>
#include <lcd.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <sys/time.h>

#define pcf8574_address 0x27        // default I2C address of Pcf8574
#define BASE 64         // BASE is not less than 64
//////// Define the output pins of the PCF8574, which are directly connected to the LCD1602 pin.
#define RS      BASE+0
#define RW      BASE+1
#define EN      BASE+2
#define LED     BASE+3
#define D4      BASE+4
#define D5      BASE+5
#define D6      BASE+6
#define D7      BASE+7

#define pcf8591_address 0x48
#define temperature_base 100
#define A0 temperature_base + 0
#define A1 temperature_base + 1 
#define A2 temperature_base + 2 
#define A3 temperature_base + 3 

#define ledPin 0 //define the ledPin 
#define buttonPin 1 // define the buttonPin

#define trigPin 4
#define echoPin 5
#define MAX_DISTANCE 220
#define timeOut MAX_DISTANCE*60

int lcdhd;// used to handle LCD
int sw = 0,mail = 0;
int warningLight = 0,warningSonar = 0;

void detectLight(){
	int adcValue;
	float voltage;

	adcValue = analogRead(A0); //read A0 pin 
	voltage = (float)adcValue / 255.0 * 3.3; // calculate voltage 
	if (voltage < 0.5 && sw == 1){
		warningLight = 1;
	}
}

int pulseIn(int pin, int level, int timeout){
   struct timeval tn, t0, t1;
   long micros;
   gettimeofday(&t0, NULL);
   micros = 0;
   while (digitalRead(pin) != level)
   {
      gettimeofday(&tn, NULL);
      if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
      micros += (tn.tv_usec - t0.tv_usec);
      if (micros > timeout) return 0;
   }
   gettimeofday(&t1, NULL);
   while (digitalRead(pin) == level)
   {
      gettimeofday(&tn, NULL);
      if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
      micros = micros + (tn.tv_usec - t0.tv_usec);
      if (micros > timeout) return 0;
   }
   if (tn.tv_sec > t1.tv_sec) micros = 1000000L; else micros = 0;
   micros = micros + (tn.tv_usec - t1.tv_usec);
   return micros;
}

float getSonar(){
	long pingTime; 
	float distance; 
	digitalWrite(trigPin,HIGH); //trigPin send 10us high level 
	delayMicroseconds(10); 
	digitalWrite(trigPin,LOW); 
	pingTime = pulseIn(echoPin,HIGH,timeOut); //read plus time of echoPin 
	distance = (float)pingTime * 340.0 / 2.0 / 10000.0; // the sound speed is 340m/s,and calculate distance 
	if (distance < 20) warningSonar = 1;
	return distance;
}

void securityMode(){
	lcdClear(lcdhd);
	lcdPosition(lcdhd,0,0);
	lcdPrintf(lcdhd,"Security Mode");
	lcdPosition(lcdhd,0,1);
	lcdPrintf(lcdhd,"Status:safe%c%c",0x3A,0x29);
}
void checkDanger(void *m){
	float distance;
	securityMode();
	while(sw == 1){
		detectLight();
		distance = getSonar();
		if (warningLight == 1 || warningSonar == 1) lcdClear(lcdhd);
		else {
			mail = 0;
			continue;
		}
		if (warningLight == 1){
			lcdPosition(lcdhd,0,0);
			lcdPrintf(lcdhd,"Light is On!");
			warningLight = 0;
		}
		if (warningSonar == 1){
			lcdPosition(lcdhd,0,1);
			lcdPrintf(lcdhd,"Object at %dcm",(int)distance);
			warningSonar = 0;
		}
		if (mail == 0){
			system("echo \"Warning!!!\" | mail -s \"Warning From Raspi\" \"sunh7@uci.edu\"");
			mail = 1;
		}
		delay(1000);
		securityMode();
	}
}

void printTemperature(){// sub function used to print CPU temperature
	int adcValue;
	float tempK,tempC,voltage,Rt;

	adcValue = analogRead(A0); //read A0 pin 
	voltage = (float)adcValue / 255.0 * 3.3; // calculate voltage 
	Rt = 10 * voltage / (3.3 - voltage); //calculate resistance value of thermistor 
	tempK = 1/(1/(273.15 + 25) + log(Rt/10)/3950.0); //calculate temperature (Kelvin) 
	tempC = tempK -273.15; //calculate temperature (Celsius) 
	printf("ADC value : %d ,\tVoltage : %.2fV, \tTemperature : %.2fC\n",adcValue,voltage,tempC);
	lcdClear(lcdhd);
	lcdPosition(lcdhd,9,1);     // set the LCD cursor position to (0,0) 
    lcdPrintf(lcdhd,"%.2f%cC",tempC,0xDF);// Display temperature on LCD    
}

void printDataTime(){//used to print system time 
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);// get system time
    timeinfo = localtime(&rawtime);// convert to local time
    //char *result_time = asctime(timeinfo);
    printf("%s \n",asctime(timeinfo));
    lcdPosition(lcdhd,4,0);// set the LCD cursor position to (0,1) 
    lcdPrintf(lcdhd,"%02d:%02d:%02d ",timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);
    lcdPosition(lcdhd,0,1);
    lcdPrintf(lcdhd,"%02d-%02d-%02d",timeinfo->tm_mon+1,timeinfo->tm_mday,timeinfo->tm_year%100);
}

void checkButton(void* m){
	while(1){
		if(digitalRead(buttonPin) == LOW){ //button has pressed down
    		sw = (sw == 1)?0:1;
       		digitalWrite(ledPin, HIGH);
       		delay(1000);
    	}else{
       		digitalWrite(ledPin, LOW); //led off
     	}
	}
}

int main(void){
    int i;
    pthread_t t1,t2;
    

    if(wiringPiSetup() == -1){ //when initialize wiring failed,print messageto screen
        printf("setup wiringPi failed !");
        return 1; 
    }

    pcf8574Setup(BASE,pcf8574_address);// initialize PCF8574
    pcf8591Setup(temperature_base,pcf8591_address);// initialize PCF8591
    for(i=0;i<8;i++){
        pinMode(BASE+i,OUTPUT);     // set PCF8574 port to output mode
    } 
    digitalWrite(LED,HIGH);     // turn on LCD backlight
    digitalWrite(RW,LOW);       // allow writing to LCD
    pinMode(ledPin, OUTPUT); 
    pinMode(buttonPin, INPUT); 
    pinMode(trigPin,OUTPUT);
    pinMode(echoPin,INPUT);
    pullUpDnControl(buttonPin, PUD_UP); //pull up to high level 

    lcdhd = lcdInit(2,16,4,RS,EN,D4,D5,D6,D7,0,0,0,0);// initialize LCD and return “handle” used to handle LCD
    
    if(lcdhd == -1){
        printf("lcdInit failed !");
        return 1;
    }
    pthread_create(&t1,NULL,checkButton,(void *)"hello,");
    while(1){
    	if (sw == 0) {
    		printTemperature();// print CPU temperature
       		printDataTime();        // print system time
    	}
    	else{
    		pthread_create(&t2,NULL,checkDanger,(void *)"hello,");
    		pthread_join(t2,NULL);
    	}
    	
        delay(1000);
    }
    return 0;
}