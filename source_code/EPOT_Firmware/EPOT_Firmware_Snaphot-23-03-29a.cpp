/*****************************************************
EPOT Project - ESP32 Firmware
(c)2023 Arnaud LE COSSEC
(c)2023 College Lionel Groulx
*****************************************************/

/*****************************************************
Change Notes : 
28/03/2023a : File creation
              void LED_init(void);
              void LED_set(uint8_t freq);
              void Ultrasonic_init(void);
              float Ultrasonic_Read(void);
              void Aref_init(void);
              void Temp_init(void);
              float Temp_Read(void);
              void Photo_Cell_init(void);
              float Photo_Cell_Read(void);
              void Humidity_init(void);
              float Humidity_Read(void);
29/03/2023a : POST test implementation
              LED not flashing bug fixed (prototype used instead of function call)

*****************************************************/


/*****************************************************
Preprocessors
*****************************************************/
#include <math.h>

#define LED_PIN 12
#define LED_PWM_CH 0

#define ULT_TRIG_PIN 26
#define ULT_ECHO_PIN 27

#define AREF_PIN 36

#define THER_PIN 39
#define THER_REF 10000

#define PHOTO_PIN 34
#define PHOTO_REF 10000

#define HUM_PIN 35
#define HUM_WET_VALUE 2630// à définir
#define HUM_DRY_VALUE 870// à définir

/*****************************************************
Prototypes
*****************************************************/

void LED_init(void);
void LED_set(uint8_t freq);

void Ultrasonic_init(void);
float Ultrasonic_Read(void);

void Aref_init(void);

void Temp_init(void);
float Temp_Read(void);

void Photo_Cell_init(void);
float Photo_Cell_Read(void);

void Humidity_init(void);
float Humidity_Read(void);



/*****************************************************
Main
*****************************************************/

void setup() {
  // Serial Setup for debug
  Serial.begin(9600);

  // Pin Setup 
  LED_init();
  Ultrasonic_init();
  Aref_init();
  Temp_init();
  Photo_Cell_init();
  Humidity_init();

  // Run check
  delay(1000);

  LED_set(1); // Set LED to 1Hz during startup to indicate CPU activity
  Serial.println("--------- EPOT PROJET DEBUG OUTPUT --------");
  Serial.println("Firmware 29/03/23a (c)2023 Arnaud LE COSSEC");

  uint8_t Error_Count = 0;

  // Check ultrasonic sensor
  if(Ultrasonic_Read()==0){Serial.println("[ERROR] Distance sensor not responding (dist = 0cm)");Error_Count++;}
  if(Ultrasonic_Read()>100){Serial.println("[ERROR] Distance sensor reading error (dist > 100cm)");Error_Count++;}
  // Check Aref
  if(analogRead(AREF_PIN)<3000){Serial.println("[ERROR] Analog reference reading error - Too low (Aref < 3000)");Error_Count++;}
  if(analogRead(AREF_PIN)>4096){Serial.println("[ERROR] Analog reference reading error - Too high (Aref > 4096)");Error_Count++;}
  // Check Temperature sensor
  if(Temp_Read()>70){Serial.println("[ERROR] Temperature sensor shorted (R_th=0; temp. < -20°C");Error_Count++;}
  if(Temp_Read()<-20){Serial.println("[ERROR] Temperature sensor not connected (R_th=inf; temp. > 70°C)");Error_Count++;}
  // Check Photo cell
  if(Photo_Cell_Read()<1){Serial.println("[ERROR] Photo cell sensor shorted (R_ph=0)");Error_Count++;}
  if(Photo_Cell_Read()>10000000){Serial.println("[ERROR] Photo cell sensor not connected (R_ph=inf)");Error_Count++;}
  // Check Humididy sensor
  if(AnalogRead(HUM_PIN)<100){Serial.println("[ERROR] Humidity sensor not connected");Error_Count++;}
  if(AnalogRead(HUM_PIN)<(HUM_WET_VALUE-100) || AnalogRead(HUM_PIN)>(HUM_WET_VALUE+100)){Serial.println("[WARNING] Humidity sensor uncalibrated");Error_Count++;}

  if(Error_Count>0){LED_set(10);}
  Serial.print(Error_Count);
  Serial.println(" errors found");


}

void loop() {

  // put your main code here, to run repeatedly:
  Serial.print(Temp_Read());
  Serial.print(" °C   ");
  Serial.print(Photo_Cell_Read());
  Serial.print(" ohm    ");
  Serial.print(Ultrasonic_Read());
  Serial.println(" cm ");
  delay(500);
}

/*****************************************************
Functions
*****************************************************/



void LED_init(void){
  pinMode(LED_PIN, OUTPUT);
  ledcAttachPin(LED_PIN, LED_PWM_CH);
}

void LED_set(uint8_t freq){
  ledcSetup(LED_PWM_CH, freq, 8);
  ledcWrite(LED_PWM_CH, 128);
}

void Ultrasonic_init(void){
  pinMode(ULT_TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ULT_ECHO_PIN, INPUT); // Sets the echoPin as an Input
}

float Ultrasonic_Read(void){
  // Clears the trigPin
  digitalWrite(ULT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ULT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULT_TRIG_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ULT_ECHO_PIN, HIGH);
  
  // Calculate the distance
  // distanceCm = duration * SOUND_SPEED/2;
  float distanceCm = duration / 58.0;
  return distanceCm;
}

void Aref_init(void){
  pinMode(AREF_PIN, INPUT);
}

void Temp_init(void){
  pinMode(THER_PIN, INPUT);
}

float Temp_Read(void){
  // Thermistor used is NTC, 3977K, 10kΩ from Vishay

  // read analog input pins
  int ADC_ref = analogRead(AREF_PIN);
  int ADC_thermistor = analogRead(THER_PIN);
  
  // Convert to volts
  float V_ref = (3.3*ADC_ref)/4095.0;
  float V_Th = (3.3*ADC_thermistor)/4095.0;

  // Calculate Thermistor resistor
  float R_Th = (V_Th*THER_REF) / (V_ref-V_Th);

  // Calculate Temperature (C°) from resistor value
  float Temp = -22.16*log(R_Th)+229.4;

  return Temp;
}

void Photo_Cell_init(void){
  pinMode(PHOTO_PIN, INPUT);
}

float Photo_Cell_Read(void){
  // read analog input pins
  int ADC_ref = analogRead(AREF_PIN);
  int ADC_photo = analogRead(PHOTO_PIN);

  // Convert to volts
  float V_ref = (3.3*ADC_ref)/4095.0;
  float V_photo = (3.3*ADC_photo)/4095.0;

  // Calculate Photo cell resistor
  float R_photo = (THER_REF*(V_ref-V_photo)) / V_photo;

  return R_photo; // Return Photo cell resistor
}

void Humidity_init(void){
  pinMode(HUM_PIN, INPUT);
}

float Humidity_Read(void){
  // Read sensor
  uint16_t ADC_hum = analogRead(HUM_PIN);
  // Convert to percentage
  return (((HUM_WET_VALUE-HUM_DRY_VALUE)/100.0)*ADC_hum + HUM_DRY_VALUE); 
}


