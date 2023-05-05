/*****************************************************
Preprocessors
*****************************************************/
#include math

#define LED_PIN 12
#define LED_PWM_CH 0

#define ULT_TRIG_PIN 26
#define ULT_ECHO_PIN 27

#define AREF_PIN 36

#define THER_PIN 39
#define THER_REF 10000

#define PHOTO_PIN 39
#define PHOTO_REF 10000

#define HUM_PIN 35
#define HUM_WET_VALUE // à définir
#define HUM_DRY_VALUE // à définir

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
Functions
*****************************************************/

void LED_init(void){
  pinMode(LED_PIN, INPUT);
  ledcAttachPin(LED_PIN, LED_PWM_CH);
}

void LED_set(uint8_t freq){
  ledcSetup(LED_PWM_CH, freq, 8);
  ledcWrite(LED_PWM_CH, 128);
}

void Ultrasonic_init(void){
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

float Ultrasonic_Read(void){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  // distanceCm = duration * SOUND_SPEED/2;
  distanceCm = duration / 58.0;
  return distanceCm;
}

void Aref_init(void){
  pinMode(Aref_init, INPUT);
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
  float Temp = -22.16*math.log(R_th)+229.4;

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
  uint16_t sensor_in = analogRead(HUM_PIN);
  // Convert to percentage
  return ((HUM_WET_VALUE-HUM_DRY_VALUE)/100)*HUM_PIN + HUM_DRY_VALUE;
}


