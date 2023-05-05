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
29/03/2023b : Task implementation
              Task Sensors Update
              Task Sensors Print - debug only, to be removed later on

*****************************************************/


/*****************************************************
Preprocessors
*****************************************************/
#include <math.h>

#define LED_PIN 12
#define LED_PWM_CH 0

#define ULT_TRIG_PIN 26
#define ULT_ECHO_PIN 27
#define WATER_TANK_HEIGHT 15 // cm
#define WATER_TANK_SENSOR_SEPERATION 5 // cm

#define AREF_PIN 36

#define THER_PIN 39
#define THER_REF 10000 // ohm

#define PHOTO_PIN 34
#define PHOTO_REF 10000 // ohm

#define HUM_PIN 35
#define HUM_WET_VALUE 870// à définir (mesure)
#define HUM_DRY_VALUE 2640// à définir (mesure)

/*****************************************************
Prototypes
*****************************************************/

void Task_Sensors_Update_routine( void * pvParameters );
void Task_Sensors_Print_routine( void * pvParameters );

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
Global variables
*****************************************************/

//TaskHandle_t Task_Database_Read_routine;
//TaskHandle_t Task_Database_Write_routine;
TaskHandle_t Task_Sensors_Update_handle;
TaskHandle_t Task_Sensors_Print_handle;


QueueHandle_t Queue_Temperature;
QueueHandle_t Queue_Light_level;
QueueHandle_t Queue_Humidity_percentage;
QueueHandle_t Queue_Water_Level_percentage;

/*****************************************************
Main
*****************************************************/

void setup() {
  // ------------------ Serial Setup for debug ------------------
  Serial.begin(9600);

  // ------------------ Pin Setup ------------------
  LED_init();
  Ultrasonic_init();
  Aref_init();
  Temp_init();
  Photo_Cell_init();
  Humidity_init();

  // ------------------ Run check ------------------
  delay(1000);

  LED_set(1); // Set LED to 1Hz during startup to indicate CPU activity
  Serial.println("--------- EPOT PROJET DEBUG OUTPUT --------");
  Serial.println("Firmware 29/03/23b (c)2023 Arnaud LE COSSEC");

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
  if(analogRead(HUM_PIN)<100){Serial.println("[ERROR] Humidity sensor not connected");Error_Count++;}
  if(analogRead(HUM_PIN)<(HUM_WET_VALUE-100) || analogRead(HUM_PIN)>(HUM_WET_VALUE+100)){Serial.print("[WARNING] Humidity sensor uncalibrated ");Serial.println(analogRead(HUM_PIN));Error_Count++;}

  if(Error_Count>0){LED_set(10);}
  Serial.print(Error_Count);
  Serial.println(" errors found");

  // ----------------- Create tasks ---------------

//  xTaskCreatePinnedToCore(
//                    Task_Database_Read_routine,         /* Task function. */
//                    "Task_Database_Read",               /* name of task. */
//                    2048,                               /* Stack size of task */
//                    NULL,                               /* parameter of the task */
//                    1,                                  /* priority of the task */
//                    &Task_Database_Read_handle,         /* Task handle to keep track of created task */
//                    1);                                 /* pin task to core 1 */

//  xTaskCreatePinnedToCore(
//                    Task_Database_Write_routine,        /* Task function. */
//                    "Task_Database_Write",              /* name of task. */
//                    2048,                               /* Stack size of task */
//                    NULL,                               /* parameter of the task */
//                    1,                                  /* priority of the task */
//                    &Task_Database_Write_handle,        /* Task handle to keep track of created task */
//                    1); 

  xTaskCreatePinnedToCore(
                    Task_Sensors_Update_routine,        /* Task function. */
                    "Task_Sensors_Update_automation",   /* name of task. */
                    2048,                               /* Stack size of task */
                    NULL,                               /* parameter of the task */
                    1,                                  /* priority of the task */
                    &Task_Sensors_Update_handle,        /* Task handle to keep track of created task */
                    1);                                 /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    Task_Sensors_Print_routine,         /* Task function. */
                    "Task_Sensors_Print_automation",    /* name of task. */
                    2048,                               /* Stack size of task */
                    NULL,                               /* parameter of the task */
                    1,                                  /* priority of the task */
                    &Task_Sensors_Print_handle,         /* Task handle to keep track of created task */
                    1);                                 /* pin task to core 1 */

  // ----------------- Create queues ---------------

  Queue_Temperature = xQueueCreate( 2, sizeof( float ) );
  Queue_Light_level = xQueueCreate( 2, sizeof( float ) );
  Queue_Humidity_percentage = xQueueCreate( 2, sizeof( float ) );
  Queue_Water_Level_percentage = xQueueCreate( 2, sizeof( float ) );

  // Check if the queues has been created properly
  if(Queue_Temperature == NULL ||
     Queue_Light_level == NULL ||
     Queue_Humidity_percentage == NULL ||
     Queue_Water_Level_percentage == NULL){

    Serial.println("[ERROR] Error occured creating the queues");
  }
}

void loop() {
/*
  // put your main code here, to run repeatedly:
  Serial.print(Temp_Read());
  Serial.print(" °C   ");
  Serial.print(Photo_Cell_Read());
  Serial.print(" ohm    ");
  Serial.print(Ultrasonic_Read());
  Serial.println(" cm ");
  delay(500);
*/
}

/*****************************************************
Tasks
*****************************************************/
/*
void Task_Database_Read_routine( void * pvParameters ){
  Serial.print("Database reading task is online on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    //
  } 
}

void Task_Database_Write_routine( void * pvParameters ){
  Serial.print("Database Writing task is online on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    //
  } 
}
*/
void Task_Sensors_Update_routine( void * pvParameters ){
  Serial.print("Sensor update task is online on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    // Read temperature
    float temperature = Temp_Read();
    xQueueSend(Queue_Temperature, &temperature, 0); // Write temperature into queue, give up immediatly if the queue is full
    // Read light level
    float light_level = Photo_Cell_Read();
    xQueueSend(Queue_Light_level, &light_level, 0); // Write temperature into queue, give up immediatly if the queue is full
    // Read Humidity level
    float humidity = Humidity_Read();
    xQueueSend(Queue_Humidity_percentage, &humidity, 0); // Write temperature into queue, give up immediatly if the queue is full
    // Read water level level
    float Water_level_percentage = (WATER_TANK_HEIGHT-Ultrasonic_Read()-WATER_TANK_SENSOR_SEPERATION)/WATER_TANK_HEIGHT; // convert sensor distance into percentage
    xQueueSend(Queue_Water_Level_percentage, &Water_level_percentage, 0); // Write temperature into queue, give up immediatly if the queue is full
  } 
}

void Task_Sensors_Print_routine( void * pvParameters ){
  Serial.print("Sensor print task is online on core ");
  Serial.println(xPortGetCoreID());

  float temperature = 0;
  float light_level = 0;
  float humidity = 0;
  float Water_level_percentage = 0;

  for(;;){
    // Read temperature from queue
    xQueueReceive(Queue_Temperature, &temperature, 0); // Write temperature from queue, give up immediatly if the queue is full
    // Read temperature from queue
    xQueueReceive(Queue_Light_level, &light_level, 0); // Write temperature from queue, give up immediatly if the queue is full
    // Read Humidity level from queue
    xQueueReceive(Queue_Humidity_percentage, &humidity, 0); // Write temperature from queue, give up immediatly if the queue is full
    // Read water level level from queue
    xQueueReceive(Queue_Water_Level_percentage, &Water_level_percentage, 0); // Write temperature from queue, give up immediatly if the queue is full

    Serial.print(temperature);
    Serial.print(" °C   ");
    Serial.print(light_level);
    Serial.print(" ohm    ");
    Serial.print(humidity);
    Serial.print(" % ");
    Serial.print(Water_level_percentage);
    Serial.println(" % ");
  } 
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
  //float coef = (100.0/(HUM_WET_VALUE-HUM_DRY_VALUE))
  //(ADC_hum - HUM_DRY_VALUE)*(100/(HUM_WET_VALUE-HUM_DRY_VALUE));
  return ((ADC_hum - HUM_DRY_VALUE)*(100.0/(HUM_WET_VALUE-HUM_DRY_VALUE))); 
}


