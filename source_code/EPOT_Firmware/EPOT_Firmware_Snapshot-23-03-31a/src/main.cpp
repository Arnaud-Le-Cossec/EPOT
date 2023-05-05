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
29/03/2023c : Wifi setup implementation
29/03/2023d : Database setup implementation
              Code separation with EPOT_lib.h
31/03/2023a : watchdog implementation

*****************************************************/


/*****************************************************
Preprocessors
*****************************************************/
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <math.h>
#include <Firebase_ESP_Client.h>
#include "EPOT_lib.h"



/*****************************************************
Prototypes
*****************************************************/

void Task_Sensors_Update_routine( void * pvParameters );
void Task_Sensors_Print_routine( void * pvParameters );


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

  Serial.println("--------- EPOT PROJET DEBUG OUTPUT --------");
  Serial.println("Firmware 29/03/23d (c)2023 Arnaud LE COSSEC");

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
  uint8_t Error_log[11] = {};
  uint8_t Error_Count = Self_test(Error_log);
  if(Error_Count>0){LED_set(10);} // Set LED to 10Hz if error occured


  // ------------------ Setup wifi ------------------

  Wifi_Setup();

  // ---------------- Setup Watchdog ----------------

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // ---------------- Setup database ----------------

  Database_init();

  Database_Write_int("/EPOT1/ERROR_Count",Error_Count);
  Database_Write_string("/EPOT1/ERROR_Description", "Test !");

  // ----------------- Create tasks -----------------

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
 //                   1,                                  /* priority of the task */
 //                   &Task_Database_Write_handle,        /* Task handle to keep track of created task */
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
  esp_task_wdt_reset();   // reset watchdog timer
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


