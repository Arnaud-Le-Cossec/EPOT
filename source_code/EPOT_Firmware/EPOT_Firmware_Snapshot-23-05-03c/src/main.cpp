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
03/04/2023a : Sensors to database
              Ledc library error fixed
04/04/2023a : Error log to database
              void Database_Self_test_repport(uint8_t Error_Count, uint8_t *Error_log)
              Change uint8_t Self_test(uint8_t *Error_log_table) : text as const char*
04/04/2023b : Bluetooth implementation
12/04/2023a : Startup sequence (stopped due to flash size issue)
13/04/2023a : New partion table with 3MB free for application use
13/04/2023b : Startup sequence (continuation)
              Task_database_read() / Task_database_write() deleted because replaced by task_database_update()
19/04/2023a : Database authentification refresh implementation
              [bug fix] "Invalid data; couldn't parse JSON object, array, or value." fixed : when left unconnected, 
              thermistor value was too large for the database. Temperature capped to 70°C
19/04/2023b : [bug fix] Bluetooth authentification systematic timeout fixed
20/04/2023a : [bug fix] Bluetooth authentification failed when retry
20/04/2023b : SSID and WIFI password exchange
27/04/2023a : Wifi connects from exchanged SSID and Password
27/04/2023b : Queue recieve moved to read sensors more often Task_Database_Update_routine
              Wifi connection timeout : if ESP can't connect to wifi, it restart credidentials exchange via Bluetooth
03/05/2023a : Permanent storage implementation with preferences.h library
              Push button reading implementation
03/05/2023b : Repace sensor queues for global variables to facilitate access for multiple tasks
03/05/2023c : Database commands and threshold levels reading

*****************************************************/


/*****************************************************
Preprocessors
*****************************************************/
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <math.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "EPOT_lib.h"

/*****************************************************
Prototypes
*****************************************************/

void Task_Sensors_Update_routine( void * pvParameters );
//void Task_Sensors_Print_routine( void * pvParameters );
void Task_Database_Update_routine(void * pvParameters);

/*****************************************************
Global variables
*****************************************************/

char g_ssid[80] = {};
char g_password[80] = {};

TaskHandle_t Task_Sensors_Update_handle;
//TaskHandle_t Task_Sensors_Print_handle;
TaskHandle_t Task_Database_Update_handle;

float g_temperature = 0.0;
int g_light_level = 0;
float g_humidity_percentage = 0.0;
float g_water_level_percentage = 0.0;

float g_light_threshold = 0.0;
bool g_light_manual_auto = MANUAL;

float g_humidity_threshold = 0.0;
bool g_watering_manual_auto = MANUAL;

/*****************************************************
Main
*****************************************************/

void setup() {
  // ------------------ Serial Setup for debug ------------------
  Serial.begin(9600);

  Serial.println("--------- EPOT PROJET DEBUG OUTPUT --------");
  Serial.println("Firmware 03/05/23b (c)2023 Arnaud LE COSSEC");

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


  // ---------------- Startup procedure -------------

  bool has_credidentials_stored;

  // init storage for wifi credidentials
  Storage_init();           

  // Check for wifi credidentials in storage
  if(Storage_read(g_ssid, g_password)==FAIL || Button_read()==PRESSED){
    has_credidentials_stored = false;
    Serial.println("No credidentials saved");
  }
  else{
    Serial.println("credidentials found in memory");
    has_credidentials_stored = true;
    Serial.println(g_ssid);
    Serial.println(g_password);
  }

  // if ssid and password are not stored in memory, then proceed to initial configuration
  l_initial_configuration: // label
  if(has_credidentials_stored==false){    
    Serial.println("Initial configuration ...");
    // -- initialize Bluetooth and wait for connexion --
    Bluetooth_Setup();

    // -- Wait for hello signal from application --
    const char* Msg_Ref = "HELLO_EPOT_APP_0";
    char Msg[20] = {};
    if(Bluetooth_Receive_String(Msg, 20, BLUETOOTH_TIMEOUT)){
      Serial.println("[ERR] Hello timeout, disconnecting...");   // Fail condition
      Bluetooth.disconnect();
      goto l_initial_configuration;
    }
    if(compare_tables(Msg_Ref, Msg, 16)){
      Serial.println("[ERR] Hello mismatch, disconnecting...");   // Fail condition
      Bluetooth.disconnect();
      goto l_initial_configuration;
    }
 
    // -- Send hello signal for application --
    //Bluetooth.println("HELLO_EPOT_CMD_0");

    delay(100);

    // -- Exchange data

    Bluetooth.write(1);                                                 // indicate to the app an SSID request
    if(Bluetooth_Receive_String(g_ssid, 80)){
      Serial.println("[ERR] SSID request timeout, disconnecting...");   // Fail condition
      Bluetooth.disconnect();
      goto l_initial_configuration;
    }

    Serial.println(g_ssid);

    Bluetooth.write(2);                                                 // indicate to the app an password request
    if(Bluetooth_Receive_String(g_password, 80)){
      Serial.println("[ERR] Password request timeout, disconnecting...");   // Fail condition
      Bluetooth.disconnect();
      goto l_initial_configuration;
    }
  
    Serial.println(g_password);

    Bluetooth.write(3);                                                 // indicate to the app the end of configuration

    Bluetooth.disconnect();
    Bluetooth.end();
  }


  // ------------------ Setup wifi ------------------

  if(Wifi_Setup(g_ssid, g_password, WIFI_TIMEOUT)){
    Serial.println("[ERR] Wifi timeout, restarting credidentials exchange via Bluetooth...");   // Fail condition
    WiFi.disconnect();
    has_credidentials_stored = false;
    goto l_initial_configuration;
  }

  // if credidentials weren't saved and wifi connection is successful, then save credidentials
  if(has_credidentials_stored == false){
    Serial.println("New credidentials saved");
    Storage_write(g_ssid, g_password);
  }

  // terminate storage instance
  Storage_close();

  // ---------------- Setup database ----------------

  Database_init();
  // Write test
  Database_Write_int("/EPOT1/STATUS", 1);   // indicate to the app that the ESP has access to the database

  Database_Write_int("/EPOT1/ERROR_Count",Error_Count); // Write error log to database
  Database_Self_test_repport(Error_Count, Error_log);

  // ---------------- Setup Watchdog ----------------

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // ----------------- Create tasks -----------------


  xTaskCreatePinnedToCore(
                    Task_Sensors_Update_routine,        /* Task function. */
                    "Task_Sensors_Update_automation",   /* name of task. */
                    1024,                               /* Stack size of task */
                    NULL,                               /* parameter of the task */
                    1,                                  /* priority of the task */
                    &Task_Sensors_Update_handle,        /* Task handle to keep track of created task */
                    1);                                 /* pin task to core 1 */

//  xTaskCreatePinnedToCore(
//                    Task_Sensors_Print_routine,         /* Task function. */
//                    "Task_Sensors_Print_automation",    /* name of task. */
//                    1024,                               /* Stack size of task */
//                    NULL,                               /* parameter of the task */
//                    1,                                  /* priority of the task */
//                    &Task_Sensors_Print_handle,         /* Task handle to keep track of created task */
//                    1);                                 /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    Task_Database_Update_routine,       /* Task function. */
                    "Task_Database_Update_routine",     /* name of task. */
                    8192,                               /* Stack size of task */
                    NULL,                               /* parameter of the task */
                    1,                                  /* priority of the task */
                    &Task_Database_Update_handle,       /* Task handle to keep track of created task */
                    1);                                 /* pin task to core 1 */

}

void loop() {
  esp_task_wdt_reset();   // reset watchdog timer

  if (Firebase.isTokenExpired()){
    Firebase.refreshToken(&Database_config);
    Serial.println("Refresh token");
  }
}

/*****************************************************
Tasks
*****************************************************/

void Task_Sensors_Update_routine( void * pvParameters ){
  Serial.print("Sensor update task is online on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    // Read temperature
    float raw_temperature = Temp_Read();
    if(raw_temperature > 70.0){raw_temperature = 70.0;}     // Cap temperature to 70° to prevent inf value to try to access to the database

    g_temperature=raw_temperature;

    // Read light level
    float raw_light_level = Photo_Cell_Read();

    if(raw_light_level > 600000){raw_light_level = 0;}      // Dim hallway
    else if(raw_light_level > 70000){raw_light_level = 1;}  // Moonlit night
    else if(raw_light_level > 10000){raw_light_level = 2;}  // Dark room
    else if(raw_light_level > 1500){raw_light_level = 3;}   // Dark overcast day / Bright room
    else if(raw_light_level > 300){raw_light_level = 4;}    // Overcast day
    else {raw_light_level = 5;}                         // Full day light

    g_light_level = raw_light_level;
    
    // Read Humidity level
    g_humidity_percentage = Humidity_Read();
    
    // Read water level level
    float g_Water_level_percentage = (WATER_TANK_HEIGHT-Ultrasonic_Read()-WATER_TANK_SENSOR_SEPERATION)/WATER_TANK_HEIGHT; // convert sensor distance into percentage
  } 
}
/*
void Task_Sensors_Print_routine( void * pvParameters ){
  Serial.print("Sensor print task is online on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    
    Serial.print(g_temperature);
    Serial.print(" °C   ");
    Serial.print(g_light_level);
    Serial.print(" ohm    ");
    Serial.print(g_humidity_percentage);
    Serial.print(" % ");
    Serial.print(g_water_level_percentage);
    Serial.println(" % ");
    
  } 
}
*/
void Task_Database_Update_routine(void * pvParameters){
  Serial.print("Database update task is online on core ");
  Serial.println(xPortGetCoreID());

  int current_time, last_time = millis();
  
  for(;;){
    current_time = millis();
       
    if(current_time - last_time > 2000){


      Database_Read_bool("/EPOT1/CMD_Light_manual_auto", &g_light_manual_auto);
      Database_Read_float("/EPOT1/CMD_Light_threshold", &g_light_threshold);
      Database_Read_bool("/EPOT1/CMD_Watering_manual_auto", &g_watering_manual_auto);
      Database_Read_float("/EPOT1/CMD_Humidity_threshold", &g_humidity_threshold);
      
      Serial.println(g_light_threshold);
      Serial.println(g_light_manual_auto);
      Serial.println(g_humidity_threshold);
      Serial.println(g_watering_manual_auto);
      
      Serial.println("Database Commands Updated");
      
      // Update database
      Database_Write_float("EPOT1/SENSOR_Temperature", g_temperature);
      Database_Write_int("EPOT1/SENSOR_Light", g_light_level);
      Database_Write_float("EPOT1/SENSOR_Humidity", g_humidity_percentage);
      Database_Write_float("EPOT1/SENSOR_Water_level", g_water_level_percentage);
      Serial.println("Database Sensor Updated");
      
      //Serial.println("Hello !");
      last_time = current_time;
    }
  }
  
}
