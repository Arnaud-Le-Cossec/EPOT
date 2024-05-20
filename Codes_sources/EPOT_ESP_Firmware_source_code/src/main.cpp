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
04/05/2023a : Automation - Force run/stop
              LED GPIO 17 and UV GPIO16 are not connected on VROVER-E. Replaced for GPIO 5 and 15 respectively
08/05/2023a : Database triggers
09/05/2023a : Update commands and levels from database triggers
09/05/2023b : Automation - Auto mode
10/05/2023a : Database sensor update only when sensor value changed
              if button pressed durring operation, hard reset is performed
15/05/2023a : RTOS Removed. RTOS handles memory in a weird way : random crashes due to memory allocation errors. 
15/05/2023b : Database update at startup
18/05/2023a : [BUG FIX] after 1h of runtime, firebase wants to refresh authentification tokens, but fails due to lack of memory
19/05/2023a : Delay added after firebase stream reconnection. Watchdog timeout extended to accomodate this delay.
20/05/2023a : LED to show CPU activity in the loop
22/05/2023a : Added relay output to cut 5v at startup
              Ultrasonic sensor measure overhaul
23/05/2023a : Pump automation overhaul
28/05/2023a : Commenting

                           

*****************************************************/


/*****************************************************
Preprocessors
*****************************************************/
#include <esp_task_wdt.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
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



/*****************************************************
Global variables
*****************************************************/

char g_ssid[80] = {};
char g_password[80] = {};

float last_temperature = 0.0;
int last_light_level = 0;
float last_humidity_percentage = 0.0;
int last_water_level_percentage = 0;
int water_level_percentage_test_a = 0;
int water_level_percentage_test_b = 0;
uint water_level_test_phase = 0;

int last_light_manual_intensity = 100;
int last_UV_manual_intensity = 100;
bool last_pump_state = STATE_OFF;

int current_time, last_time, last_watering, last_ultrasonic_update = millis();



/*****************************************************
Main
*****************************************************/

void setup() {
  heap_caps_free(NULL);

  
  // ------------------ Serial Setup for debug ------------------
  Serial.begin(9600);

  Serial.println("--------- EPOT PROJET DEBUG OUTPUT --------");
  Serial.println("Firmware 28/05/23a (c)2023 Arnaud LE COSSEC");

  Serial.println(heap_caps_get_free_size(MALLOC_CAP_8BIT));

  // ------------------ Pin Setup ------------------

  pinMode(LED_CONTROL_PIN, OUTPUT);
  digitalWrite(LED_CONTROL_PIN, HIGH);
  pinMode(UV_CONTROL_PIN, OUTPUT);
  digitalWrite(UV_CONTROL_PIN, HIGH);
  pinMode(PUMP_CONTROL_PIN, OUTPUT);
  digitalWrite(PUMP_CONTROL_PIN, HIGH);

  pinMode(POWER_RELAY, OUTPUT);
  digitalWrite(POWER_RELAY, LOW);

  LED_init();
  Ultrasonic_init();
  Aref_init();
  Temp_init();
  Photo_Cell_init();
  Humidity_init();

  // ---------------- visual check ------------------
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
    
  }

  Bluetooth.end();

  esp_bluedroid_disable();                                              // Deactivate and release Bluetooth memory to save RAM
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);

  // ------------------ Setup wifi ------------------

  if(Wifi_Setup(g_ssid, g_password, WIFI_TIMEOUT)){
    Serial.println("[ERR] Wifi timeout, restarting credidentials exchange via Bluetooth...");   // Fail condition
    WiFi.disconnect();
    has_credidentials_stored = false;
    goto l_initial_configuration;                                                               // Credidentials may be wrong, go to initial configuration
  }

  // if credidentials weren't saved and wifi connection is successful, then save credidentials
  if(has_credidentials_stored == false){
    Serial.println("New credidentials saved");
    Storage_write(g_ssid, g_password);
  }

  // terminate storage instance
  Storage_close();

  // ---------------- ENABLE 5v ----------------

  delay(1000);
  digitalWrite(POWER_RELAY, HIGH);          // enable +5v rail for LEDs and ultrasonic sensors
  delay(1000);


  // ---------------- Setup database ----------------

  Database_init();                          // Initialize database connexion
  // Write test
  Database_Write_int("/EPOT1/STATUS", 1);   // indicate to the app that the ESP has access to the database

  Database_Self_test_repport(Error_Count, Error_log);   // Repport errors to the database

  //delay(500);

  Database_trigger_init();

  delay(6000);

  // -------------- Read initial values ----------------

  Database_Read_float("/EPOT1/CMD/1_CMD_Humidity_threshold", &g_humidity_threshold);
  Database_Read_bool("/EPOT1/CMD/2_CMD_Pump_manual_on_off", &g_pump_manual_on_off);
  Database_Read_int("/EPOT1/CMD/3_CMD_Pump_auto_on_time", &g_pump_auto_on_time);
  Database_Read_bool("/EPOT1/CMD/4_CMD_Watering_manual_auto", &g_watering_manual_auto);
  
  Database_Read_int("/EPOT1/CMD/5_CMD_Light_threshold", &g_light_threshold);
  Database_Read_int("/EPOT1/CMD/6_CMD_Light_manual_intensity", &g_light_manual_intensity);
  Database_Read_bool("/EPOT1/CMD/7_CMD_Light_manual_auto", &g_light_manual_auto);
  
  Database_Read_int("/EPOT1/CMD/8_CMD_UV_threshold", &g_UV_threshold);
  Database_Read_int("/EPOT1/CMD/9_CMD_UV_manual_intensity", &g_UV_manual_intensity);
  Database_Read_bool("/EPOT1/CMD/a_CMD_UV_manual_auto", &g_UV_manual_auto);

  // ---------------- Setup Watchdog ----------------

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // ---------------- Status LED --------------------

  ledcDetachPin(LED_PIN);   // From now on, the status LED will be driven manualy

}

void loop() {
  esp_task_wdt_reset();   // reset watchdog timer

  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Flip LED state : if the CPU crashes, the LED stops

  current_time = millis(); // update current time for time-based tasks

  if(!Firebase.ready()){    // Firebase.ready() updates the database authentification tokens
    digitalWrite(LED_PIN, LOW);
  }

  // Old token refresh method : uses too much memory
  //if (Firebase.isTokenExpired()){
  //  Firebase.refreshToken(&Database_config);
  //  Serial.println("Refresh token");
  //}

  if(StreamCallBackTimeout_status == true){ // If the database stream callback fails, terminate it and create a new one
    Firebase.RTDB.endStream(&Database_stream);
    Firebase.RTDB.removeStreamCallback(&Database_stream);
    Database_trigger_init();
    delay(6000);
    StreamCallBackTimeout_status = false;
  }

  if(Button_read()==PRESSED){               // reset ESP if the button is pressed. Note for the future : put this into an interrupt
    ESP.restart();
  }

  
  /*****************************************************
  SENSOR UPDATE
  *****************************************************/

  // Read temperature
  float raw_temperature = Temp_Read();
  if(raw_temperature > 70.0){raw_temperature = 70.0;}     // Cap temperature to 70° to prevent inf value to try to access to the database
  g_temperature=round(raw_temperature*10)/10;             // allow 0.1 resolution

  // Read light level
  float raw_light_level = Photo_Cell_Read();               // Photo resistor is not a precise lux meter, use stages instead
  if(raw_light_level > 600000){raw_light_level = 0;}       // Dim hallway
  else if(raw_light_level > 70000){raw_light_level = 20;}  // Moonlit night
  else if(raw_light_level > 10000){raw_light_level = 40;}  // Dark room
  else if(raw_light_level > 1500){raw_light_level = 60;}   // Dark overcast day / Bright room
  else if(raw_light_level > 300){raw_light_level = 80;}    // Overcast day
  else {raw_light_level = 100;}                            // Full day light
  g_light_level = raw_light_level;
  
  // Read Humidity level
  g_humidity_percentage = round(Humidity_Read());
  
  // Read water level level
  // We wait 10s between each measures to avoid ultrasonic bouncing in the reservoir messing up with the sensor
  // We still face inconsistencies between measures, so we take two samples, and if they are identical, they are probably correct
  if(current_time - last_ultrasonic_update > 10000){      
    if(water_level_test_phase == 0){                      
      water_level_percentage_test_a = round(Ultrasonic_Read());
      //Serial.print("p1 ");
      //Serial.println(water_level_percentage_test_a);
      water_level_test_phase = 1;
      last_ultrasonic_update = current_time;
    }
    else if(water_level_test_phase == 1){
      water_level_percentage_test_b = round(Ultrasonic_Read());
      //Serial.print("p2 ");
      //Serial.println(water_level_percentage_test_b);
      if(water_level_percentage_test_a == water_level_percentage_test_b){
        float result = ((WATER_TANK_HEIGHT+WATER_TANK_SENSOR_SEPERATION)-water_level_percentage_test_a);
        g_water_level_percentage = round((result/WATER_TANK_HEIGHT)*100); // accuracy is not needed for the tank level
        Serial.println(g_water_level_percentage);
      }
      water_level_test_phase = 0;
      last_ultrasonic_update = current_time;
    }
  }

  /*****************************************************
  DATABASE UPDATE
  *****************************************************/

  if(current_time - last_time > 5000){      // Sensor values are sent to the database every 5s to save bandwidth
     
    // Update database
    if(g_temperature != last_temperature){  // Sensor are only updated of they are different that the previous measure
      Database_Write_float("EPOT1/SENSOR_Temperature", g_temperature);
      last_temperature = g_temperature;
    }
    if(g_light_level != last_light_level){  // Sensor are only updated of they are different that the previous measure
      Database_Write_int("EPOT1/SENSOR_Light", g_light_level);
      last_light_level = g_light_level;
    }
    if(g_humidity_percentage != last_humidity_percentage){  // Sensor are only updated of they are different that the previous measure
      Database_Write_float("EPOT1/SENSOR_Humidity", g_humidity_percentage);
      last_humidity_percentage = g_humidity_percentage;
    }
    if(g_water_level_percentage != last_water_level_percentage){  // Sensor are only updated of they are different that the previous measure
      Database_Write_int("EPOT1/SENSOR_Water_level", g_water_level_percentage);
      last_water_level_percentage = g_water_level_percentage;
    }
    
    Serial.println("Database Sensor Updated");
    
    last_time = current_time;

  }
  /*****************************************************
  LIGHT AUTOMATION
  *****************************************************/

  if(g_light_manual_auto == MANUAL){    
    // Manual mode
    if(g_light_manual_intensity != last_light_manual_intensity){
      analogWrite(LED_CONTROL_PIN, (255-g_light_manual_intensity*2.5));
      last_light_manual_intensity = g_light_manual_intensity;
    }
  }
  else{
    // Auto mode
    if(g_light_level <= g_light_threshold){ 
      analogWrite(LED_CONTROL_PIN, (255-g_light_manual_intensity*2.5));
    }
    else{
      analogWrite(LED_CONTROL_PIN, 255);
    }
  }

  /*****************************************************
  UV AUTOMATION
  *****************************************************/

  if(g_light_manual_auto == MANUAL){ //if(g_UV_manual_auto == MANUAL){ -> UV manual mode is now tied the normal LED manual mode
    // Manual mode
    if(g_UV_manual_intensity != last_UV_manual_intensity){
      analogWrite(UV_CONTROL_PIN, (255-g_UV_manual_intensity*2.5));
      last_UV_manual_intensity = g_UV_manual_intensity;
    }
  }
  else{
    // Auto mode
    if(g_light_level <= g_UV_threshold){
      analogWrite(UV_CONTROL_PIN, (255-g_UV_manual_intensity*2.5));
    }
    else{
      analogWrite(UV_CONTROL_PIN, 255);
    }
  }

  /*****************************************************
  WATERING AUTOMATION
  *****************************************************/

  if(g_watering_manual_auto == MANUAL){
    // Wattering manual mode
    digitalWrite(PUMP_CONTROL_PIN, !g_pump_manual_on_off); 
  }else
  {
    //Wattering auto mode : if current soil humidity is below the set threshold, 
    //then the pump is activated for 5s every minute to let the soil absorb the water
    if(g_humidity_percentage < g_humidity_threshold){
      if((current_time - last_watering > 60000) && (last_pump_state == STATE_OFF) && (g_water_level_percentage > 5)){
        digitalWrite(PUMP_CONTROL_PIN, STATE_ON);
        last_pump_state = STATE_ON;
        last_watering = current_time;
        Serial.println("ON");
      }
      else if((current_time - last_watering > g_pump_auto_on_time) && (last_pump_state == STATE_ON)){
        digitalWrite(PUMP_CONTROL_PIN, STATE_OFF);
        last_pump_state = STATE_OFF;
        last_watering = current_time;
        Serial.println("OFF");
      }
    }
    else{
      digitalWrite(PUMP_CONTROL_PIN, STATE_OFF);
      last_pump_state = STATE_OFF;
      Serial.println("F OFF");
    }
  }    

}

