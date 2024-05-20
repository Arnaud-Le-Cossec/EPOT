/*****************************************************
EPOT Project - ESP32 Library
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

*****************************************************/

/*****************************************************
Preprocessors
*****************************************************/
#include <Arduino.h>

#define POWER_RELAY 25

#define LED_PIN 12
#define LED_PWM_CH 1

#define BUTTON_PIN 14

#define ULT_TRIG_PIN 26
#define ULT_ECHO_PIN 27
#define WATER_TANK_HEIGHT 16 // cm
#define WATER_TANK_SENSOR_SEPERATION 5 // cm

#define AREF_PIN 36

#define THER_PIN 39
#define THER_REF 10000 // ohm
#define TEMP_OFFSET -5.0 //°C

#define PHOTO_PIN 34
#define PHOTO_REF 10000 // ohm

#define HUM_PIN 35
#define HUM_WET_VALUE 870// à définir (mesure)
#define HUM_DRY_VALUE 2640// à définir (mesure)

#define UV_CONTROL_PIN 15
#define LED_CONTROL_PIN 5
#define PUMP_CONTROL_PIN 18

#define WIFI_SSID "Nokia 5.1 Plus" //"iPhone de Axel"
#define WIFI_PASSWORD "cbf59af34a6e" //"axelnouy"

#define DATABASE_API_KEY "AIzaSyAoZcEjNppUw_e1oGDSBEqHTdy2MNe8C1Y"//"AIzaSyBPVg5J6stIq3hJN9WCjgGDYjIIZ2TUKWo"
#define DATABASE_URL "https://naturecollection-9fe2d-default-rtdb.firebaseio.com/EPOT1" //"https://epot-test-database-default-rtdb.firebaseio.com/EPOT1"
#define DATABASE_USER_EMAIL "lecossec.arnaud@gmail.com"
#define DATABASE_USER_PASSWORD "EPOTPASS"

#define DATABASE_UPDATE_RATE 5000 // milliseconds - interval between two updates

#define WDT_TIMEOUT 20 // seconds
#define BLUETOOTH_TIMEOUT 15000 // milliseconds
#define WIFI_TIMEOUT 15000 // milliseconds

#define FAIL 1
#define PASS 0

#define PRESSED 0
#define RELEASED 1

#define AUTO 0
#define MANUAL 1

#define STATE_ON 0
#define STATE_OFF 1

/*****************************************************
Global variables and constants
*****************************************************/

BluetoothSerial Bluetooth;   // Bluetooth object

// Database object
FirebaseData Database_object;
FirebaseData Database_stream;
FirebaseAuth Database_auth;
FirebaseConfig Database_config;

// We use a trigger to notify us when a database value changes 
//String Database_listenerPath = "EPOT1/CMD/";

Preferences preferences;  // Preferences object

float g_temperature = 0.0;
int g_light_level = 0;
float g_humidity_percentage = 0.0;
int g_water_level_percentage = 0.0;

int g_light_threshold = 0;
int g_light_manual_intensity = 0;
bool g_light_manual_auto = MANUAL;

int g_UV_threshold = 0;
int g_UV_manual_intensity = 0;
bool g_UV_manual_auto = MANUAL;

float g_humidity_threshold = 0.0;
int g_pump_auto_on_time = 5000;
bool g_pump_manual_on_off = STATE_OFF;
bool g_watering_manual_auto = MANUAL;

const char* Msg_Err_dist_not_responding =  "[ERROR] Distance sensor not responding (dist = 0cm)";
const char* Msg_Err_dist_read_err = "[ERROR] Distance sensor reading error (dist > 100cm)";
const char* Msg_Err_Aref_too_low = "[ERROR] Analog reference reading error - Too low (Aref < 3000)";
const char* Msg_Err_Aref_too_high = "[ERROR] Analog reference reading error - Too high (Aref > 4096)";
const char* Msg_Err_Temp_shorted = "[ERROR] Temperature sensor shorted (R_th=0; temp. < -20°C)";
const char* Msg_Err_Temp_not_connected = "[ERROR] Temperature sensor not connected (R_th=inf; temp. > 70°C)";
const char* Msg_Err_Photo_shorted = "[ERROR] Photo cell sensor shorted (R_ph=0)";
const char* Msg_Err_Photo_not_connected = "[ERROR] Photo cell sensor not connected (R_ph=inf)";
const char* Msg_Err_Hum_not_connected = "[ERROR] Humidity sensor not connected";
const char* Msg_Warn_Hum_not_calibrated = "[WARNING] Humidity sensor uncalibrated ";

bool StreamCallBackTimeout_status = false;

/*****************************************************
Prototypes
*****************************************************/

void LED_init(void);
void LED_set(uint8_t freq);

void Button_init(void);
bool Button_read(void);

void Ultrasonic_init(void);
float Ultrasonic_Read(void);

void Aref_init(void);

void Temp_init(void);
float Temp_Read(void);

void Photo_Cell_init(void);
float Photo_Cell_Read(void);

void Humidity_init(void);
float Humidity_Read(void);

bool Wifi_Setup(char* ssid, char* wifi_password, uint16_t Timeout);

void Bluetooth_Setup(void);
bool Bluetooth_Receive_String(char *TargetString, uint8_t StringSize, uint16_t Timeout);
bool Bluetooth_Receive_String(char *TargetString, uint8_t StringSize);

void Database_init(void);
bool Database_Read_int(const char* path, int* value);
bool Database_Read_float(const char* path, float* value);
bool Database_Read_bool(const char* path, bool* value);
bool Database_Write_float(const char* path,float value);
bool Database_Write_int(const char* path,int value);
bool Database_Write_string(const char* path,const char* value);
bool Database_Write_string(const char* path, String value);

void Storage_init(void);
void Storage_write(char* ssid, char* password);
bool Storage_read(char* ssid, char* password);
void Storage_close(void);

uint8_t Self_test(uint8_t *Error_log_table);
void Database_Self_test_repport(uint8_t Error_Count, uint8_t *Error_log);

bool compare_tables(const char* Table1, char* Table2, uint8_t Table_Size);
void clear_table(char* Table, uint8_t Table_Size);
void TrueBlutoothBuffer_Flush(void);

void Database_trigger_init(void);
void streamCallback(FirebaseStream data);
void streamTimeoutCallback(bool timeout);

/*****************************************************
Functions
*****************************************************/

void LED_init(void){
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(LED_PWM_CH, 60, 12);
  ledcAttachPin(LED_PIN, LED_PWM_CH);
  ledcWrite(LED_PWM_CH,2048);
}

void LED_set(uint8_t freq){
  ledcChangeFrequency(LED_PWM_CH,freq,12);
}

void Button_init(void){
  pinMode(BUTTON_PIN, INPUT);
}

bool Button_read(void){
  return digitalRead(BUTTON_PIN);
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

  return Temp + TEMP_OFFSET;
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

bool Wifi_Setup(char* ssid, char* wifi_password, uint16_t Timeout){
  // Connect to Wi-Fi network with SSID and password
  Serial.print("[WIFI] Connecting to ");
  Serial.println(ssid);

  uint16_t start_time = millis();

  WiFi.begin(ssid, wifi_password);
  while ((WiFi.status() != WL_CONNECTED) && (millis() < (start_time + Timeout))) {
    delay(500);
    Serial.print(".");
  }

  if(millis() >= (start_time + Timeout)){
    //Serial.println("[ERROR] wifi connection timeout");
    return 1;
  }

  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return 0;
}

void Bluetooth_Setup(void){
  Bluetooth.begin("EPOT-2023"); //Bluetooth device name
  Serial.println("[BLUETOOTH] The device started under the name 'EPOT-2023' ");

  const uint8_t LED_patern[] = {100,0,100,0,0,0};
  uint8_t LED_patern_index = 0;

  while(Bluetooth.connected() != true){
    //LED_set(LED_patern[LED_patern_index]);
    //LED_patern_index = (LED_patern_index + 1)%6;
    delay(500);
    Serial.print(".");
  }
  Serial.println("/n[BLUETOOTH]Connected");
}

bool Bluetooth_Receive_String(char *TargetString, uint8_t StringSize, uint16_t Timeout){
  uint8_t i = 0;
  clear_table(TargetString, StringSize);

  uint16_t start_time = millis();

  do{
    if(Bluetooth.available()){
        TargetString[i] = Bluetooth.read();
        Serial.print(TargetString[i]);
        i++;
    }
  }while((millis() < (start_time + Timeout)) && Bluetooth.connected() && (i <= StringSize) && (TargetString[i-1] != 0x0D)); // 0x0D is carriage return
  
  TargetString[i-1]=0;
  TrueBlutoothBuffer_Flush();

  if(millis() >= (start_time + Timeout)){
    Serial.println("[ERROR] Bluetooth string reception timeout");
    return 1;
  }
  if(Bluetooth.connected() == false){
    Serial.println("[ERROR] Bluetooth disconnected");
    return 1;
  }
  return 0;
}

bool Bluetooth_Receive_String(char *TargetString, uint8_t StringSize){
  uint8_t i = 0;
  clear_table(TargetString, StringSize);
  
  do{
    if(Bluetooth.available()){
        TargetString[i] = Bluetooth.read();
        Serial.print(TargetString[i]);
        i++;
    }
  }while(Bluetooth.connected() && (i <= StringSize) && (TargetString[i-1] != 0x0D)); // 0x0D is carriage return
  
  TargetString[i-1]=0;
  TrueBlutoothBuffer_Flush();

  if(Bluetooth.connected() == false){
    Serial.println("[ERROR] Bluetooth disconnected");
    return 1;
  }
  return 0;
}

void Database_init(void){
  

  Database_config.api_key = DATABASE_API_KEY;
  Database_config.database_url = DATABASE_URL;

  Database_auth.user.email = DATABASE_USER_EMAIL;
  Database_auth.user.password = DATABASE_USER_PASSWORD;
  /*
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  */
  
  // reconnect wifi if lost connection
  Firebase.reconnectWiFi(true);

  Database_config.token_status_callback = tokenStatusCallback;
  Database_config.max_token_generation_retry = 5;

  // Initialize Firebase with the config and authentification credidentials
  Firebase.begin(&Database_config, &Database_auth);


  Serial.println("Waiting for database authentification");
  while ((Database_auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  String uid = Database_auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);
}

bool Database_Read_int(const char* path, int* value){
  if(Firebase.RTDB.getInt(&Database_object, path)){
    *value = Database_object.intData();
    return 0; // Read success
  }
   // Read fail
  Serial.print("[ERROR] Database read operation failed : ");
  Serial.println(Database_object.errorReason());
  return 1; 
}

bool Database_Read_float(const char* path, float* value){
  if(Firebase.RTDB.getFloat(&Database_object, path)){
    *value = Database_object.floatData();
    return 0; // Read success
  }
   // Read fail
  Serial.print("[ERROR] Database read operation failed : ");
  Serial.println(Database_object.errorReason());
  return 1; 
}

bool Database_Read_bool(const char* path, bool* value){
  if(Firebase.RTDB.getFloat(&Database_object, path)){
    *value = Database_object.boolData();
    return 0; // Read success
  }
  // Read fail
  Serial.print("[ERROR] Database read operation failed : ");
  Serial.println(Database_object.errorReason());
  return 1; 
}

bool Database_Write_float(const char* path,float value){
  if(Firebase.RTDB.setFloat(&Database_object, path, value)){
    return 0; // Write success
  }
   // Write fail
  Serial.println("[ERROR] Database write operation failed");
  Serial.println(Database_object.errorReason());
  return 1; 
}

bool Database_Write_int(const char* path,int value){
  if(Firebase.RTDB.setInt(&Database_object, path, value)){
    return 0; // Write success
  }
   // Write fail
  Serial.println("[ERROR] Database write operation failed");
  Serial.println(Database_object.errorReason());
  return 1; 
}

bool Database_Write_string(const char* path,const char* value){
  if(Firebase.RTDB.setString(&Database_object, path, value)){
    return 0; // Write success
  }
   // Write fail
  Serial.println("[ERROR] Database write operation failed");
  Serial.println(Database_object.errorReason());
  return 1; 
}

bool Database_Write_string(const char* path, String value){
  if(Firebase.RTDB.setString(&Database_object, path, value)){
    return 0; // Write success
  }
   // Write fail
  Serial.println("[ERROR] Database write operation failed");
  Serial.println(Database_object.errorReason());
  return 1; 
}

void Storage_init(void){
  preferences.begin("EPOT_Storage", false);  // Create/Open storage namespace "EPOT_Storage" in read/write mode
}

void Storage_write(char* ssid, char* password){
  preferences.putString("ssid",ssid);
  preferences.putString("password", password);
}

bool Storage_read(char* ssid, char* password){
  String temp_ssid = preferences.getString("ssid", ""); 
  String temp_password = preferences.getString("password", "");
  if (temp_ssid == "" || temp_password == ""){
    Serial.println("[INFO] No values saved for ssid or password. Begining new setup via Bluetooth");
    return 1;
  }

  strcpy(ssid, temp_ssid.c_str());
  strcpy(password, temp_password.c_str());
  //ssid = temp_ssid.c_str();
  //password = temp_password.c_str();

  return 0;
}

void Storage_close(void){
  preferences.end();
}

uint8_t Self_test(uint8_t *Error_log_table){

  // Clear error log table
  for(uint8_t i=0; i<11; i++){
    Error_log_table[i] = 0;
  }
  /*  index 0 : General error
      index 1 : [Distance sensor not responding]
      index 2 : [Distance sensor reading error]
      index 3 : [Analog reference reading error - Too low]
      index 4 : [Analog reference reading error - Too high]
      index 5 : [Temperature sensor shorted]
      index 6 : [Temperature sensor not connected]
      index 7 : [Photo cell sensor shorted]
      index 8 : [Photo cell sensor not connected]
      index 9 : [Humidity sensor not connected]
      index 10 : [Humidity sensor uncalibrated]
  */



  Serial.println("Self-test running ... ");

  uint8_t Error_Count = 0;

  // Check ultrasonic sensor
  if(Ultrasonic_Read()==0){Serial.println(Msg_Err_dist_not_responding);Error_Count++;Error_log_table[1]=1;}
  if(Ultrasonic_Read()>100){Serial.println(Msg_Err_dist_read_err);Error_Count++;Error_log_table[2]=1;}
  // Check Aref
  if(analogRead(AREF_PIN)<3000){Serial.println(Msg_Err_Aref_too_low);Error_Count++;Error_log_table[3]=1;}
  if(analogRead(AREF_PIN)>4096){Serial.println(Msg_Err_Aref_too_high);Error_Count++;Error_log_table[4]=1;}
  // Check Temperature sensor
  if(Temp_Read()>70){Serial.println(Msg_Err_Temp_shorted);Error_Count++;Error_log_table[5]=1;}
  if(Temp_Read()<-20){Serial.println(Msg_Err_Temp_not_connected);Error_Count++;Error_log_table[6]=1;}
  // Check Photo cell
  if(Photo_Cell_Read()<1){Serial.println(Msg_Err_Photo_shorted);Error_Count++;Error_log_table[7]=1;}
  if(Photo_Cell_Read()>10000000){Serial.println(Msg_Err_Photo_not_connected);Error_Count++;Error_log_table[8]=1;}
  // Check Humididy sensor
  if(analogRead(HUM_PIN)<100){Serial.println(Msg_Err_Hum_not_connected);Error_Count++;Error_log_table[9]=1;}
  if(analogRead(HUM_PIN)<(HUM_WET_VALUE-100) || analogRead(HUM_PIN)>(HUM_DRY_VALUE+100)){Serial.print(Msg_Warn_Hum_not_calibrated);Serial.println(analogRead(HUM_PIN));Error_Count++;Error_log_table[10]=1;}

  Serial.print(Error_Count);
  Serial.println(" errors found");

  return Error_Count;
}

void Database_Self_test_repport(uint8_t Error_Count, uint8_t *Error_log){
  String Output_string = "";
  char last_index = 0;
  
  if(Error_log[1]){ Output_string += Msg_Err_dist_not_responding; }
  if(Error_log[2]){ Output_string += Msg_Err_dist_read_err;}
  if(Error_log[3]){ Output_string += Msg_Err_Aref_too_low;}
  if(Error_log[4]){ Output_string += Msg_Err_Aref_too_high;}
  if(Error_log[5]){ Output_string += Msg_Err_Temp_shorted;}
  if(Error_log[6]){ Output_string += Msg_Err_Temp_not_connected;}
  if(Error_log[7]){ Output_string += Msg_Err_Photo_shorted;}
  if(Error_log[8]){ Output_string += Msg_Err_Photo_not_connected;}
  if(Error_log[9]){ Output_string += Msg_Err_Hum_not_connected;}
  if(Error_log[10]){ Output_string += Msg_Warn_Hum_not_calibrated;}

  Database_Write_int("EPOT1/ERROR/ERROR_Count", Error_Count);
  Database_Write_string("EPOT1/ERROR/ERROR_Description", Output_string);

}

bool compare_tables(const char* Table1, char* Table2, uint8_t Table_Size){
  bool mismatch = false;
  uint8_t i=0;
  do{
    //Serial.print(Table1[i]);
    //Serial.print(" ");
    //Serial.print(Table2[i]);
    if(Table1[i] != Table2[i]){mismatch = true;}
    //Serial.println((mismatch)?" Fail":" Success");
    i++;
  }while ((i <= Table_Size) && (Table1[i]!=0x0D) && (Table2[i]!=0x0D)); // Compare until maximum size is reached, or carriage return (0x0D)
  
	return mismatch;
}

void clear_table(char* Table, uint8_t Table_Size){
  for(uint8_t i=0; i<Table_Size; i++ ){
    Table[i] = 0;
  }
}

void TrueBlutoothBuffer_Flush(void){
  uint8_t dumbvalue = 0;
  while (Bluetooth.available())
  {
    dumbvalue = Bluetooth.read();
  }
}

void Database_trigger_init(void){
  // Streaming (whenever data changes on a path)
  // Begin stream on a database path --> board1/outputs/digital
  if (!Firebase.RTDB.beginStream(&Database_stream, "EPOT1/CMD/"));
    Serial.printf("stream begin error, %s\n\n", Database_stream.errorReason().c_str());
  
  //Assign a calback function to run when it detects changes on the database
  Firebase.RTDB.setStreamCallback(&Database_stream, streamCallback, streamTimeoutCallback);

}

void streamCallback(FirebaseStream data){

  Serial.println("Stream Data...");

  String streamPath = String(data.dataPath());
  Serial.println(streamPath);

  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_json){
    FirebaseJson json = data.to<FirebaseJson>();

    // To iterate all values in Json object
    size_t count = json.iteratorBegin();

    g_humidity_threshold = json.valueAt(0).key.toFloat();
    g_pump_manual_on_off = (json.valueAt(1).key.toInt())?1:0;
    g_pump_auto_on_time = json.valueAt(2).key.toInt();
    g_watering_manual_auto = (json.valueAt(3).key.toInt())?1:0;

    g_light_threshold = json.valueAt(4).key.toInt();
    g_light_manual_intensity = json.valueAt(5).key.toInt();
    g_light_manual_auto = (json.valueAt(6).key.toInt())?1:0;

    g_UV_threshold = json.valueAt(7).key.toInt();
    g_UV_manual_intensity = json.valueAt(8).key.toInt();
    g_UV_manual_auto = (json.valueAt(9).key.toInt())?1:0;
  
    json.iteratorEnd(); // required for free the used memory in iteration (node data collection)
  }

  switch (streamPath[1])
  {
  case '1':
    /* CMD_Humidity_threshold */
    g_humidity_threshold = data.floatData();
    Serial.println(g_humidity_threshold);
    break;
  
  case '2':
    /* CMD_Pump_manual_on_off */
    g_pump_manual_on_off = data.boolData();
    Serial.println(g_pump_manual_on_off);
    break;

  case '3':
    /* CMD_Pump_manual_on_off */
    g_pump_auto_on_time = data.intData();
    Serial.println(g_pump_auto_on_time);
    break;

  case '4':
    /* CMD_Watering_manual_auto */
    g_watering_manual_auto = data.boolData();
    Serial.println(g_watering_manual_auto);
    break;

  case '5':
    /* CMD_Light_threshold */
    g_light_threshold = data.intData();
    Serial.println(g_light_threshold);
    break;

  case '6':
    /* CMD_Light_manual_intensity */
    g_light_manual_intensity = data.intData();
    Serial.println(g_light_manual_intensity);
    break;

  case '7':
    /* CMD_Light_manual_auto */
    g_light_manual_auto = data.boolData();
    Serial.println(g_light_manual_auto);
    break;

  case '8':
    /* CMD_UV_threshold */
    g_UV_threshold = data.intData();
    Serial.println(g_UV_threshold);
    break;
  
  case '9':
    /* CMD_UV_manual_intensity */
    g_UV_manual_intensity = data.intData();
    Serial.println(g_UV_manual_intensity);
    break;

  case 'a':
    /* CMD_UV_manual_auto */
    g_UV_manual_auto = data.boolData();
    Serial.println(g_UV_manual_auto);
    break;

  default:
    break;
  }

  //Serial.println(data.streamPath());
  //Serial.println(data.dataPath());
  //Serial.println(data.dataType());

}

void streamTimeoutCallback(bool timeout){
  if (timeout){
    Serial.println("stream timeout\n");
    StreamCallBackTimeout_status = true;
  }
}