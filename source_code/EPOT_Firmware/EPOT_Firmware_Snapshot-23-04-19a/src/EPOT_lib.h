/*****************************************************
EPOT Project - ESP32 Library
(c)2023 Arnaud LE COSSEC
(c)2023 College Lionel Groulx
*****************************************************/

/*****************************************************
Change Notes : 
30/03/2023a : File creation
31/03/2023a : Watchdog implementation
03/04/2023a : Sensors to database
              Ledc library error fixed
04/04/2023a : Error log to database
*****************************************************/

/*****************************************************
Preprocessors
*****************************************************/
#include <Arduino.h>

#define LED_PIN 12
#define LED_PWM_CH 1

#define ULT_TRIG_PIN 26
#define ULT_ECHO_PIN 27
#define WATER_TANK_HEIGHT 15 // cm
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


#define WIFI_SSID "Nokia 5.1 Plus" //"iPhone de Axel"
#define WIFI_PASSWORD "cbf59af34a6e" //"axelnouy"

#define DATABASE_API_KEY "AIzaSyBPVg5J6stIq3hJN9WCjgGDYjIIZ2TUKWo"
#define DATABASE_URL "https://epot-test-database-default-rtdb.firebaseio.com/EPOT1"
#define DATABASE_USER_EMAIL "lecossec.arnaud@gmail.com"
#define DATABASE_USER_PASSWORD "EPOTPASS"

#define DATABASE_UPDATE_RATE 5000 // milliseconds - interval between two updates

#define WDT_TIMEOUT 10 // seconds
#define BLUETOOTH_TIMEOUT 30000 // milliseconds

/*****************************************************
Global variables and constants
*****************************************************/

BluetoothSerial Bluetooth;   // Bluetooth object

FirebaseData Database_object;   // Database object
FirebaseAuth Database_auth;
FirebaseConfig Database_config;

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

void Wifi_Setup(void);

void Bluetooth_Setup(void);
bool Bluetooth_Receive_String(char *TargetString, uint8_t StringSize, uint16_t Timeout);

void Database_init(void);
bool Database_Read_int(const char* path, int* value);
bool Database_Read_float(const char* path, float* value);
bool Database_Write_float(const char* path,float value);
bool Database_Write_int(const char* path,int value);
bool Database_Write_string(const char* path,const char* value);
bool Database_Write_string(const char* path, String value);

uint8_t Self_test(uint8_t *Error_log_table);
void Database_Self_test_repport(uint8_t Error_Count, uint8_t *Error_log);

bool compare_tables(const char* Table1, char* Table2, uint8_t Table_Size);

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

void Wifi_Setup(void){
  // Connect to Wi-Fi network with SSID and password
  Serial.print("[WIFI] Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void Bluetooth_Setup(void){
  Bluetooth.begin("EPOT-2023"); //Bluetooth device name
  Serial.println("[BLUETOOTH] The device started under the name 'EPOT-2023' ");

  const uint8_t LED_patern[] = {100,0,100,0,0,0};
  uint8_t LED_patern_index = 0;

  while(Bluetooth.connected() != true){
    //LED_set(LED_patern[LED_patern_index]);
    //LED_patern_index = (LED_patern_index + 1)%6;
    Serial.print(".");
  }
}

bool Bluetooth_Receive_String(char *TargetString, uint8_t StringSize, uint16_t Timeout){
  uint16_t start_time = millis();
  uint8_t i = 0;
  while((millis() < (start_time + Timeout)) && Bluetooth.connected()){
    if(Bluetooth.available()  && i < StringSize ){
        TargetString[i] = Bluetooth.read();
        i++;
    }
  }
  if(millis() > (start_time + Timeout)){
    Serial.println("[ERROR] Bluetooth string reception timeout");
    return 1;
  }
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
    *value = Database_object.floatData();
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

void EEPROM_write(void){
  
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

  Database_Write_int("EPOT1/ERROR_Count", Error_Count);
  Database_Write_string("EPOT1/ERROR_Description", Output_string);

}

bool compare_tables(const char* Table1, char* Table2, uint8_t Table_Size){
  bool mismatch = false;
	for(uint8_t i; i<Table_Size; i++){
		if(Table1[i] != Table2[i]){mismatch = true;}
	}
	return mismatch;
}