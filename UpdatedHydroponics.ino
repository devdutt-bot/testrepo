#include <WiFi.h>
#include <FirebaseESP32.h>    //firebase library version already addded

//#define USE_PULSE_OUT   //to be defined compulasrily before include ph library

#ifdef USE_PULSE_OUT            //including libraries for ph sensor
#include "ph_iso_grav.h"
Gravity_pH_Isolated pH = Gravity_pH_Isolated(34);
#else
#include "ph_grav.h"
Gravity_pH pH = Gravity_pH(34);
#endif

#include<EEPROM.h>

#include "BluetoothSerial.h"
#include <Wire.h>                //enable I2C.

#include<DHT.h>     //library for dht sensor

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)             //used for enabling bluetooth
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#define FIREBASE_HOST "https://semi-final-ebd63-default-rtdb.firebaseio.com"

/** The database secret is obsoleted, please use other authentication methods,
   see examples in the Authentications folder.
*/
#define FIREBASE_AUTH "73FchRYiQgAAQaXqoyLC6577wlAvBNLSU4InHmBx"

//this section contains all the pins

#define Motor1_plus 13
#define Motor2_plus 12
#define Motor3_plus 14
#define Motor4_plus 27
#define Motor1_minus 26
#define Motor2_minus 25
#define Motor3_minus 33
#define Motor4_minus 32
#define phPin 19
#define address 100              //default I2C ID number for EZO EC Circuit.

//till here
#define DHTTYPE DHT22

int BTcheck=0;

char ssid[50]=""     ;      //wifi pass and ssid defined here
char password[50]="" ;

int checkWifi = 0;

String strinn;                    //

uint8_t user_bytes_received = 0;      //for parse function for seperating values to be printed
const uint8_t bufferlen = 32;
char user_data[bufferlen];

int mtime1 = 10, mtime2 = 10, mtime3 = 10, mtime4 = 10;   //time for each motor
double time1 = 4, time2 = 4, time3 = 4, time4 = 4, time5 = 4;   //time for each sensor

int digli = -1;   //used as the corresponding minus pin for motor when turned on

FirebaseData fbdo;

char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer = 0; //we need to know how many characters have been received.
byte serial_event = 0;           //a flag to signal when data has been received from the pc/mac/other.
byte code = 0;                   //used to hold the I2C response code.
char ec_data[32];                //we make a 32 byte character array to hold incoming data from the EC circuit.
byte in_char = 0;                //used as a 1 byte buffer to store inbound bytes from the EC Circuit.
byte i = 0;                      //counter used for ec_data array.
int time_ = 570;                 //used to change the delay needed depending on the command sent to the EZO Class EC Circuit.

char *ec;                        //char pointer used in string parsing.
char *tds;                       //char pointer used in string parsing.
char *sal;                       //char pointer used in string parsing.
char *sg;                        //char pointer used in string parsing.

float ec_float;                  //float var used to hold the float value of the conductivity.
float tds_float;                 //float var used to hold the float value of the TDS.
float sal_float;                 //float var used to hold the float value of the salinity.
float sg_float;                  //float var used to hold the float value of the specific gravity.

String c = "a";         //used for changing between bluetooth and wifi once accessible
String getVals = "";
String getValsTime = "";    //get bt value
String getValsSpeed = ""; //get bt value
String getValsType = "";
const int trigPin = 35;      //pot pin used for the fifth  sensor to get the variable resistance
const int echoPin = 23;   //
int potValue = 0;

int checkman = 0;       //this man checks whether there is a bluetooth connection or not

int speed1 = 0, speed2 = 0, speed3 = 0, speed4 = 0; //for motor speed
BluetoothSerial SerialBT;

DHT dht(phPin, DHTTYPE);
float h = 1.0;    //variable for humididty
float t = 1.0;    //variable for temperature

void parse_cmd(String string) {         //used for caliberating
  
  if (string == "CAL,7") {
    pH.cal_mid();
    Serial.println("MID CALIBRATED");
  }
  else if (string == "CAL,4") {
    pH.cal_low();
    Serial.println("LOW CALIBRATED");
  }
  else if (string == "CAL,10") {
    pH.cal_high();
    Serial.println("HIGH CALIBRATED");
  }
  else if (string=="CAL,CLEAR") {
    pH.cal_clear();
    Serial.println("CALIBRATION CLEARED");
  }
}



void WiFiConnection() {                       //to establish connection to wifi
  WiFi.begin(ssid, password);
  delay(2000);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //WiFi.onEvent(WiFiEvent);
        delay(500);
        Serial.print(".");
        checkWifi = 0;
    }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  checkWifi = 0;
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(fbdo, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(fbdo, "medium");

  /*
    This option allows get and delete functions (PUT and DELETE HTTP requests) works for device connected behind the
    Firewall that allows only GET and POST requests.

    Firebase.enableClassicRequest(fbdo, true);
  */

  //String path = "/data";


  Serial.println("------------------------------------");
  Serial.println("Connected...");

  checkWifi = 1;
  BTcheck=0;
}


void serialEvent() {                                 //precaution for calierating via serial monitor                             //this interrupt will trigger when the data coming from the serial monitor(pc/mac/other) is received.
  received_from_computer = Serial.readBytesUntil(13, computerdata, 20);           //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
  computerdata[received_from_computer] = 0;                                       //stop the buffer from transmitting leftovers or garbage.
  serial_event = true;                                                            //set the serial event flag.
}

void readSensor1() {        //used for reading ph sensor 

  if (Firebase.getString(fbdo, "/Variable/Caliberate")) { //now is the start of the speed and time of motors
    Serial.print("calibrate = ");
      strinn = fbdo.stringData();
    Serial.println(fbdo.stringData());
    if(strinn=="0"){
      //parse_cmd(strinn);
      //Firebase.setString(fbdo,"/Variable/Calibrate/cal7","0");
      strinn="0";
    }else if(fbdo.stringData() == "CAL,7"){
      strinn = fbdo.stringData();
      parse_cmd(strinn);
      Firebase.setString(fbdo,"/Variable/Caliberate","0");
      strinn="0";
    }else if(fbdo.stringData() == "CAL,4"){
      strinn = fbdo.stringData();
      parse_cmd(strinn);
      Firebase.setString(fbdo,"/Variable/Caliberate","0");
      strinn="0";
    }else if(fbdo.stringData() == "CAL,10"){
      strinn = fbdo.stringData();
      parse_cmd(strinn);
      Firebase.setString(fbdo,"/Variable/Caliberate","0");
      strinn="0";
    }
  }

  Serial.println(pH.read_ph());

  if (Firebase.setDouble(fbdo, "/Variable/readings/reading1", pH.read_ph()))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }



  //  else
  //    Firebase.setFloat(fbdo, "/Variable/readings/reading1", pH.read_ph());

}

void readSensor2() {
  delay(500);// used for reading dht sensor and return the value of temperature
  h = dht.readHumidity();
  t = dht.readTemperature();
  // if (isnan(h) || isnan(t)) {
  // Serial.println("Failed to read from DHT sensor!");
  // return;
  // }


  float hic = dht.computeHeatIndex(t, h, false);
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(t);
  Serial.println();
  if (isnan(t)) {
    if (Firebase.setDouble(fbdo, "/Variable/readings/reading2", -400))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
      Serial.println("ETag: " + fbdo.ETag());
      Serial.print("VALUE: ");
      printResult(fbdo);
      Serial.println("------------------------------------");
      Serial.println();
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
      WiFi.begin();
      Serial.println("------------------------------------");
      Serial.println();
    }
  }
  else {
    if (Firebase.setDouble(fbdo, "/Variable/readings/reading2", t))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
      Serial.println("ETag: " + fbdo.ETag());
      Serial.print("VALUE: ");
      printResult(fbdo);
      Serial.println("------------------------------------");
      Serial.println();
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
      WiFi.begin();
      Serial.println("------------------------------------");
      Serial.println();
    }
  }

}

void readSensor3() {    //used for reading tds sensor

  if (serial_event == true) {                                                     //if a command was sent to the EZO device.
    for (i = 0; i <= received_from_computer; i++) {                               //set all char to lower case, this is just so this exact sample code can recognize the "sleep" command.
      computerdata[i] = tolower(computerdata[i]);                                 //"Sleep" ≠ "sleep"
    }
    i = 0;                                                                        //reset i, we will need it later
    if (computerdata[0] == 'c' || computerdata[0] == 'r')time_ = 570;             //if a command has been sent to calibrate or take a reading we wait 570ms so that the circuit has time to take the reading.
    else time_ = 250;                                                             //if any other command has been sent we wait only 250ms.


    Wire.beginTransmission(address);                                            //call the circuit by its ID number.
    Wire.write(computerdata);                                                   //transmit the command that was sent through the serial port.
    Wire.endTransmission();                                                     //end the I2C data transmission.


    if (strcmp(computerdata, "sleep") != 0) {                                   //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
      //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the EC circuit.

      delay(time_);                                                             //wait the correct amount of time for the circuit to complete its instruction.

      Wire.requestFrom(address, 32, 1);                                         //call the circuit and request 32 bytes (this could be too small, but it is the max i2c buffer size for an Arduino)
      code = Wire.read();                                                       //the first byte is the response code, we read this separately.

      switch (code) {                           //switch case based on what the response code is.
        case 1:                                 //decimal 1.
          Serial.println("Success");            //means the command was successful.
          break;                                //exits the switch case.

        case 2:                                 //decimal 2.
          Serial.println("Failed");             //means the command has failed.
          break;                                //exits the switch case.

        case 254:                               //decimal 254.
          Serial.println("Pending");            //means the command has not yet been finished calculating.
          break;                                //exits the switch case.

        case 255:                               //decimal 255.
          Serial.println("No Data");            //means there is no further data to send.
          break;                                //exits the switch case.
      }

      while (Wire.available()) {                 //are there bytes to receive.
        in_char = Wire.read();                   //receive a byte.
        ec_data[i] = in_char;                    //load this byte into our array.
        i += 1;                                  //incur the counter for the array element.
        if (in_char == 0) {                      //if we see that we have been sent a null command.
          i = 0;                                 //reset the counter i to 0.
          break;                                 //exit the while loop.
        }
      }

      Serial.println(ec_data);   //print the data.

      if (Firebase.setDouble(fbdo, "/Variable/readings/reading3", tds_float))
      {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
        Serial.println("ETag: " + fbdo.ETag());
        Serial.print("VALUE: ");
        printResult(fbdo);
        Serial.println("------------------------------------");
        Serial.println();
      }
      else
      {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
        WiFi.begin();
        Serial.println("------------------------------------");
        Serial.println();
      }

      Serial.println();                         //this just makes the output easier to read by adding an extra blank line
    }
    serial_event = false;                      //reset the serial event flag.

    if (computerdata[0] == 'r') string_pars(); //uncomment this function if you would like to break up the comma separated string into its individual parts.
  }
}

void readSensor4() {        //used for reading dht sensor and returning humidity
  delay(500);
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");

    if (Firebase.setDouble(fbdo, "/Variable/readings/reading4", -400))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
      Serial.println("ETag: " + fbdo.ETag());
      Serial.print("VALUE: ");
      printResult(fbdo);
      Serial.println("------------------------------------");
      Serial.println();
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
      WiFi.begin();
      Serial.println("------------------------------------");
      Serial.println();
    }
    return;
  }
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.println();

  if (Firebase.setDouble(fbdo, "/Variable/readings/reading4", h))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    WiFi.begin();
    Serial.println("------------------------------------");
    Serial.println();
  }
}

int duration = 0;
    
void readSensor5() {        //used for reading ultrasonic sensor
  delay(100);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  potValue = duration * 0.34 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(potValue);
  Serial.println(" mm");
  delay(500);

  if (Firebase.setDouble(fbdo, "/Variable/readings/reading5", potValue))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    WiFi.begin();
    Serial.println("------------------------------------");
    Serial.println();
  }
}


void printResult(FirebaseData &data)        //prints the result if data successfllu sent or not and if not then reason for not sending
{

  if (data.dataType() == "int")
    Serial.println(data.intData());
  else if (data.dataType() == "float")
    Serial.println(data.floatData(), 5);
  else if (data.dataType() == "double")
    printf("%.9lf\n", data.doubleData());
  else if (data.dataType() == "boolean")
    Serial.println(data.boolData() == 1 ? "true" : "false");
  else if (data.dataType() == "string")
    Serial.println(data.stringData());
  else if (data.dataType() == "json")
  {
    Serial.println();
    FirebaseJson &json = data.jsonObject();
    //Print all object data
    Serial.println("Pretty printed JSON data:");
    String jsonStr;
    json.toString(jsonStr, true);
    Serial.println(jsonStr);
    Serial.println();
    Serial.println("Iterate JSON data:");
    Serial.println();
    size_t len = json.iteratorBegin();
    String key, value = "";
    int type = 0;
    for (size_t i = 0; i < len; i++)
    {
      json.iteratorGet(i, type, key, value);
      Serial.print(i);
      Serial.print(", ");
      Serial.print("Type: ");
      Serial.print(type == FirebaseJson::JSON_OBJECT ? "object" : "array");
      if (type == FirebaseJson::JSON_OBJECT)
      {
        Serial.print(", Key: ");
        Serial.print(key);
      }
      Serial.print(", Value: ");
      Serial.println(value);
    }
    json.iteratorEnd();
  }
  else if (data.dataType() == "array")
  {
    Serial.println();
    //get array data from FirebaseData using FirebaseJsonArray object
    FirebaseJsonArray &arr = data.jsonArray();
    //Print all array values
    Serial.println("Pretty printed Array:");
    String arrStr;
    arr.toString(arrStr, true);
    Serial.println(arrStr);
    Serial.println();
    Serial.println("Iterate array values:");
    Serial.println();
    for (size_t i = 0; i < arr.size(); i++)
    {
      Serial.print(i);
      Serial.print(", Value: ");

      FirebaseJsonData &jsonData = data.jsonData();
      //Get the result data from FirebaseJsonArray object
      arr.get(jsonData, i);
      if (jsonData.typeNum == FirebaseJson::JSON_BOOL)
        Serial.println(jsonData.boolValue ? "true" : "false");
      else if (jsonData.typeNum == FirebaseJson::JSON_INT)
        Serial.println(jsonData.intValue);
      else if (jsonData.typeNum == FirebaseJson::JSON_FLOAT)
        Serial.println(jsonData.floatValue);
      else if (jsonData.typeNum == FirebaseJson::JSON_DOUBLE)
        printf("%.9lf\n", jsonData.doubleValue);
      else if (jsonData.typeNum == FirebaseJson::JSON_STRING ||
               jsonData.typeNum == FirebaseJson::JSON_NULL ||
               jsonData.typeNum == FirebaseJson::JSON_OBJECT ||
               jsonData.typeNum == FirebaseJson::JSON_ARRAY)
        Serial.println(jsonData.stringValue);
    }
  }
  else if (data.dataType() == "blob")
  {

    Serial.println();

    for (size_t i = 0; i < data.blobData().size(); i++)
    {
      if (i > 0 && i % 16 == 0)
        Serial.println();

      if (i < 16)
        Serial.print("0");

      Serial.print(data.blobData()[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  else if (data.dataType() == "file")
  {

    Serial.println();

    File file = data.fileStream();
    int i = 0;

    while (file.available())
    {
      if (i > 0 && i % 16 == 0)
        Serial.println();

      int v = file.read();

      if (v < 16)
        Serial.print("0");

      Serial.print(v, HEX);
      Serial.print(" ");
      i++;
    }
    Serial.println();
    file.close();
  }
  else
  {
    Serial.println(data.payload());
  }
}



String getValue(String data, char separator, int index)         //used for seperating string to different values which has been received in the string form from bluetooth
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length();

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void checkForMotor() {          //checks if there are any non -1 values in the available set of data in firebase for using motor
  if (Firebase.getInt(fbdo, "/Variable/motor1/speed1")) { //now is the start of the speed and time of motors
    Serial.print("data = ");
    if (fbdo.intData() == -1)
      speed1 = 0;
    else
      speed1 = fbdo.intData();
    Serial.println(fbdo.intData());
  }

  if (Firebase.getInt(fbdo, "/Variable/motor2/speed2")) { //now is the start of the speed and time of motors
    Serial.print("data = ");
    if (fbdo.intData() == -1)
      speed2 = 0;
    else
      speed2 = fbdo.intData();
    Serial.println(fbdo.intData());

  }

  if (Firebase.getInt(fbdo, "/Variable/motor3/speed3")) { //now is the start of the speed and time of motors

    Serial.print("data = ");
    if (fbdo.intData() == -1)
      speed3 = 0;
    else
      speed3 = fbdo.intData();
    Serial.println(fbdo.intData());

  }

  if (Firebase.getInt(fbdo, "/Variable/motor4/speed4")) { //now is the start of the speed and time of motors (timer-interrupt)
    Serial.print("data = ");
    if (fbdo.intData() == -1)
      speed4 = 0;
    else
      speed4 = fbdo.intData();
    Serial.println(fbdo.intData());

  }

  if (Firebase.getInt(fbdo, "/Variable/motor1/time1")) { //now is the start of the speed and time of motors
    Serial.print("data = ");
    if (fbdo.intData() == -1)
      mtime1 = 0;
    else
      mtime1 = fbdo.intData();
    Serial.println(fbdo.intData());

  }

  if (Firebase.getInt(fbdo, "/Variable/motor2/time2")) { //now is the start of the speed and time of motors
    Serial.print("data = ");
    if (fbdo.intData() == -1)
      mtime2 = 0;
    else
      mtime2 = fbdo.intData();
    Serial.println(fbdo.intData());

  }

  if (Firebase.getInt(fbdo, "/Variable/motor3/time3")) { //now is the start of the speed and time of motors
    Serial.print("data = ");
    if (fbdo.intData() == -1)
      mtime3 = 0;
    else
      mtime3 = fbdo.intData();
    Serial.println(fbdo.intData());

  }

  if (Firebase.getInt(fbdo, "/Variable/motor4/time4")) { //now is the start of the speed and time of motors

    Serial.print("data = ");
    if (fbdo.intData() == -1)
      mtime4 = 0;
    else
      mtime4 = fbdo.intData();
    Serial.println(fbdo.intData());
  }
}

void turnMOTOR(int digi, int &speedy, int &timmi) {   //now the digis are 13,12,14,27then 26,25,33,32

  Serial.print("motor pin: ");
  Serial.print(digi);
  Serial.print(" speed: ");
  Serial.print(speedy);
  Serial.print(" time: ");
  Serial.print(timmi);
  unsigned int i;
  digitalWrite(digi, HIGH);
  if (digi == 13)
    digli = Motor1_minus;
  else if (digi == 12)
    digli = Motor2_minus;
  else if (digi == 14)
    digli = Motor3_minus;
  else if (digi == 27)
    digli = Motor4_minus;
  for (i = 0; i <= timmi; i++)
  {
    digitalWrite(digli, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(speedy);                       // wait for a second
    digitalWrite(digli, LOW);    // turn the LED off by making the voltage LOW
    delay(speedy);                       // wait for a second
  }
  digitalWrite(digi,LOW);
  digitalWrite(digli,LOW);
  doThisAsWell();
  speedy = 0;
  timmi = 0;
}

void doThisAsWell() {     //turn the values stored in the firebase for motor back to -1


  if (Firebase.setDouble(fbdo, "/Variable/motor4/speed4", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }
  if (Firebase.setDouble(fbdo, "/Variable/motor4/time4", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }



  if (Firebase.setDouble(fbdo, "/Variable/motor3/speed3", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }

  if (Firebase.setDouble(fbdo, "/Variable/motor3/time3", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }


  if (Firebase.setDouble(fbdo, "/Variable/motor2/speed2", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    WiFi.reconnect();
    Serial.println("------------------------------------");
    Serial.println();
  }

  if (Firebase.setDouble(fbdo, "/Variable/motor2/time2", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    WiFi.reconnect();
    Serial.println("------------------------------------");
    Serial.println();
  }



  if (Firebase.setDouble(fbdo, "/Variable/motor1/time1", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    WiFi.reconnect();
    Serial.println("------------------------------------");
    Serial.println();
  }

  if (Firebase.setDouble(fbdo, "/Variable/motor1/speed1", -1))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("ETag: " + fbdo.ETag());
    Serial.print("VALUE: ");
    printResult(fbdo);
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    WiFi.reconnect();
    Serial.println("------------------------------------");
    Serial.println();
  }
}
void string_pars() {                  //this function will break up the CSV string into its 4 individual parts. EC|TDS|SAL|SG.
  //this is done using the C command “strtok”.

  ec = strtok(ec_data, ",");          //let's pars the string at each comma.
  tds = strtok(NULL, ",");            //let's pars the string at each comma.
  sal = strtok(NULL, ",");            //let's pars the string at each comma.
  sg = strtok(NULL, ",");             //let's pars the string at each comma.

  Serial.print("EC:");                //we now print each value we parsed separately.
  Serial.println(ec);                 //this is the EC value.

  Serial.print("TDS:");               //we now print each value we parsed separately.
  Serial.println(tds);                //this is the TDS value.

  Serial.print("SAL:");               //we now print each value we parsed separately.
  Serial.println(sal);                //this is the salinity value.

  Serial.print("SG:");               //we now print each value we parsed separately.
  Serial.println(sg);                //this is the specific gravity.
  Serial.println();                  //this just makes the output easier to read by adding an extra blank line

  //uncomment this section if you want to take the values and convert them into floating point number.

  ec_float = atof(ec);
  tds_float = atof(tds);
  sal_float = atof(sal);
  sg_float = atof(sg);

}

void setup() {
  SerialBT.begin("byDevdutt");
  BTcheck = 1;
  // put your setup code here, to run once:
  Wire.begin();                  //enable I2C port.
  Serial.println(F("Use commands \"CAL,7\", \"CAL,4\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
  if (pH.begin()) {
    Serial.println("Loaded EEPROM");
  }
  Serial.begin(115200);
  

  pinMode(Motor1_plus, OUTPUT);
  pinMode(Motor2_plus, OUTPUT);
  pinMode(Motor3_plus, OUTPUT);
  pinMode(Motor4_plus, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);
}

void loop() {
  
//  if (Firebase.getString(fbdo, "Variable/BTRef")) {
//    c = fbdo.stringData();
//  }
  //checkBT();

  //speedyman();
  if(BTcheck==0){
    checkForMotor();        //this checks if there any values available for motor
  
    if (speed1 != 0 && mtime1 != 0) {             //decide which motor has been asked to turn on 
      turnMOTOR(13, speed1, mtime1);
    } else if (speed2 != 0 && mtime2 != 0) {
      turnMOTOR(12, speed2, mtime2);
    } else if (speed3 != 0 && mtime3 != 0) {
      turnMOTOR(14, speed3, mtime3);
    } else if (speed4 != 0 && mtime4 != 0) {
      turnMOTOR(27, speed4, mtime4);
    } else {                                    //if no motors are turned on then read sensors
  
      if (Firebase.getInt(fbdo, "/Variable/sensors/data")) {        //checks if there is any change in the time assgined to each sensors to run
        time1 = fbdo.intData();
        //Serial.println(time1);
      }
  
      if (Firebase.getInt(fbdo, "/Variable/sensors/data2")) {
        time2 = fbdo.intData();
  
      }
  
      if (Firebase.getInt(fbdo, "/Variable/sensors/data3")) {
        time3 = fbdo.intData();
      }
  
      if (Firebase.getInt(fbdo, "/Variable/sensors/data4")) {
        time4 = fbdo.intData();
      }
  
      readSensor5();
      readSensor1();
      readSensor2();
      readSensor3();
      readSensor4();
  
      delay(5000);                         //a delay of 10000 is given for now
    }
  }else if(BTcheck==1){
    if(SerialBT.available()==0){
      String rec = "";
      rec = SerialBT.readString();
      Serial.println(rec);
      if(rec!=""){
        String vals1="",vals2="";
        vals1=getValue(rec,':',0);
        vals2=getValue(rec,':',1);
        vals1.toCharArray(ssid,vals1.length());
        Serial.println(ssid);
        vals2.toCharArray(password,vals2.length());
        Serial.println(password);

        delay(100);
        SerialBT.println("received");
        delay(500);
        rec="No";
      }
      if(rec == "No"){
        SerialBT.end();
        delay(1000);
        if(checkWifi == 0){
          WiFiConnection();
        }
      }
    }
  }
}

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("Disconnected from WiFi access point");
            BTcheck = 1;
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            Serial.println("Lost IP address and IP address is reset to 0");
            BTcheck = 1;
            break;
        default:
            BTcheck = 0;
    }}
