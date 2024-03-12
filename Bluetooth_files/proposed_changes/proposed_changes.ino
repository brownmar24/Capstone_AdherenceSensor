/*
Go Baby Go Adherence Sensor Master Code
Written by Ashlee B, Kyler M, Tara P 4/26/2023
Adapted by Margo B, Kaylee M, Anna Y 4/26/2024
*/

// Bluetooth Libraries
#include <BTAddress.h> // Part of BluetoothSerial
#include <BTAdvertisedDevice.h> // Part of BluetoothSerial
#include <BTScan.h> // Part of BluetoothSerial
#include <BluetoothSerial.h> // Part of BluetoothSerial

//Libraries
#include <SPI.h> // Communication with Serial Peripheral Interface (SPI) devices (ex. microcontroller and other circuits)
#include <Wire.h> // I2C communication between chips
#include <SparkFun_RV8803.h> // Read and set time on the RTC
#include <Adafruit_Sensor.h> // Required library for all Adafruit Unified Sensor libraries
#include <Adafruit_LIS3DH.h> // Configures and communicates data from Adafruit LIS3DH accelerometer (uses Unifed Sensor Library)
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Used to monitor external battery level
#include <AES.h> // 128-bit CBC for encryption
#include <AESLib.h> // AES implementation (128-bit CBC) for low-memory conditions
//#include "BluetoothSerial.h" // Device can send and receive messages to Bluetooth terminal
#include <CTR.h> // Part of the crypto libraries
#include <Crypto.h> // Rhys Weatherly's crypto library
#include "arduino_base64.hpp" // Turns AES encryption into base64
#include "FS.h" // ESP 32 File System library
#include <SD.h> // Potential Problem: the duplicate libraries
#include "SPI.h" // Part of the SPI library

//Pins
#define EVI 14 // Timestamp pin
#define R_LED 4 // Red LED pin
#define G_LED 13 // Green LED pin - changed from 16 to 13 
#define B_LED 17 // Blue LED pin
#define BUTTON 25 // Button input pin

// Bluetooth enabling definition
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run 'make menuconfig' to and enable it
#endif

// Bluetooth serial enabling definition
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// Electronic component variables
RV8803 rtc; // Real Time Clock variable - uses the class RV8803
Adafruit_LIS3DH lis = Adafruit_LIS3DH(); // LIS3DH Accelerometer variable - UPDATE 10/30/2023: looks used
SFE_MAX1704X battery; // External battery object, used to 
double battery_percent; // Holds the current battery percentage
int low_battery_threshold = 20; // Threshold for low battery indication

// Bluetooth Variable
BluetoothSerial SerialBT;

float accel;
int detect = 0; // Number of no-motion detections
bool det = false; // Indicates a new motion was detected
float numReadings = 50.0; // Used to average the accelerometer readings
const float gravity = 9.80665;  // Earth's gravity in m/s^2

// Hold the previous averaged readings for each axis on the accelerometer
float last_x;
float last_y;
float last_z;

// Hold the current averaged readings for each axis on the accelerometer
float x;
float y;
float z;

// Read state of the button press
int buttonState = 0;

// Idle Mode: variables that measure the time since the last detected motion
unsigned long stop_motion; // Time when motion stopped
unsigned long current_time; // Current time
lis3dh_dataRate_t current_rate = lis.getDataRate(); // Gets the current accelerometer data collection mode (for low power mode)

// AES object used for encryption
AESLib aesLib;

// The accelerationdata.txt file
File file; 

void setup() {
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(EVI, OUTPUT); // Connecting to the Real-Time Clock
  Wire.begin();//I2C addresses begin
  Serial.begin(115200);//set baud rate

  //RTC initialization
  if (rtc.begin() == false) // If the RTC cannot be found or started
  {
    Serial.println(F("Device not found. Please check wiring. Freezing.")); // Prints to the serial monitor that the RTC cannot be found
    while (1); // NEED INFORMATION: why are they using an infinite loop?
  }
  Serial.println(F("RTC online!")); // Prints to the serial monitor that the RTC was found 

  if (rtc.setToCompilerTime() == false) {
    Serial.println(F("An error has occurred in setting the RTC to compiler time"));
  }

  rtc.setEVIEventCapture(RV8803_ENABLE); //Enables the Timestamping function

  Serial.println(F("LIS3DH test!"));
  if (! lis.begin(0x18)) { // If the LIS3DH accelerometer cannot be found or used, follow this condition
    Serial.println(F("Couldnt start")); // Prints to the serial monitor that the accelerometer cannot be found
    while (1) yield(); // NEED INFORMATION: why are they using an infinite loop AGAIN? What is yield?
  }
  Serial.println(F("LIS3DH found!")); // The LIS3DH accelerometer is found

  // Set up lis and init readings
  lis.setRange(LIS3DH_RANGE_4_G); // NEED INFORMATION: What does this mean?

  delay(10); // Delays by 10 milliseconds
  lis.read(); // Takes a reading from the accelerometer
  
  //orientated on the back of the permobil
  x = -lis.z; // Takes the x-axis reading, using a negated z-axis accelerometer reading
  y = lis.y; // Takes the y-axis reading
  z = -lis.x; // Takes the z-axis reading, using a negated x-axis accelerometer reading

  // Check if SD is connected correctly
  if (!SD.begin()) { // If the SD card cannot be found, follow this condition
    Serial.println(F("Card Mount Failed")); // Prints to the serial monitor that the SD card cannot be found
    // Removed: digitalWrite(LED, HIGH); while (1) {}
    return; // Returns without further action
  }
  uint8_t cardType = SD.cardType(); // Saves the type of SD card to an unsigned 8-bit integer

  if (cardType == CARD_NONE) { // If there is no SD card, follow this condition
    Serial.println(F("No SD card attached")); // Prints that there is no SD card attached
    // Removed: digitalWrite(LED, HIGH); while (1) {}
    return; // Returns with no further action
  }

  Serial.print("SD Card Type: "); // Prints this statement to the serial monitor
  if (cardType == CARD_MMC) { // If the card is a MultiMediaCard, follow this condition
    Serial.println(F("MMC")); // Prints that the card is a MultiMediaCard (MMC) to the serial monitor
  }
  else if (cardType == CARD_SD) { // If the card is a standard SD card, follow this condition
    Serial.println(F("SDSC")); // Prints that the card is a standard SD card to the serial monitor
  }
  else if (cardType == CARD_SDHC) { // If the card is a high capacity SD card, follow this condition
    Serial.println(F("SDHC")); // Prints that the card is a high capacity SD card to the serial monitor
  }
  else { // If the card is of another type, follow this condition
    Serial.println(F("UNKNOWN")); // Prints that the card is of an unknown type of the serial monitor
  }

  // Initialize the MAX17043
  if (battery.begin() == false) {
    Serial.println(F("Can't find MAX1704X Battery"));
  }

  // For low battery mode
  battery.quickStart(); // Restarts the MAX1704X to createa. more accurate guess for the SOC
  battery.setThreshold(low_battery_threshold); // An interrupte to alert when the battery reahes 20% and below

  //axis zeroing to eliminate false positives
  x = 0;
  y = 0;
  z = 0;
  
  // Procedure at device start-up
  String startTime = RTC(); //timestamp
  Serial.println();
  file = SD.open("/accelerationdata.txt", FILE_APPEND); //open file.txt to write data
  if (!file) { // If the file cannot be opened, follow this condition
    Serial.println(F("Could not open file(writing).")); // Prints that the accelerometer.txt file cannot be opened
  }

  else { // If the file can be opened, follow this condition
    file.println(); // Adds a new line to the file
    file.print("Device start up at: "); // Writes this to the bottom of the file
    file.print(startTime); // Prints the start-up time to the bottom of the file
    file.println(); // Prints another new line to the bottom of the file
    file.close(); // Closes the file
  }
}

void loop() {
  // Records whether the button has been pressed
  buttonState = digitalRead(BUTTON);
  current_time = millis();

  // Gets the battery percent
  battery_percent = battery.getSOC();

  /*
   * Condition 1: Bluetooth File Transfer
   * The user presses the button to send the file from the device to a supported computer or mobile device
  */

  // Check for button press 
  if (buttonState == 1) {
    // Bluetooth Transfer Set-Up
    SerialBT.begin("Adherence_Sensor"); // Sets the name of the device
    Serial.println(F("The device can connect to Bluetooth"));
    setColor(0, 0, 255); // Set LED color to blue
    SerialBT.println("Enter one of the following commands:");
    SerialBT.println("'read' - prints out all data in the accelerometerdata.txt file");
    SerialBT.println("'delete' - resets the accelerometerdata.txt file");
    SerialBT.println("'exit' - exits Bluetooth mode (please turn off or restart the device after exiting)");

    while(1) {
      if (SerialBT.available()) { // If there is data coming in (from Bluetooth)
        String command = SerialBT.readStringUntil('\n'); // Read string until new line
        command.trim(); // Remove leading and trailing whitespace

        if (command == "read") { // Protocol for the read command
          file = SD.open("/accelerationdata.txt", FILE_READ); // Opens the file to print to the Bluetooth terminal
          if (file) { // Prints all data from the file to the serial monitor
            while(file.available()) { // Prints the data from the file line-by-line
              SerialBT.write(file.read()); // Writes the line to the Bluetooth terminal
            }
            file.close(); // Closes the accelerometerdata.txt file
          }
          else { // If the file cannot be opened
            Serial.println(F("ERROR: Cannot open file"));
            SerialBT.println("ERROR: Cannot open /accelerationdata.txt; File Size: " + String(SD.open("/accelerationdata.txt").size()));
          }
        }
        else if (command == "delete") { // Protocol for the delete command
          SD.remove("/accelerationdata.txt"); // Deletes the accelerationdata.txt file from the SD card
          if (SD.exists("/accelerationdata.txt")) { // Checks that the file stil exists
            Serial.println(F("Delete functionality did not work"));
          }
          else { // If the file successfully deletes
            file = SD.open("/accelerationdata.txt", FILE_APPEND); // Resets the accelerationdata.txt 
            file.close(); // Closes the accelerationdata.txt file
            SerialBT.println("Please turn off or restart the Adherence Sensor"); // Prompts the user to reset the Adherence Sensor
          }
        }
        else if (command == "exit") { // Protocol for the exit command
          SerialBT.println("Please turn off or restart the Adherence Sensor"); // Prompts the user to reset the device to normal collection
          break; // Exits to the main loop() function
        }
        else { // Asks the user to input a valid command
          SerialBT.println("Invalid command: " + command);
        }
      }
    }  
    // If the user (incorrectly) leaves the device on after exiting, the following will execute before exiting this condition
    delay(5000); // Five-second delay
    buttonState = 0; // Returns the button variable to the "not-pushed" state
    setColor(0, 255, 0); // Sets the LED color back to green
  }
  
  /* CHECK FOR IDLE MODE HERE */
  else if (current_time - stop_motion > 30000) {
    setColor(255, 30, 0); // Sets the color to "orange-ish"
    lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); // Sets the accelerometer to low-power mode
    motionDetection(); // Resumes motion detection
  }

  // Checks if battery is below 20 percent 
  else if (battery_percent < 20) {
    setColor(255, 0, 0); // Sets the LED color to red
    motionDetection(); // Runs the motion detection
    // Print the battery percentage
    Serial.print("Battery Percentage: ");
    Serial.print(battery_percent);
    Serial.println("%");
  }

  else {
    setColor(0, 255, 0); // Sets the LED color to green
    lis.setDataRate(LIS3DH_DATARATE_10_HZ); // Sets the accelerometer to normal data collection
    motionDetection(); // Runs the motion detection
  }  
}

// Accelerometer: Uses a delta scheme to detect movement
void motionDetection() {
  accelRead(); // Takes 50 accelerometer readings and averages them
  Serial.println(x); // Prints the x value, which is influenced by vertical gravity

  // Threshold of change in each axis to be considered a detected movement
  float move_tholdX = 0.1;
  float move_tholdY = 0.1;
  float move_tholdZ = 0.1;

  // If there is a significant change in all three axes, create a timestampe to record to SD card, and indicate the start of a new motion
  if ( abs(last_x - x) > move_tholdX &&
       abs(last_y - y) > move_tholdY &&
       abs(last_z - z) > move_tholdZ && det == false) {
    digitalWrite(EVI, HIGH);   // trigger EVI pin
    delay(20);                 // wait for a second
    digitalWrite(EVI, LOW);    // turn off EVI pin output
    Serial.print("Detected "); //Debug purposes
    det = true; // Indicates a new motion was detected
    stop_motion = millis();
    sd_Start(); // Writes to the file when the motion was detected
  } 
  
  // If there is no longer significant change in all three axes add to the detection counter
  else if (move_tholdX > abs(last_x - x) &&
             move_tholdY > abs(last_y - y) &&
             move_tholdY > abs(last_z - z) && detect < 3 && det == true) {
    detect += 1;
  }

    // If after 300ms there is no more significant change, the time of stopped motion will be printed to the SD card
  else if (move_tholdX > abs(last_x - x) &&
             move_tholdY > abs(last_y - y) &&
             move_tholdY > abs(last_z - z) && detect == 3 && det == true) {
    digitalWrite(EVI, HIGH);   // trigger EVI pin
    delay(20);                       // wait for a second
    digitalWrite(EVI, LOW);    // turn off EVI pin output
    Serial.print("Stopped "); //Debug purposes
    sd_Stop(); // Prints the time the motion stopped
    det = false; // Resets the det boolean to false
    detect = 0; // Resets the detect counter
    stop_motion = millis(); // Detects the time motion stops
  } 
  
  else { //if the data goes above the thresholds over reset detect counter
    detect = 0; // Resets the detect counter
  }

  // Give time between readings
  delay(100); // 100 milliseconds between readings
}

//timestamp subroutine
String RTC()
{
  String time; // The string used to save the timestamp data
  //RTC timestamp
  if (rtc.getInterruptFlag(FLAG_EVI)) {
    rtc.updateTime(); // Grabs an updated gime from the compiler
    rtc.clearInterruptFlag(FLAG_EVI);

    // Get the current date in mm/dd/yyyy format
    String currentTime = rtc.stringDateUSA();

    //Get the timestamp
    String timestamp = rtc.stringTimestamp();

    // Saves the date string and the time string
    time = currentTime + " " + timestamp;

    Serial.print(time); // Prints the date and time to the serial monitor
    //end debug
  }
  return time; // Returns the timestamp string
}

//read from the accelerometer function puts data through moving average filter
void accelRead() {
  sensors_event_t event; // new sensor event

  // Sums the 50 samples for each axis
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;

  // Moving Average Filter- Take average of 50 samples for each axis
  for (int i = 0; i < numReadings; i++) {
    lis.getEvent(&event); // Grabs the specific accelerometer data

    x_sum += (-event.acceleration.z); // Adds this specific x-axis instance to the sum, using the negated z-axis accelerometer data
    y_sum += event.acceleration.y; // Adds this specific y-axis instance to the sum
    z_sum += (-event.acceleration.x); // Adds this specific z-axis instance to the sum, using the negated x-axis accelerometer data
    delay(2); // Delays for 2 milliseconds
  }

  // Assigns the previous readings to the 'last' versions of the specific axes
  last_x = x;
  last_y = y;
  last_z = z;

  // Divides the readings by 50 (because of the 50 readings), and assigns these values to their respective axis readings
  x = x_sum / numReadings;
  y = y_sum / numReadings;
  z = z_sum / numReadings;
}

void sd_Start() {//prints to sd card starting

  String startTime = RTC(); // Grabs the initial timestamp using the RTC helper function
  Serial.println(); // Adds a new line in the serial monitor

  String start_encrypt = "Start Time of Motion Detected At: " + startTime + "\n"; // String for the start time
  encryptAndWrite(start_encrypt);
}

void sd_Stop() {//printing to the SD card

  String stopTime = RTC(); //timestamp
  Serial.println(); // Adds a new line

  // The acclerationdata.txt file could be read
  String stop_encrypt = "Time of Stop Detected At: " + stopTime + "\n";
  encryptAndWrite(stop_encrypt);
  //file.print(stop_encrypt); // Appends this to the bottom of the file when motion is stopped
}

void encryptAndWrite(String data) {
  file = SD.open("/accelerationdata.txt", FILE_APPEND); //open file.txt to write data
  if (!file) { // If the accelerationdata.txt file couldn't be read, follow this condition
    Serial.println("Could not open file(writing)."); // Prints to the serial monitor that the file couldn't be opened
  }

  else { 
    data = encrypt(data); // Encrypts the data
    String data2 = decrypt(data); // Decrypts the data

    file.println(data); // Prints encrypted data to the file
    Serial.println(data); // Prints the encurpted file to the serial terminal
    Serial.println(data2); // Prints the decrypted data to the serial terminal
    // Serial.println(data2);
    file.close(); // Closes accelerationdata.txt file
  }
}

// The encryption function
String encrypt(String inputText) {

    // calculate the length of bytes of the input text
    // an extra of byte must be added for a null character
    // a null character will be filled as a text terminator
    // so that the process will not overflow to other parts of memory    
    int bytesInputLength = inputText.length() + 1;

    // declare an empty byte array (a memory storage)
    byte bytesInput[bytesInputLength];

    // convert the text into bytes, a null char is filled at the end
    inputText.getBytes(bytesInput, bytesInputLength);

    // calculate the length of bytes after encryption done
    int outputLength = aesLib.get_cipher_length(bytesInputLength);

    // declare an empty byte array (a memory storage)
    byte bytesEncrypted[outputLength];

    // initializing AES engine

    // Cipher Mode and Key Size are preset in AESLib
    // Cipher Mode = CBC
    // Key Size = 128

    // declare the KEY and IV
    byte aesKey[] = { 23, 45, 56, 67, 67, 87, 98, 12, 32, 34, 45, 56, 67, 87, 65, 5 };
    byte aesIv[] = { 123, 43, 46, 89, 29, 187, 58, 213, 78, 50, 19, 106, 205, 1, 5, 7 };

    // set the padding mode to paddingMode.CMS
    aesLib.set_paddingmode((paddingMode)0);

    // encrypt the bytes in "bytesInput" and store the output at "bytesEncrypted"
    // param 1 = the source bytes to be encrypted
    // param 2 = the length of source bytes
    // param 3 = the destination of encrypted bytes that will be saved
    // param 4 = KEY
    // param 5 = the length of KEY bytes (16)
    // param 6 = IV
    aesLib.encrypt(bytesInput, bytesInputLength, bytesEncrypted, aesKey, 16, aesIv);

    // declare a empty char array
    char base64EncodedOutput[base64::encodeLength(outputLength)];

    // convert the encrypted bytes into base64 string "base64EncodedOutput"
    base64::encode(bytesEncrypted, outputLength, base64EncodedOutput);

    // convert the encoded base64 char array into string
    return String(base64EncodedOutput);
}

// The decryption function
String decrypt(String encryptedBase64Text) {

    // calculate the original length before it was coded into base64 string
    int originalBytesLength = base64::decodeLength(encryptedBase64Text.c_str());

    // declare empty byte array (a memory storage)
    byte encryptedBytes[originalBytesLength];
    byte decryptedBytes[originalBytesLength];

    // convert the base64 string into original bytes
    // which is the encryptedBytes
    base64::decode(encryptedBase64Text.c_str(), encryptedBytes);

    // initializing AES engine

    // Cipher Mode and Key Size are preset in AESLib
    // Cipher Mode = CBC
    // Key Size = 128

    // declare the KEY and IV
    byte aesKey[] = { 23, 45, 56, 67, 67, 87, 98, 12, 32, 34, 45, 56, 67, 87, 65, 5 };
    byte aesIv[] = { 123, 43, 46, 89, 29, 187, 58, 213, 78, 50, 19, 106, 205, 1, 5, 7 };

    // set the padding mode to paddingMode.CMS
    aesLib.set_paddingmode((paddingMode)0);

    // decrypt bytes in "encryptedBytes" and save the output in "decryptedBytes"
    // param 1 = the source bytes to be decrypted
    // param 2 = the length of source bytes
    // param 3 = the destination of decrypted bytes that will be saved
    // param 4 = KEY
    // param 5 = the length of KEY bytes (16)
    // param 6 = IV
    aesLib.decrypt(encryptedBytes, originalBytesLength, 
                   decryptedBytes, aesKey, 16, aesIv);

    // convert the decrypted bytes into original string
    String decryptedText = String((char*)decryptedBytes);

    return decryptedText;
}

// Sets the color of the LED
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(R_LED, redValue);
  analogWrite(G_LED, greenValue);
  analogWrite(B_LED, blueValue);
}
