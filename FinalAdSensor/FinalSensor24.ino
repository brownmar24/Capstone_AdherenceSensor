
//Go Baby Go Adherence Sensor Master Code
//Written by Ashlee B, Kyler M, Tara P 4/26/23

//Components:
//Accelerometer- reading implemented
//Low Power Mode
//RTC- EVI Pin implemented
//SD Card R/W
//Bluetooth(possible)

//Libraries
#include <SPI.h> // Communication with Serial Peripheral Interface (SPI) devices (ex. microcontroller and other circuits)
#include <Wire.h> // I2C communication between chips
#include <SparkFun_RV8803.h> // Read and set time on the RTC
#include "SparkFun_KX13X.h" // Configures and communicates data from the SparkFun KX13X accelerometer
#include <Adafruit_Sensor.h> // Required library for all Adafruit Unified Sensor libraries
#include <Adafruit_LIS3DH.h> // Configures and communicates data from Adafruit LIS3DH accelerometer (uses Unifed Sensor Library)
#include "FS.h" // ESP 32 File System library
#include "SD.h" // Reading and writing to SD cards using the SPI interface of a microcontroller
#include "SPI.h" // Part of the SPI library


//variables
RV8803 rtc; // Real Time Clock variable - uses the class RV8803
Adafruit_LIS3DH lis = Adafruit_LIS3DH(); // LIS3DH Accelerometer variable - UPDATE 10/30/2023: looks used
const float gravity = 9.80665;  // Earth's gravity in m/s^2

float accel;
int detect = 0;
float timeBlink = 0;
bool det = false;
float numReadings = 50.0;

// Hold the previous averaged readings for each axis on the accelerometer
float last_x;
float last_y;
float last_z;

// Hold the current averaged readings for each axis on the accelerometer
float x;
float y;
float z;

// Threshold of change in each axis to be considered a detected movement
float move_tholdX = 0.1;
float move_tholdY = 0.1;
float move_tholdZ = 0.1;

//Pins
#define EVI 14 // Timestamp pin
#define LED 17 // LED pin



void setup()
{
  pinMode(LED, OUTPUT); // Sets the LED pin for output information (turning on and off the LED)
  pinMode(EVI, OUTPUT); // NEED INFORMATION: Why is this an output pin?
  Wire.begin();//I2C addresses begin
  Serial.begin(115200);//set baud rate

  //RTC initialization
  if (rtc.begin() == false) // If the RTC cannot be found or started
  {
    Serial.println("Device not found. Please check wiring. Freezing."); // Prints to the serial monitor that the RTC cannot be found
    while (1); // NEED INFORMATION: why are they using an infinite loop?
  }
  Serial.println("RTC online!"); // Prints to the serial monitor that the RTC was found 

  rtc.setEVIEventCapture(RV8803_ENABLE); //Enables the Timestamping function

  Serial.println("LIS3DH test!");
  if (! lis.begin(0x18)) { // If the LIS3DH accelerometer cannot be found or used, follow this condition
    Serial.println("Couldnt start"); // Prints to the serial monitor that the accelerometer cannot be found
    while (1) yield(); // NEED INFORMATION: why are they using an infinite loop AGAIN? What is yield?
  }
  Serial.println("LIS3DH found!"); // The LIS3DH accelerometer is found

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
    Serial.println("Card Mount Failed"); // Prints to the serial monitor that the SD card cannot be found
    // Removed: digitalWrite(LED, HIGH); while (1) {}
    return; // Returns without further action
  }
  uint8_t cardType = SD.cardType(); // Saves the type of SD card to an unsigned 8-bit integer

  if (cardType == CARD_NONE) { // If there is no SD card, follow this condition
    Serial.println("No SD card attached"); // Prints that there is no SD card attached
    // Removed: digitalWrite(LED, HIGH); while (1) {}
    return; // Returns with no further action
  }

  Serial.print("SD Card Type: "); // Prints this statement to the serial monitor
  if (cardType == CARD_MMC) { // If the card is a MultiMediaCard, follow this condition
    Serial.println("MMC"); // Prints that the card is a MultiMediaCard (MMC) to the serial monitor
  }
  else if (cardType == CARD_SD) { // If the card is a standard SD card, follow this condition
    Serial.println("SDSC"); // Prints that the card is a standard SD card to the serial monitor
  }
  else if (cardType == CARD_SDHC) { // If the card is a high capacity SD card, follow this condition
    Serial.println("SDHC"); // Prints that the card is a high capacity SD card to the serial monitor
  }
  else { // If the card is of another type, follow this condition
    Serial.println("UNKNOWN"); // Prints that the card is of an unknown type of the serial monitor
  }
  
  //axis zeroing to eliminate false positives
  x = 0;
  y = 0;
  z = 0;
  
  // Procedure at device start-up
  String startTime = RTC(); //timestamp
  Serial.println();
  File file = SD.open("/accelerationdata.txt", FILE_APPEND); //open file.txt to write data
  if (!file) { // If the file cannot be opened, follow this condition
    Serial.println("Could not open file(writing)."); // Prints that the accelerometer.txt file cannot be opened
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
  // Accelerometer
  // Uses a delta scheme to detect movement
  accelRead();
  Serial.println(x);

  // If there is a significant change in all three axes, create a timestampe to record to SD card, and indicate the start of a new motion
  if ( abs(last_x - x) > move_tholdX &&
       abs(last_y - y) > move_tholdY &&
       abs(last_z - z) > move_tholdZ && det == false) {
    digitalWrite(EVI, HIGH);   // trigger EVI pin
    delay(20);                       // wait for a second
    digitalWrite(EVI, LOW);    // turn off EVI pin output
    Serial.print("Detected "); //Debug purposes
    det = true; // Indicates a new motion was detected
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

  // new sensor event
  sensors_event_t event;

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
  File file = SD.open("/accelerationdata.txt", FILE_APPEND); //open file.txt to write data
  if (!file) { // If the accelerationdata.txt file couldn't be read, follow this condition
    Serial.println("Could not open file(writing)."); // Prints to the serial monitor that the file couldn't be opened
  }
  else { // The accelerationdata.txt file could be read: 
    file.print("Start Time of Motion Detected At: "); // Appends this to the bottom of the file when a new motion is detected
    file.print(startTime); // Prints the initial time stamp to the file
    file.println(); // Adds a new line to the text file
    file.close(); // Closes accelerationdata.txt file
  }

}
void sd_Stop() {//printing to the SD card

  String stopTime = RTC(); //timestamp
  Serial.println(); // Adds a new line
  File file = SD.open("/accelerationdata.txt", FILE_APPEND); //open file.txt to write data
  if (!file) { // If the accelerationdata.txt file couldn't be read, follow this condition
    Serial.println("Could not open file(writing)."); // Prints to the serial monitor that the file couldn't be opened
  }

  else { // The acclerationdata.txt file could be read
    file.print("Time of Stop Detected At: "); // Appends this to the bottom of the file when motion is stopped
    file.print(stopTime); // Prints the stopping time stamp to the file
    file.println(); // Adds a new line to the text file
    file.close(); // Closes the accelerationdata.txt file
  }
}
