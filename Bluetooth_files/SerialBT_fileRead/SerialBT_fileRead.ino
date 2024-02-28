/* Header comment here
 */

#include "BluetoothSerial.h"
#include <SD.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
File file;
String str = "Hello World!";

void setup() {
  Serial.begin(115200);

  // File setup stuff
  if(!SD.begin()) {
    Serial.println("SD card initialization failed!");
    return;
  }
  file = SD.open("/letters.txt", FILE_WRITE);
  if (!file) {
    Serial.println("ERROR: cannot open file.");
    return;
  }
  fill_file(); // Add all letters
  file.close();
  // operator is_open = file.isOpen();
  Serial.println("File written successfully");

  // Bluetooth setup stuff
  SerialBT.begin("ESP32_test11"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
} // setup()

void loop() {
  if (SerialBT.available()) { // If there is data coming in (from Bluetooth)
    String command = SerialBT.readStringUntil('\n'); // Read string till new line char
    command.trim(); // remove leading and trailing whitespace

    if (command == "read") { // this is true, command reads fine
      file = SD.open("letters.txt", FILE_READ); // returning 0???
      if (file) {
        while (file.available()) {
          Serial.write(file.read());
        } // while()
        file.close();
      } else { // if file == false
        Serial.print("ERROR: cannot open file. Error code: ");
        //Serial.println(SD.error());
        SerialBT.print("ERROR: cannot open file.");
        //SerialBT.println(SD.error());
        Serial.println("File name: /letters.txt");
        SerialBT.println("File name: /letters.txt");
        Serial.println("File size: " + String(SD.open("/letters.txt").size()));
        SerialBT.println("File size: " + String(SD.open("/letters.txt").size()));
        Serial.println("File exists: " + String(SD.exists("/letters.txt")));
        SerialBT.println("File exists: " + String(SD.exists("/letters.txt")));
      }
    } else if (command == "delete") {
      str.remove(0); // remove all chars from the string
      Serial.println("Saved data deleted.");
      SerialBT.println("Saved data deleted."); // send confirmation to the Bluetooth device
    } else {
      Serial.println("ERROR: please enter 'read', or 'delete'.");
      SerialBT.println("ERROR: please enter 'read', or 'delete'."); // send error msg to bluetooth device
    }
  }
} // loop()

void fill_file() {
  for (char c = 'A'; c <= 'Z'; c++) {
    file.println(c);
  } // for()
  for (char c = 'a'; c <= 'z'; c++) {
    file.println(c);
  } // for()
} // fill_file()