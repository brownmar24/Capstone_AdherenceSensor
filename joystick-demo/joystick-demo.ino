// Adapted from: https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/examples/LITTLEFS_test/LITTLEFS_test.ino
// Project details: https://RandomNerdTutorials.com/esp32-write-data-littlefs-arduino/

#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>

//  You only need to format LittleFS the first time you run a
//  test or else use the LITTLEFS plugin to create a partition 
//  https://github.com/lorol/arduino-esp32littlefs-plugin

#define FORMAT_LITTLEFS_IF_FAILED true

#define VRX_PIN 36
#define VRY_PIN 39

int x_value = 0;
int y_value = 0;

int key = 5;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}

void setup(){
    Serial.begin(115200);

    if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
        Serial.println("LittleFS Mount Failed");
        return;
    }

    bool fileexists = LittleFS.exists("/joystick.txt");
   Serial.print(fileexists);
   if(!fileexists) {
       Serial.println("File doesn’t exist");
       Serial.println("Creating file...");
       // Create File and add header
       writeFile(LittleFS, "/joystick.txt", "JOYSTICK DATA \r\n");
   }
   else {
       Serial.println("File already exists");
   }
}

String encrypt(String orig_data) {
  char temp;
  String encrypt_data = orig_data;

  for (int i = 0; encrypt_data[i] != '\0'; i++){
    temp = encrypt_data[i];
    if(temp >= 'a' && temp <= 'z'){
      temp = temp + key;
      
      if(temp > 'z'){
        temp = temp - 'z' + 'a' - 1;
      }
      
      encrypt_data[i] = temp;
    }else if(temp >= 'A' && temp <= 'Z'){
      temp = temp + key;
      
      if(temp > 'Z'){
        temp = temp - 'Z' + 'A' - 1;
      }
      
      encrypt_data[i] = temp;
    }
  }//for()

}//encrypt()

void loop(){
  x_value = analogRead(VRX_PIN);
  y_value = analogRead(VRY_PIN);
  // 1. write real string to a variable
  String x_y_data = String(x_value) + ", " + String(y_value) + "\r\n"; 
  // 2. Encrypt data set
  String x_y_encrypted = encrypt(x_y_data);
  // 3. Serial print real & encrypted data string
  Serial.println("Real value: " + x_y_data);
  Serial.println("Encrypted value: " + x_y_encrypted + "\n");
  // 4. Write encrypted string to the file
  appendFile(LittleFS, "/joystick.txt", (String(x_value) + ", " + String(y_value) + "\r\n").c_str());
  Serial.println(String(x_value) + ", " + String(y_value) +  "\n");
  delay(1000);
}
