
#include <Arduino.h>
#include <EEPROM.h>
#include <ESPUI.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <SPI.h>
#include <ESPmDNS.h>
#include <Adafruit_ADS1X15.h>
//#include "ADS1X15.h"
//#include <WebServer.h>
#include <SD.h>
#include "FS.h"
#include "SPI.h"
#include <time.h>
#include <WiFi.h>
#include <ESP32Time.h>
#include "HX711.h"


//Config function module before calling for using in VS code IDE
void generalCallback(Control *sender, int type);
void startButtonCallback(Control *sender, int type);
void loadResultCallback(Control *sender, int type);
void downloadCallback(Control *sender, int type);
void moveAxisXY(Control *sender, int type);
void posButtonCallback(Control *sender, int type);
void setTextInputCallback(Control *sender, int type);
void configButtonCallback(Control *sender, int type);
void enterWifiDetailsCallback(Control * sender, int type);
void connectWifi();
void readStringFromEEPROM(String & buf, int baseaddress, int size);

// Define some steppers and the pins the will use
// AccelStepper stepper1; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperX(AccelStepper::FULL2WIRE, 4, 2);   //Axis X
AccelStepper stepperY(AccelStepper::FULL2WIRE, 15, 13);   //Axis Y
AccelStepper stepperZ(AccelStepper::FULL2WIRE, 14, 27);     //Axis Z

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 26;
const int LOADCELL_SCK_PIN = 25;


//Settings
#define SLOW_BOOT 0
#define HOSTNAME "KU.Physics.LAB"
#define FORCE_USE_HOTSPOT 0
#define trigWDTPin 32
#define ledHeartPIN 0


//UI handles
uint16_t wifi_ssid_text, wifi_pass_text;
uint16_t resultLabel, mainLabel, grouplabel, grouplabel2, mainSwitcher, mainSlider, mainText;
uint16_t settingZNumber, resultButton, mainTime, downloadButton, selectDownload, SDcardRead, ADSStatus, patternStatus, logStatus;
uint16_t maxDept; 
uint16_t styleButton, styleLabel, styleSwitcher, styleSlider, styleButton2, styleLabel2, styleSlider2;
uint16_t nameText, loopText, posText, moveText, saveConfigButton, depthText, intervalPress, offSet1, offSet2, offSet3, offSet4;
uint16_t graph;
volatile bool updates = false;

//UI Variables for status
String cardStatus = "Initialize SD card";
String ADS = "Initialized ADS1115";
String dataLog = "File is ready";

String fileNameResult = "";


// REPLACE WITH YOUR CALIBRATION FACTOR
#define CALIBRATION_FACTOR -88.22111 //-89.32745  // 184.703557312253
int offset = -6; //940  1760  80 1040 1240
#define zero_factor 58744

//loadcell
#define DEC_POINT  3

/*int16_t raw10;
int16_t raw20;
int16_t raw30;
int16_t raw40;
float volt10;
float volt20;
float volt30;
float volt40;*/

// กำหนดค่าการคาลิเบรตสำหรับแต่ละตัว (Offset และ Gain)
float offset1 = 0.00; // Offset สำหรับตัวแรก
//float gain1 = 0.99;    // Gain สำหรับตัวแรก

float offset2 = 0.00; // Offset สำหรับตัวที่สอง
//float gain2 = 0.99;    // Gain สำหรับตัวที่สอง

float offset3 = 0.00; // Offset สำหรับตัวที่สาม
//float gain3 = 0.99;    // Gain สำหรับตัวที่สาม

float offset4 = 0.00; // Offset สำหรับตัวที่สี่
//float gain4 = 0.99;    // Gain สำหรับตัวที่สี่

//Constant Value for calculation
float R1 = 100000; // constant R1 = 100 kOhm for 1x1 // constant R1 = 10 kOhm for 3x3
float R2 = 100000; // constant R2 = 100 kOhm for 1x1 // constant R1 = 10 kOhm for 3x3

float I;
float amp10;
float amp20;
float amp30;
float amp40;
float Res10;
float Res20;
float Res30;
float Res40;
float loadCellReading;
float readloadcell;
float g = 9.81;

unsigned long millisAtLastSync = 0;  // ค่า millis() ตอน sync เวลา
time_t lastEpoch = 0;               // Epoch เวลาในวินาทีที่ sync ล่าสุด
String yearStr = "";
String monthStr = "";
String dayStr = "";
String hourStr = "";
String minStr = "";
String secStr = "";
String msStr = "";
String record = "";

//WebServer server(80);
//String htmlView = "";
// WiFi credentials
const char *ssid = "LINKSYS2";  //{TP-Link_AD28, TP-Link_5080, TP-Link_0B37, LINKSYS2}
const char *password = "cblast401"; //{:96969755,  :70254211, :11741428, :cblast401}

String dateTimeStr = "";
long timezone = 7;
byte daysavetime = 0;
int testCount = 1;
int loopCount = 1;

unsigned long diffMillis = 0;
unsigned long diff = 0;
const int limit = 1000;
int axisX = 1;
int axisY = 1;
int axisZ = 1;
int depth = 40;
int interval = 30;

int x = 0;
int y = 0;

int operatingPercen = 0;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
boolean isStopStart = false;
String fileName = "/KUPhysicsLabTester_";

int previousX = 0;
int previousY = 0;
int previousZ = 0;

// Define a struct to hold configuration parameters
struct Config01 {
    String name;
    int loop;
    int numPos;
    int **pos; // Dynamically allocated array
};

struct Config02 {
    String name;
    int loop;
    int numPos;
    int **pos; // Dynamically allocated array
};

struct Config03 {
    String name;
    int loop;
    int numPos;
    int **pos; // Dynamically allocated array
};

Config01 config01;
Config02 config02;
Config03 config03;

File myFile;
ESP32Time rtc;

HX711 scale;

//ADS1115 ads[4];// Array to store ADS1115 objects
/*ADS1115 ads01(0x48);
ADS1115 ads02(0x49);
ADS1115 ads03(0x4A);
ADS1115 ads04(0x4B);
*/

// สร้างตัวแปร ADS1115 สำหรับแต่ละตัว
Adafruit_ADS1115 ads1; // Address 0x48
Adafruit_ADS1115 ads2; // Address 0x49
Adafruit_ADS1115 ads3; // Address 0x4A
Adafruit_ADS1115 ads4; // Address 0x4B

struct tm tmstruct;

void heartBeat()
{
  //   Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);

  // Led monitor for Heartbeat
  digitalWrite(ledHeartPIN, LOW);
  delay(100);
  digitalWrite(ledHeartPIN, HIGH);

  // Return to high-Z
  pinMode(trigWDTPin, INPUT);

  // Serial.println("Heartbeat");
  // SerialBT.println("Heartbeat");
}

/**
 * Splits a string into substrings and stores the integer values in a 2D integer array.
 *
 * @param str The string to be split.
 * @param delimiter The character used to split the string.
 * @param arr A 2D integer array to store the integer values.
 * @param row The row index in the 2D array where the values will be stored.
 *
 * @throws None
 */
void split(const String &str, char delimiter, int **arr, int row) {
    // Initialize start and end indices
    int startIndex = 0;
    int endIndex = str.indexOf(delimiter);
    
    // Retrieve the integer value from the substring and store it in the array
    arr[row][0] = str.substring(startIndex, endIndex).toInt();
    
    // Update start index to exclude the delimiter
    startIndex = endIndex + 1;
    
    // Find the index of the next delimiter
    endIndex = str.indexOf(delimiter, startIndex);
    
    // Retrieve the integer value from the substring and store it in the array
    arr[row][1] = str.substring(startIndex, endIndex).toInt();
    
    // Update start index to exclude the delimiter
    startIndex = endIndex + 1;
    
    // Retrieve the remaining substring and convert it to an integer
    arr[row][2] = str.substring(startIndex).toInt();
}


// Function to read configuration data from file and populate the Config struct
bool readConfig(const String &filename, Config01 &config01, Config02 &config02, Config03 &config03) {
    // Initialize SD card
    if (!SD.begin()) {
        // Serial.println("SD card initialization failed.");
        return false;
    }

    // Open the config file
    File configFile = SD.open(filename);
    if (!configFile) {
        // Serial.println("Config file not found.");
        return false;
    }

    // Read the contents of the config file line by line
    String line;
    int rows = 0;
    int **arr = nullptr;

    if (filename.equals("/test01.config")) {
        rows = 9;
        config01.pos = new int*[rows];
        for (int i = 0; i < rows; ++i) {
            config01.pos[i] = new int[3]();
        }
        arr = config01.pos;
    } else if (filename.equals("/test02.config")) {
        rows = 1;
        config02.pos = new int*[rows];
        for (int i = 0; i < rows; ++i) {
            config02.pos[i] = new int[3]();
        }
        arr = config02.pos;
    } else if (filename.equals("/test03.config")) {
        rows = 990;
        config03.pos = new int*[rows];
        for (int i = 0; i < rows; ++i) {
            config03.pos[i] = new int[3]();
        }
        arr = config03.pos;
    }

    // Serial.println("FileconfigName: " + filename);

    while (configFile.available()) {
        line = configFile.readStringUntil('\n');
        line.trim();
        if (line.startsWith("Name=")) {
            if (filename.equals("/test01.config")) {
                config01.name = line.substring(5); // Extract the name from the line
                // Serial.println("Config name: " + config01.name);
                ESPUI.updateControlValue(nameText, config01.name);
            } else if (filename.equals("/test02.config")) {
                config02.name = line.substring(5); // Extract the name from the line
                // Serial.println("Config name: " + config02.name);
                ESPUI.updateControlValue(nameText, config02.name);
            } else if (filename.equals("/test03.config")) {
                config03.name = line.substring(5); // Extract the name from the line
                // Serial.println("Config name: " + config03.name);
                ESPUI.updateControlValue(nameText, config03.name);
            }
        } else if (line.startsWith("Loop=")) {
            if (filename.equals("/test01.config")) {
                config01.loop = line.substring(5).toInt(); // Convert loop value to integer
                //Serial.println("Loop value: " + String(config01.loop));
                ESPUI.updateControlValue(loopText, String(config01.loop));
            } else if (filename.equals("/test02.config")) {
                config02.loop = line.substring(5).toInt(); // Convert loop value to integer
                //Serial.println("Loop value: " + String(config02.loop));
                ESPUI.updateControlValue(loopText, String(config02.loop));
            } else if (filename.equals("/test03.config")) {
                config03.loop = line.substring(5).toInt(); // Convert loop value to integer
                //Serial.println("Loop value: " + String(config03.loop));
                ESPUI.updateControlValue(loopText, String(config03.loop));
            }
        } else if (line.startsWith("Pos=")) {
            if (filename.equals("/test01.config")) {
                config01.numPos = line.substring(4).toInt(); // Convert numPos value to integer
                //Serial.println("Number of positions: " + String(config01.numPos));
                ESPUI.updateControlValue(posText, String(config01.numPos));
            } else if (filename.equals("/test02.config")) {
                config02.numPos = line.substring(4).toInt(); // Convert numPos value to integer
                //Serial.println("Number of positions: " + String(config02.numPos));
                ESPUI.updateControlValue(posText, String(config02.numPos));
            } else if (filename.equals("/test03.config")) {
                config03.numPos = line.substring(4).toInt(); // Convert numPos value to integer
                //Serial.println("Number of positions: " + String(config03.numPos));
                ESPUI.updateControlValue(posText, String(config03.numPos));
            }
        } else if (line.startsWith("Move=")) {
            //  Serial.println("Move positions:");
            if (filename.equals("/test01.config")) {
                for (int i = 0; i < config01.numPos; i++) {
                    line = configFile.readStringUntil('\n');
                    split(line, ',', config01.pos, i);
                    /*Serial.print(config01.pos[i][0]);
                    Serial.print(",");
                    Serial.print(config01.pos[i][1]);
                    Serial.print(",");
                    Serial.println(config01.pos[i][2]);*/
                }
            } else if (filename.equals("/test02.config")) {
                for (int i = 0; i < config02.numPos; i++) {
                    line = configFile.readStringUntil('\n');
                    split(line, ',', config02.pos, i);
                    /*Serial.print(config02.pos[i][0]);
                    Serial.print(",");
                    Serial.print(config02.pos[i][1]);
                    Serial.print(",");
                    Serial.println(config02.pos[i][2]);*/
                }
            } else if (filename.equals("/test03.config")) {
                for (int i = 0; i < config03.numPos; i++) {
                    line = configFile.readStringUntil('\n');
                    split(line, ',', config03.pos, i);
                    /*Serial.print(config03.pos[i][0]);
                    Serial.print(",");
                    Serial.print(config03.pos[i][1]);
                    Serial.print(",");
                    Serial.println(config03.pos[i][2]);*/
                }
            }
        }
    }
    configFile.close();

    return true;
}

// Function to clean up dynamically allocated memory
/*void cleanUp(Config01 &config01, Config02 &config02, Config03 &config03) {
    if (config01.pos) {
        for (int i = 0; i < 9; ++i) {
            delete[] config01.pos[i];
        }
        delete[] config01.pos;
    }
    if (config02.pos) {
        for (int i = 0; i < 1; ++i) {
            delete[] config02.pos[i];
        }
        delete[] config02.pos;
    }
    if (config03.pos) {
        for (int i = 0; i < 961; ++i) {
            delete[] config03.pos[i];
        }
        delete[] config03.pos;
    }
}*/

/**
 * @brief Lists the contents of a directory and its subdirectories.
 * 
 * @param fs The file system object.
 * @param dirname The name of the directory.
 * @param levels The number of levels to list. Use 0 for just the directory itself.
 */
void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    // Initialize the file counter
    int countFile = 0;

    // Print the directory being listed
    // Serial.printf("Listing directory: %s\n", dirname);

    // Open the directory
    File root = fs.open(dirname);

    // Check if the directory was opened successfully
    if (!root)
    {
        // Print an error message if the directory could not be opened
        // Serial.println("Failed to open directory");
        return;
    }

    // Check if the opened file is a directory
    if (!root.isDirectory())
    {
        // Print an error message if the file is not a directory
        // Serial.println("Not a directory");
        return;
    }

    // Initialize the file object
    File file = root.openNextFile();

    // Initialize the file name result string
    fileNameResult = "16:36:48.365 -> server.log.2023-07-25Nserver.log.2023-09-26Nserver.log.2023-09-23Nserver.log.2023-09-22Nserver.log.2023-09-21Nserver.log.2023-09-18Nserver.log.2023-09-08Nserver.log.2023-09-04Nserver.log.2023-09-01Nserver.log.2023-08-30Nserver.log.2023-08-29Nserver.log.2023-08-16Nserver.log.2023-08-15Nserver.log.2023-08-10Nserver.log.2023-07-31Ntest.txtNfoo.txtNtest01.configN";

    // Loop through all files in the directory and its subdirectories
    while (file)
    {
        // Check if the file is a directory
        if (file.isDirectory())
        {
            // Print the directory name and its last write time
            // Serial.print("  DIR : ");
            // Serial.print(file.name());

            time_t t = file.getLastWrite();
            struct tm *tmstruct = localtime(&t);
            // Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);  //คำนวณก่อน sec+1 millisec>1000

            // If the number of levels is greater than 0, recursively list the subdirectories
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            // Print the file name and its size, as well as its last write time
            // Serial.print("  FILE: ");
            // Serial.print(file.name());
            fileNameResult.concat(file.name());

            // Serial.print("  SIZE: ");
            // Serial.print(file.size());

            time_t t = file.getLastWrite();
            struct tm *tmstruct = localtime(&t);
            // Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
        }

        // Open the next file in the directory
        file = root.openNextFile();
    }
}


/**
 * @brief Create a directory on the file system.
 *
 * @param fs The file system to create the directory on.
 * @param path The path of the directory to create.
 *
 * This function creates a directory on the specified file system with the given path.
 * If the directory is successfully created, a message is printed to the Serial
 * interface indicating that the directory was created. If the directory creation fails,
 * a message is printed indicating the failure.
 */
void createDir(fs::FS &fs, const char *path)
{
  // Print the path of the directory being created
  // Serial.printf("Creating Dir: %s\n", path);

  // Attempt to create the directory
  if (fs.mkdir(path))
  {
    // Print a success message if the directory was created
    // Serial.println("Dir created");
  }
  else
  {
    // Print a failure message if the directory creation failed
    // Serial.println("mkdir failed");
  }
}

/**
 * @brief Remove a directory from the file system.
 *
 * @param fs The file system to remove the directory from.
 * @param path The path of the directory to remove.
 *
 * This function removes a directory from the specified file system with the given path.
 * If the directory is successfully removed, a message is printed to the Serial
 * interface indicating that the directory was removed. If the directory removal fails,
 * a message is printed indicating the failure.
 */
void removeDir(fs::FS &fs, const char *path)
{
  // Print the path of the directory being removed
  // Serial.printf("Removing Dir: %s\n", path);

  // Attempt to remove the directory
  if (fs.rmdir(path))
  {
    // Print a success message if the directory was removed
    // Serial.println("Dir removed");
  }
  else
  {
    // Print a failure message if the directory removal failed
    // Serial.println("rmdir failed");
  }
}

/**
 * @brief Read a file from the file system.
 *
 * @param fs The file system to read the file from.
 * @param path The path of the file to read.
 *
 * This function reads a file from the specified file system with the given path.
 * If the file is successfully opened, the contents of the file are printed to the
 * Serial interface. If the file cannot be opened, a message is printed to the
 * Serial interface indicating the failure.
 */
void readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file)
  {
    // Serial.println("Failed to open file for reading");
    return;
  }

  // Read the contents of the file

  while (file.available())
  {
    // Serial.write(file.read());
  }

  file.close();
}

/**
 * @brief Write a message to a file on the file system.
 *
 * @param fs The file system to write the file to.
 * @param path The path of the file to write.
 * @param message The message to write to the file.
 *
 * This function opens a file on the specified file system with the given path,
 * writes the given message to the file, and then closes the file. If the file
 * cannot be opened for writing, a message is printed to the Serial interface
 * indicating the failure. If the write operation fails, a message is printed
 * to the Serial interface indicating the failure.
 */
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  // Print a message indicating the file we are writing to
  // Serial.printf("Writing file: %s\n", path);

  // Open the file for writing
  File file = fs.open(path, FILE_WRITE);

  // If the file failed to open, print a failure message and return
  if (!file)
  {
    // Serial.println("Failed to open file for writing");
    return;
  }

  // Write the message to the file and check if it was successful
  if (file.print(message))
  {
    // If the write was successful, print a success message
    // Serial.println("File written");
  }
  else
  {
    // If the write failed, print a failure message
    // Serial.println("Write failed");
  }

  // Close the file
  file.close();

}

/**
 * @brief Appends a message to a file on the given file system.
 *
 * @param fs The file system to use.
 * @param path The path to the file to append to.
 * @param message The message to append to the file.
 *
 * This function opens a file on the specified file system with the given path,
 * appends the given message to the end of the file, and then closes the file.
 * If the file cannot be opened for appending, a message is printed to the
 * Serial interface indicating the failure. If the append operation fails, a
 * message is printed to the Serial interface indicating the failure.
 */
void appendFile(fs::FS &fs, const char *path, const char *message)
{
  // Print a message indicating the file we are appending to
  // Serial.printf("Appending to file: %s\n", path);

  // Open the file for appending
  File file = fs.open(path, FILE_APPEND);

  // If the file failed to open, print a failure message and return
  if (!file)
  {
    // Serial.println("Failed to open file for appending");
    return;
  }

  // Append the message to the file and check if it was successful
  if (file.print(message))
  {
    // If the append was successful, print a success message
    dataLog = "Message appended";
    ESPUI.updateLabel(logStatus, String(dataLog));
    // Serial.println(dataLog);
  }
  else
  {
    // If the append failed, print a failure message
    dataLog = "Append failed";
    ESPUI.updateLabel(logStatus, String(dataLog));
    // Serial.println(dataLog);
  }

  // Close the file
  file.close();
}

/**
 * Renames a file in the specified filesystem.
 *
 * @param fs The filesystem to rename the file in.
 * @param path1 The current path of the file to rename.
 * @param path2 The new path to rename the file to.
 */
void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
  // Print a message indicating the file we are renaming and to what it is being renamed
  // Serial.printf("Renaming file %s to %s\n", path1, path2);

  // Attempt to rename the file
  if (fs.rename(path1, path2))
  {
    // If the rename was successful, print a success message
    // Serial.println("File renamed");
  }
  else
  {
    // If the rename failed, print a failure message
    // Serial.println("Rename failed");
  }
}

/**
 * Deletes a file in the specified filesystem.
 *
 * @param fs The filesystem to delete the file in.
 * @param path The path of the file to delete.
 */
void deleteFile(fs::FS &fs, const char *path)
{
  // Print a message indicating the file being deleted
  // Serial.printf("Deleting file: %s\n", path);

  // Attempt to delete the file
  if (fs.remove(path))
  {
    // If the deletion was successful, print a success message
    // Serial.println("File deleted");
  }
  else
  {
    // If the deletion failed, print a failure message
    // Serial.println("Delete failed");
  }
}

/**
 * Formats a single digit number into a string with a leading zero if necessary.
 *
 * @param n The number to format.
 * @return A string representation of the number with a leading zero if n < 10.
 */
String a0(int n)
{
  return (n < 10) ? "0" + String(n) : String(n);
}

/**
 * @brief Format a two-digit number into a string with a leading zero if necessary.
 *
 * @param n The number to format.
 * @return A string representation of the number with a leading zero if n < 100.
 */
String a00(int n)
{
  return (n < 100) ? "0" + String(n) : String(n);
}

/**
 * @brief Reads the current weight from the load cell.
 *
 * @details
 * This function reads the current weight from the load cell and returns it as a
 * floating point value. The value is the weight in grams.
 *
 * @return The current weight in grams.
 */
float readLoadCell()
{

  //scale.tare();
  float reading = 0;

  // Check if the scale is ready to take a reading
  if (scale.wait_ready_timeout(200)) {
    // Read the weight from the scale
    reading = abs(round(scale.get_units(1)));
        //Serial.print("HX711 reading: ");
        //Serial.println(reading);
   if (reading < 10){
      reading = 0;  
    }
    else {
    reading = reading + offset;
      }
    
  } else {
    // Serial.println("HX711 not found.");
  }
  
  return reading;
}

void moveLeft()
{
  // Serial.println("moveLeft");
  // delay(100);
}

void moveRight()
{
  // Serial.println("moveRight");
  // delay(100);
}

void lift()
{
  // Serial.println("lift");
  // delay(100);
}
void down()
{
  // Serial.println("down");//
  // delay(100);
}

void press()
{

}
void armLogic()
{
  lift();
  moveRight();
  down();
  // Serial.println(config01.numPos);


}

boolean isStarted()
{

  return true;
}

/**
 * @brief This callback function is used to handle the changed values even though it doesn't do anything itself.
 *
 * @param sender A pointer to the Control object that triggered the callback.
 * @param type The type of event that triggered the callback.
 */
void textCallback(Control *sender, int type) {
  // The purpose of this callback function is to handle the changed values
  // even though it doesn't do anything itself.
}


/**
 * @brief This function handles the movement of the Z-axis.
 *
 * It takes in a Control pointer and an integer type as parameters.
 * The current value of the sender control is converted to an integer.
 * The Z-axis movement is calculated by multiplying the current value with the axisZ value.
 * The function then prints the current Z-axis position to the serial monitor.
 * If the current Z-axis position is greater than the previous Z-axis position,
 * the function prints "up" to the serial monitor and moves the Z-axis up by 100 steps.
 * If the current Z-axis position is less than the previous Z-axis position,
 * the function prints "down" to the serial monitor and moves the Z-axis down by 100 steps.
 * Finally, the function updates the previousZ value to the current Z-axis position.
 *
 * @param sender A pointer to the Control object that triggered the callback.
 * @param type The type of event that triggered the callback.
 */
void moveAxisZ(Control* sender, int type)
{
  // Convert the value of the sender control to an integer
  int currentZ = String(sender->value).toInt();
  
  // Calculate the Z-axis movement by multiplying the current value with the axisZ value
  currentZ = currentZ * (axisZ * 10);
  
  // Print the current Z-axis position to the serial monitor
  // Serial.println("moveAxisZ");
  // Serial.println(currentZ);

  // Check if the current Z-axis position is greater than the previous Z-axis position
  if (currentZ > previousZ) {
    // Print "up" to the serial monitor
    // Serial.println("up");
    
    // Move the Z-axis up by 100 steps
    stepperZ.moveTo(100);
    stepperZ.runToPosition();
    stepperZ.setCurrentPosition(0);

  }
  // Check if the current Z-axis position is less than the previous Z-axis position
  else if (currentZ < previousZ) {
    // Print "down" to the serial monitor
    // Serial.println("down");
    
    // Move the Z-axis down by 100 steps
    stepperZ.moveTo(-100);
    stepperZ.runToPosition();
    stepperZ.setCurrentPosition(0);
  }
  
  // Update the previousZ value to the current Z-axis position
  previousZ = currentZ;
}



// This is the main function which builds our GUI
void setUpUI() {

  //Turn off verbose  ging
  ESPUI.setVerbosity(Verbosity::Quiet);

  //Make sliders continually report their position as they are being dragged.
  ESPUI.sliderContinuous = true;

  //This GUI is going to be a tabbed GUI, so we are adding most controls using ESPUI.addControl
  //which allows us to set a parent control. If we didn't need tabs we could use the simpler add
  //functions like:
  //    ESPUI.button()
  //    ESPUI.label()
  String clearLabelStyle = "background-color: unset; width: 100%;";

  /*
     Tab: Basic Controls
     This tab contains all the basic ESPUI controls, and shows how to read and update them at runtime.
    -----------------------------------------------------------------------------------------------------------*/
  auto maintab = ESPUI.addControl(Tab, "", "Test Plan");

  //ESPUI.addControl(Separator, "Config Status", "", None, maintab);
  //patternStatus = ESPUI.addControl(Label, "Pattern", String(Pattern), Sunflower, maintab);

  ESPUI.addControl(Separator, "General controls", "", None, maintab);
  mainLabel = ESPUI.addControl(Label, "Processing", String(loopCount), Emerald, maintab, generalCallback);

  //  mainSwitcher = ESPUI.addControl(Switcher, "Switcher", "", Sunflower, maintab, generalCallback);
  grouplabel = ESPUI.addControl(Label, "Start/Stop", "File name", Dark, maintab);

  grouplabel2 = ESPUI.addControl(Label, "", "fileName", Emerald, grouplabel);

  ESPUI.addControl(Switcher, "", "1", Sunflower, grouplabel, startButtonCallback);
  ESPUI.setElementStyle(grouplabel2, "font-size: x-large; font-family: serif;");
 
  auto resulttab = ESPUI.addControl(Tab, "", "Results");

  /*
     Tab: Results
     This tab shows how multiple control can be grouped into the same panel through the use of the
     parentControl value. This also shows how to add labels to grouped controls, and how to use vertical controls.
      -----------------------------------------------------------------------------------------------------------*/
  ESPUI.addControl(Separator, "Result download", "", None, resulttab);

  //The parent of this button is a tab, so it will create a new panel with one control.
  auto groupbutton = ESPUI.addControl(Button, "Reload results", "Reload", Dark, resulttab, loadResultCallback);

  resultLabel = ESPUI.addControl(Label, "Result", String(fileName), Emerald, resulttab, loadResultCallback);
  selectDownload = ESPUI.addControl( ControlType::Select, "Select Title", "", Emerald, resultLabel );

  downloadButton = ESPUI.addControl(Button, "", "Download", Dark, resultLabel, downloadCallback);

  ESPUI.updateLabel(resultLabel, String(fileNameResult));

  /*
     Tab: Example UI
     An example UI for the documentation

    -----------------------------------------------------------------------------------------------------------*/

  auto settingTab = ESPUI.addControl(Tab, "Setting", "Setting");
  //  ESPUI.addControl(Separator, "Control and Status", "", None, settingTab);
  //  ESPUI.addControl(Switcher, "Power", "1", Alizarin, settingTab, generalCallback);
  //  ESPUI.addControl(Label, "Status", "System status: OK", Wetasphalt, settingTab, generalCallback);

  //  ESPUI.addControl(Separator, "Settings", "", None, settingTab);
  //  ESPUI.addControl(PadWithCenter, "Attitude Control", "Axis", Dark, settingTab, generalCallback);
  //  auto examplegroup1 = ESPUI.addControl(Button, "Activate Features", "Feature A", Carrot, settingTab, generalCallback);
  //  ESPUI.addControl(Button, "Activate Features", "Feature B", Carrot, examplegroup1, generalCallback);
  //  ESPUI.addControl(Button, "Activate Features", "Feature C", Carrot, examplegroup1, generalCallback);
  //  ESPUI.addControl(Slider, "Value control", "45", Peterriver, settingTab, generalCallback);

  ESPUI.addControl(Separator, "Calibrating Axis", "", None, settingTab);
  //  ESPUI.addControl(Pad, "Normal", "", Peterriver, settingTab, generalCallback);
  ESPUI.addControl(PadWithCenter, "Axis X,Y", "", Peterriver, settingTab, moveAxisXY);

  //Number inputs also accept Min and Max components, but you should still validate the values.
  settingZNumber = ESPUI.addControl(Number, "Axis Z", String(axisZ), Emerald, settingTab, &moveAxisZ);

  ESPUI.addControl(Min, "", "-50", None, settingZNumber);
  ESPUI.addControl(Max, "", "50", None, settingZNumber);

 /* ESPUI.addControl(Separator, "Off Set", "", None, settingTab);
  offSet1 = ESPUI.addControl(Number, "Off Set1", String(offset1), None, settingTab);
  offSet2 = ESPUI.addControl(Number, "Off Set2", String(offset2), None, settingTab);
  offSet3 = ESPUI.addControl(Number, "Off Set3", String(offset3), None, settingTab);
  offSet4 = ESPUI.addControl(Number, "Off Set4", String(offset4), None, settingTab);*/
    
  ESPUI.addControl(Separator, "Setting Configuration Pattern", "", None, settingTab);
  posText = ESPUI.addControl(Select, "Pattern", "", None, settingTab, setTextInputCallback);
  ESPUI.addControl(Option, "Not Selected", "", None, posText, setTextInputCallback);
  ESPUI.addControl(Option, "3x3", "1", None, posText, setTextInputCallback);
  ESPUI.addControl(Option, "Center Point", "2", None, posText, setTextInputCallback);
  ESPUI.addControl(Option, "30x30", "3", None, posText, setTextInputCallback);
  loopText = ESPUI.addControl(ControlType::Number, "Loop", "", ControlColor::None, settingTab, setTextInputCallback);
  depthText = ESPUI.addControl(ControlType::Number, "Depth press (mm. X 0.1)", String(depth), ControlColor::None, settingTab, setTextInputCallback);
  intervalPress = ESPUI.addControl(ControlType::Number, "The interval between presses (seconds)", String(interval), ControlColor::None, settingTab, setTextInputCallback);
  maxDept = ESPUI.addControl(Number, "Maximun of Depth (mm.)", "5", None, settingTab, setTextInputCallback);

  //  ESPUI.addControl(Separator, "Setting Config File", "", None, settingTab);

  //  nameText = ESPUI.addControl(ControlType::Text, "Name", "", ControlColor::None, settingTab, setTextInputCallback);
  //  loopText = ESPUI.addControl(ControlType::Text, "Loop", "", ControlColor::None, settingTab, setTextInputCallback);  
  //  moveText = ESPUI.addControl(ControlType::Text, "Move", "", ControlColor::None, settingTab, setTextInputCallback);

  //  saveConfigButton = ESPUI.addControl(ControlType::Button, "Save Config", "Save", ControlColor::None, settingTab, configButtonCallback);

  /*
     Tab: WiFi Credentials
     You use this tab to enter the SSID and password of a wifi network to autoconnect to.
    -----------------------------------------------------------------------------------------------------------*/
  auto wifitab = ESPUI.addControl(Tab, "", "WiFi Credentials");
  wifi_ssid_text = ESPUI.addControl(Text, "SSID", "", Alizarin, wifitab, textCallback);
  //Note that adding a "Max" control to a text control sets the max length
  ESPUI.addControl(Max, "", "32", None, wifi_ssid_text);
  wifi_pass_text = ESPUI.addControl(Text, "Password", "", Alizarin, wifitab, textCallback);
  ESPUI.addControl(Max, "   ", "64", None, wifi_pass_text);
  ESPUI.addControl(Button, "Save", "Save", Peterriver, wifitab, enterWifiDetailsCallback);
  
  String printValue = "Download<script>document.getElementById('btn12').onclick = function() {var d = document.createElement('a');d.href = '/download/' + document.querySelector('#select11').value;d.download = document.querySelector('#select11').value; d.click()};fetch('/filelist').then(function(data){return data.text()}).then(function(text){console.log(text.split('\\n').forEach(function(i){var option = document.createElement('option');option.value = i;option.innerText = i;document.querySelector('#select11').append(option);}))})</script>";
  ESPUI.print(downloadButton, printValue);

  //Finally, start up the UI.
  //This should only be called once we are connected to WiFi.
  ESPUI.begin(HOSTNAME);

  auto eventTab = ESPUI.addControl(Tab, "", "Event Log");
  SDcardRead = ESPUI.addControl(Label, "SD Card Status", String(cardStatus), Alizarin, eventTab, generalCallback);
  ADSStatus = ESPUI.addControl(Label, "ADS Status", String(ADS), Alizarin, eventTab, generalCallback);

  logStatus = ESPUI.addControl(Label, "Record Status", String(dataLog), Alizarin, eventTab, generalCallback);

}

//This callback generates and applies inline styles to a bunch of controls to change their colour.

void setTextInputCallback(Control *sender, int type) {
  // Serial.println(sender->value);
  ESPUI.updateControl(sender);
  /*Control* pos_ = ESPUI.getControl(posText);
  Serial.println("posText: " + pos_->value);*/
  
}

/*void configButtonCallback(Control *sender, int type) {
  String file = "";
  if (type == B_UP) {
    Control* name_ = ESPUI.getControl(nameText);
    //  ESPUI.updateControl(nameText);
    Control* loop_ = ESPUI.getControl(loopText);
    //  ESPUI.updateControl(loopText);
    Control* pos_ = ESPUI.getControl(posText);
    //  ESPUI.updateControl(posText);
    file += "Name=";
    file += name_->value;
    file += "\n";
    file += "Loop=";
    file += loop_->value;
    file += "\n";
    file += "Pos=";
    file += pos_->value;
    file += "\n";
    file += "Move=\n";
    file += "-1000,-1000,-4\n1000,0,-4\n1000,0,-4\n0,1000,-4\n-1000,0,-4\n-1000,0,-4\n0,1000,-4\n1000,0,-4\n1000,0,-4\nEnd";
    Serial.println(file);
    writeFile(SD, "/test01.config", file.c_str());
    config.name = name_->value;
    config.loop = loop_->value.toInt();
    config.numPos = pos_->value.toInt();
  }
}*/

void styleCallback(Control *sender, int type) {
  //Declare space for style strings. These have to be static so that they are always available
  //to the websocket layer. If we'd not made them static they'd be allocated on the heap and
  //will be unavailable when we leave this function.
  static char stylecol1[60], stylecol2[30];
  if (type == B_UP) {
    //Generate two random HTML hex colour codes, and print them into CSS style rules
    sprintf(stylecol1, "border-bottom: #999 3px solid; background-color: #%06X;", (unsigned int) random(0x0, 0xFFFFFF));
    sprintf(stylecol2, "background-color: #%06X;", (unsigned int) random(0x0, 0xFFFFFF));

    //Apply those styles to various elements to show how controls react to styling
    ESPUI.setPanelStyle(styleButton, stylecol1);
    ESPUI.setElementStyle(styleButton, stylecol2);
    ESPUI.setPanelStyle(styleLabel, stylecol1);
    ESPUI.setElementStyle(styleLabel, stylecol2);
    ESPUI.setPanelStyle(styleSwitcher, stylecol1);
    ESPUI.setElementStyle(styleSwitcher, stylecol2);
    ESPUI.setPanelStyle(styleSlider, stylecol1);
    ESPUI.setElementStyle(styleSlider, stylecol2);
  }
}


void updateCallback(Control *sender, int type) {
  updates = (sender->value.toInt() > 0);
}

void getTimeCallback(Control *sender, int type) {
  if (type == B_UP) {
    ESPUI.updateTime(mainTime);
  }
}

void graphAddCallback(Control *sender, int type) {
  if (type == B_UP) {
    ESPUI.addGraphPoint(graph, random(1, 50));
  }
}

void graphClearCallback(Control *sender, int type) {
  if (type == B_UP) {
    ESPUI.clearGraph(graph);
  }
}

void downloadCallback(Control *sender, int type) {
  //  Control* labelControl = ESPUI.getControl(grouplabel2);
  //  sender->callback = nullptr;  // Prevent infinite loop
  //  //ESPUI.print(sender->id, redirectScript);
  //  sender->callback = downloadCallback;  // Restore the callback
}

void startButtonCallback(Control *sender, int type) {
  // Serial.print("CB: id(");
  // Serial.print(sender->id);
  // Serial.print(") Type(");
  // Serial.print(type);
  // Serial.print(") '");
  // Serial.print(sender->label);
  // Serial.print("' = ");
  // Serial.println(sender->value);
  boolean isPress = String(sender->value).toInt();
  // Serial.print("isPress:"); Serial.println(isPress);
  if (isPress)
    isStopStart = true;
  else
    isStopStart = false;

}
//Most elements in this test UI are assigned this generic callback which prints some
//basic information. Event types are defined in ESPUI.h
void generalCallback(Control *sender, int type) {
  // Serial.print("CB: id(");
  // Serial.print(sender->id);
  // Serial.print(") Type(");
  // Serial.print(type);
  // Serial.print(") '");
  // Serial.print(sender->label);
  // Serial.print("' = ");
  // Serial.println(sender->value);
}
/**
 * \brief Callback function for the load result button
 * \param sender The control that was clicked
 * \param type The type of the event that was triggered
 */

void loadResultCallback(Control *sender, int type) {
  // Serial.print("CB: id(");
  // Serial.print(sender->id);
  // Serial.print(") Type(");
  // Serial.print(type);
  // Serial.print(") '");
  // Serial.print(sender->label);
  // Serial.print("' = ");
  // Serial.println(sender->value);

  // Update the label with the file name
  ESPUI.updateLabel(grouplabel2, String(fileName));

  // Update the result label
  ESPUI.updateControl(resultLabel);

  // Append the record to the file
  //  listDir(SD, "/", 0);
  // readFile(SD, fileName.c_str());
  //  readFile(SD, "/server.log.2023-07-31");
  record.concat("Date/Time,Loop,Round,Value1,Value2,Value3,Value4,Value5,Value6,Value7,Value8,Value9,Value10,Value11,Value12,LoadCell\n");
  appendFile(SD, fileNameResult.c_str(), record.c_str());
  record = "";
  // Serial.println("Result:");
  // Serial.println(fileNameResult);


}

/**
 * \brief Callback function to move the X and Y axes
 * \param sender The control that was clicked
 * \param type The type of the event that was triggered
 *
 * This function is called when the X or Y axis is to be moved. It takes the
 * control that was clicked and the type of the event as parameters. The
 * function then determines which direction to move the axis based on the type
 * of the event. The axis is moved using the AccelStepper library.
 */
void moveAxisXY(Control *sender, int type) {
  // Print a message indicating the axis being moved
  // Serial.println("moveAxisXY");

  // Print information about the control that was clicked
  // Serial.print("CB: id(");
  // Serial.print(sender->id);
  // Serial.print(") Type(");
  // Serial.print(type);
  // Serial.print(") '");
  // Serial.println(sender->label);

  // Determine the direction to move the axis based on the type of event
  if (4 == abs(type)) {  // Up
    // Serial.println("up");
    stepperY.moveTo(100);  // Move the Y axis up
    stepperY.runToPosition();  // Run the Y axis to the new position
    stepperY.setCurrentPosition(0);  // Reset the Y axis position
  }

  if (3 == abs(type)) {  // Right
    // Serial.println("right");
    stepperX.moveTo(-100);  // Move the X axis right
    stepperX.runToPosition();  // Run the X axis to the new position
    stepperX.setCurrentPosition(0);  // Reset the X axis position
  }

  if (2 == abs(type)) {  // Left
    // Serial.println("left");
    stepperX.moveTo(100);  // Move the X axis left
    stepperX.runToPosition();  // Run the X axis to the new position
    stepperX.setCurrentPosition(0);  // Reset the X axis position
  }

  if (5 == abs(type)) {  // Down
    // Serial.println("down");
    stepperY.moveTo(-100);  // Move the Y axis down
    stepperY.runToPosition();  // Run the Y axis to the new position
    stepperY.setCurrentPosition(0);  // Reset the Y axis position
  }
}


// Most elements in this test UI are assigned this generic callback which prints some
// basic information. Event types are defined in ESPUI.h
// The extended param can be used to hold a pointer to additional information
// or for C++ it can be used to return a this pointer for quick access
// using a lambda function
void extendedCallback(Control* sender, int type, void* param)
{
  // Serial.print("CB: id(");
  // Serial.print(sender->id);
  // Serial.print(") Type(");
  // Serial.print(type);
  // Serial.print(") '");
  // Serial.print(sender->label);
  // Serial.print("' = ");
  // Serial.println(sender->value);
  // Serial.println(String("param = ") + String((int)param));
}

// Task1code: blinks an LED every 1000 ms
void Task1code(void *pvParameters)
{

  for (;;)
  {
    //    Serial.print("Task1 running on core ");
    //    Serial.println(xPortGetCoreID());
        heartBeat();
    vTaskDelay((2000) / portTICK_PERIOD_MS);
  }
}

// Task2code: blinks an LED every 700 ms
/*
void Task2code(void *pvParameters)
{
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());
  //  Serial.println(ESP.getFreeHeap());
  for (;;)
  {
  }
}

// Task2code: blinks an LED every 700 ms
void Task3code(void *pvParameters)
{

  for (;;)
  {
    //    if (isStopStart && (testCount <= config.loop) ) {
    //
    //
    //
    //      //      diffMillis = millis();
    //      //      Serial.println(); // Move to the next row
    //      Serial.println(fileName);
    //
    //    }
    //    vTaskDelay((1000) / portTICK_PERIOD_MS);
  }
}*/

/**
 * @brief Initializes all the ADS1115 sensors. 
 * 
 * Sets the gain and data rate of each sensor. If any sensor fails to initialize, 
 * it will print an error message and return false.
 * 
 * @return true if all sensors initialized successfully, false otherwise.
 */
bool initializeSensors() {
  
  bool success = true; // Flag to indicate if all sensors initialized successfully

  // Initialize ADS1115 (0x48)
  ads1.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  ads1.setDataRate(4);

  // Initialize ADS1115 (0x49)
  ads2.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  ads2.setDataRate(4);

  // Initialize ADS1115 (0x4A)
  ads3.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  ads3.setDataRate(4);

  // Initialize ADS1115 (0x4B)
  ads4.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  ads4.setDataRate(4);

  if (!ads1.begin(0x48) || !ads2.begin(0x49) || !ads3.begin(0x4A) || !ads4.begin(0x4B)) {
    ADS = "Failed to initialize ADS1115!";
    ESPUI.updateLabel(ADSStatus, String(ADS));
    // Serial.println(ADS);
    success = false;
    while(1);
  }
  
  return success;

}


/**
 * @brief Move the steppers to the starting position based on the position selected by the user
 *
 * This function moves the steppers to the starting position based on the position selected by the user
 * in the UI. The position can be either 1, 2, or 3, which correspond to the following positions:
 * - Position 1: -1000, -1000
 * - Position 2: 0, 0
 * - Position 3: -1000, -1000 (300x300 mm)
 */
void moveToStart() {
  // Get the position selected by the user from the UI
  //  Serial.println("move2start");
  
  Control* pos_ = ESPUI.getControl(posText);

  if (pos_->value.equals("1")){ //20x20 mm. or 3x3 pattern
    stepperX.moveTo(-1000);
    stepperX.runToPosition();
    stepperX.setCurrentPosition(0);

    stepperY.moveTo(-1000);
    stepperY.runToPosition();
    stepperY.setCurrentPosition(0);
  }
    // Move the steppers to the starting position
  else if (pos_->value.equals("2")){ //1x1
    stepperX.moveTo(0);
    stepperX.runToPosition();
    stepperX.setCurrentPosition(0);

    stepperY.moveTo(0);
    stepperY.runToPosition();
    stepperY.setCurrentPosition(0);
  }
    
    // Move the steppers to the starting position
  else if (pos_->value.equals("3")){ //300x300 mm. or 30x30 pattern

    for (int x = 0; x < 15; x++){
      stepperX.moveTo(-1000);
      stepperX.runToPosition();
      stepperX.setCurrentPosition(0);
    }

    for (int x = 0; x < 15; x++){
      stepperY.moveTo(-1000);
      stepperY.runToPosition();
      stepperY.setCurrentPosition(0);
    }
  }

}

float I0(float value){
  
  I = ((5 - value) / R1)  * 1000;
  return I;
  }

float R0(float value){
  float R0 = ((R2 * value) / (((I * 0.001) * R2 ) - value )) * 0.000001 ; 
  return R0;
  }

//Utilities
//
//If you are here just to see examples of how to use ESPUI, you can ignore the following functions
//------------------------------------------------------------------------------------------------
void readStringFromEEPROM(String & buf, int baseaddress, int size) {
  buf.reserve(size);
  for (int i = baseaddress; i < baseaddress + size; i++) {
    char c = EEPROM.read(i);
    buf += c;
    if (!c) break;
  }
}

void connectWifi() {
  int connect_timeout;

#if defined(ESP32)
  WiFi.setHostname(HOSTNAME);
#else
  WiFi.hostname(HOSTNAME);
#endif
  //  Serial.println("Begin wifi...");

  //Load credentials from EEPROM
  if (!(FORCE_USE_HOTSPOT)) {
    yield();
    EEPROM.begin(100);
    String stored_ssid, stored_pass;
    readStringFromEEPROM(stored_ssid, 0, 32);
    readStringFromEEPROM(stored_pass, 32, 96);
    EEPROM.end();

    //Try to connect with stored credentials, fire up an access point if they don't work.
#if defined(ESP32)
    WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());
#else
    WiFi.begin(stored_ssid, stored_pass);
#endif
    connect_timeout = 28; //7 seconds
    while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) {
      delay(250);
      // Serial.print(".");
      connect_timeout--;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    // Serial.println(WiFi.localIP());
    // Serial.println("Wifi started");

    if (!MDNS.begin(HOSTNAME)) {
      // Serial.println("Error setting up MDNS responder!");
    }
  } else {
    // Serial.println("\nCreating access point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(HOSTNAME);

    connect_timeout = 20;
    do {
      delay(250);
      // Serial.print(",");
      connect_timeout--;
    } while (connect_timeout);
  }
}

void enterWifiDetailsCallback(Control * sender, int type) {
  if (type == B_UP) {
    // Serial.println("Saving credentials to EPROM...");
    // Serial.println(ESPUI.getControl(wifi_ssid_text)->value);
    // Serial.println(ESPUI.getControl(wifi_pass_text)->value);
    unsigned int i;
    EEPROM.begin(100);
    for (i = 0; i < ESPUI.getControl(wifi_ssid_text)->value.length(); i++) {
      EEPROM.write(i, ESPUI.getControl(wifi_ssid_text)->value.charAt(i));
      if (i == 30) break; //Even though we provided a max length, user input should never be trusted
    }
    EEPROM.write(i, '\0');

    for (i = 0; i < ESPUI.getControl(wifi_pass_text)->value.length(); i++) {
      EEPROM.write(i + 32, ESPUI.getControl(wifi_pass_text)->value.charAt(i));
      if (i == 94) break; //Even though we provided a max length, user input should never be trusted
    }
    EEPROM.write(i + 32, '\0');
    EEPROM.end();
  }
}  

float readVoltage(Adafruit_ADS1115 &ads, int channel, float offset) {
  // อ่านค่าดิบจากช่องที่เลือก
  int16_t raw = ads.readADC_SingleEnded(channel);
  
  // แปลงค่าดิบเป็นแรงดัน (ADC ใช้ 16 บิตสำหรับค่าแรงดัน)
  //  float voltage = (raw * 0.1875) / 1000.0; // 0.1875mV ต่อขั้น สำหรับ ±6.144V
    float voltage = ads.computeVolts(raw);
  
  // ปรับค่าด้วย Offset และ Gain
  //voltage = (voltage * gain) + offset;
  //  voltage = voltage + offset;
   
  return voltage;
}

void readADS(){
   for (int i = 0; i < 3; i++){ 
          float volt10 = readVoltage(ads1, i, offset1);
          amp10 = I0(volt10);
          Res10 = R0(volt10);
          record.concat(String(volt10, 4));
          record.concat(",");
          record.concat(String(amp10, 4));
          record.concat(",");
          record.concat(String(Res10, 4));
          record.concat(",");
          // record.concat(" ");
     }

      for (int i = 0; i < 3; i++){
          float volt20 = readVoltage(ads2, i, offset2);
          amp20 = I0(volt20);
          Res20 = R0(volt20);
          record.concat(String(volt20, 4));
          record.concat(",");
          record.concat(String(amp20, 4));
          record.concat(",");
          record.concat(String(Res20, 4));
          record.concat(",");
          // record.concat(" ");
      }

      for (int i = 0; i < 3; i++){
            float volt30 = readVoltage(ads3, i, offset3);
            amp30 = I0(volt30);
            Res30 = R0(volt30);
            record.concat(String(volt30, 4));
            record.concat(",");
            record.concat(String(amp30, 4));
            record.concat(",");
            record.concat(String(Res30, 4));
            record.concat(",");
            // record.concat(" ");
      }

      for (int i = 0; i < 3; i++){
            float volt40 = readVoltage(ads4, i, offset4);
            amp40 = I0(volt40);
            Res40 = R0(volt40);
            record.concat(String(volt40, 4));
            record.concat(",");
            record.concat(String(amp40, 4));
            record.concat(",");
            record.concat(String(Res40, 4));
            record.concat(",");
            // record.concat(" ");
      }
}


void setup() {


  randomSeed(0);
  Serial.begin(115200);

//Start I2C section
   Wire.begin();
  Wire.setClock(3400000); // Set I2C clock frequency to 400 kHz
  // Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  // Serial.println("+/- 1.024V  1 bit = 0.5mV    0.03125mV");

//Start WiFi section
  while (!Serial);
  if (SLOW_BOOT) delay(5000); //Delay booting to give time to connect a serial monitor
  connectWifi();
#if defined(ESP32)
  WiFi.setSleep(false); //For the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)
#endif

  setUpUI(); //Start the GUI

  // Serial.print("Connecting to ");
  // Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
  }
  // Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // Serial.println("Contacting Time Server");
  // configTime(3600 * timezone, daysavetime * 3600, "0.pool.ntp.org", "time.nist.gov", "1.pool.ntp.org"); //configTime(3600 * timezone, daysavetime * 3600, "time.nist.gov", "1.pool.ntp.org", "0.pool.ntp.org");
  // บันทึกเวลาสำหรับการ sync ครั้งแรก
  millisAtLastSync = millis();
  lastEpoch = mktime(&tmstruct);
  delay(2000);
  tmstruct.tm_year = 0;
  getLocalTime(&tmstruct, 5000);
  yearStr = String(tmstruct.tm_year + 1900, DEC);
  monthStr = String(tmstruct.tm_mon + 1, DEC);
  dayStr = String(tmstruct.tm_mday, DEC);
  hourStr = String(a0(tmstruct.tm_hour));
  minStr = String(a0(tmstruct.tm_min));
  secStr = String(a0(tmstruct.tm_sec));
  fileName.concat(yearStr);
  fileName.concat(monthStr);
  fileName.concat(dayStr);
  fileName.concat(hourStr);
  fileName.concat(minStr);
  fileName.concat(".csv");
  // Serial.printf("\nNow is : %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
  // Serial.println("");

//Start SD card section
  if (!SD.begin())
  {
    cardStatus = "Card Mount Failed";
    ESPUI.updateLabel(SDcardRead, String(cardStatus));
    // Serial.println(cardStatus);
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    // Serial.println("No SD card attached");
    return;
  }

  // Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    // Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    // Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    // Serial.println("SDHC");
  }
  else
  {
    // Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  // Serial.printf("SD Card Size: %lluMB\n", cardSize);

  

  diff = millis();

  // Initialize SD card
  //  if (!SD.begin())
  //  {
  //    Serial.println("SD Card initialization failed!");
  //    return;
  //  }
  //  Serial.println("SD Card initialized");


  //  xTaskCreate(Task1code, "Task1", 10000, NULL, tskIDLE_PRIORITY, NULL);
  //
  //  xTaskCreate(Task2code, "Task2", 10000, NULL, tskIDLE_PRIORITY, NULL);
  //  xTaskCreate(Task3code, "Task3", 20000, NULL, 1, NULL);

//End SD card section

//Start set up stepper motors
  stepperX.setMaxSpeed(4000);
  stepperX.setAcceleration(6000);
  stepperX.setCurrentPosition(0);

  stepperY.setMaxSpeed(4000);
  stepperY.setAcceleration(6000);
  stepperY.setCurrentPosition(0);

  stepperZ.setMaxSpeed(4000);
  stepperZ.setAcceleration(6000);
  stepperZ.setCurrentPosition(0);
//End set up stepper motors

//Start read config files
  if (readConfig("/test01.config", config01, config02, config03)) {
    // Print configuration data

  }

  if (readConfig("/test02.config", config01, config02, config03)) {
    // Print configuration data

  }

  if (readConfig("/test03.config", config01, config02, config03)) {
    // Print configuration data

  }
//End read config files

  initializeSensors(); //Start ADS1115 sensors

//Start scale section
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_gain(128);
  scale.set_scale(CALIBRATION_FACTOR); // this value is obtained by calibrating the scale with known weights
  //scale.set_offset(zero_factor);
  scale.tare();
//End scale section  

  // Serial.print("Free heap memory: ");
  // Serial.print(ESP.getFreeHeap());
  // Serial.println(" bytes");

}

void loop() {
  static long unsigned lastTime = 0;

  //Send periodic updates if switcher is turned on
  if (updates && millis() > lastTime + 500) {
    static uint16_t sliderVal = 10;

    //Flick this switcher on and off
    ESPUI.updateSwitcher(mainSwitcher, ESPUI.getControl(mainSwitcher)->value.toInt() ? false : true);

    lastTime = millis();

    }

  //Simple   UART interface
  if (Serial.available()) {
    switch (Serial.read()) {
      case 'w': //Print IP details
        // Serial.println(WiFi.localIP());
        break;
      case 'W': //Reconnect wifi
        connectWifi();
        break;
      case 'C': //Force a crash (for testing exception decoder)
#if !defined(ESP32)
        ((void (*)())0xf00fdead)();
#endif
      default:
        break;
    }
  }

#if !defined(ESP32)
  //We don't need to call this explicitly on ESP32 but we do on 8266lineCount

  MDNS.update();
#endif
  Control* loop_  = ESPUI.getControl(loopText);
  Control* pos_   = ESPUI.getControl(posText);
  Control* depth_ = ESPUI.getControl(depthText);
  Control* interval_ = ESPUI.getControl(intervalPress);
  Control* max_ = ESPUI.getControl(maxDept);
  depth = -(depth_->value.toInt()) * 10;
  interval = interval_->value.toInt();
  float mm = abs(depth_->value.toFloat()) / 10;
  float deptStep = 0.0;
  int max = max_->value.toFloat();
  int calD = -400;

//Start of main loop
  if (isStopStart && (loopCount <= loop_->value.toInt() )) {
        //Serial.print("loopCount: ");
        //Serial.println(loopCount);
        //Serial.print("Loop: ");
        //Serial.println(config.loop);
        // Serial.printf("Check loopCount: %d loop input: %d \n", loopCount, loop_->value.toInt());

    // tmstruct.tm_year is years since 1900, so we add 1900 to get the full year
    yearStr = String(tmstruct.tm_year + 1900, DEC);
    // tmstruct.tm_mon is months since January, so we add 1 to get a 1-based month
    monthStr = String(tmstruct.tm_mon + 1, DEC);
    if (pos_->value.equals("1")){ 
      //testCount > config01.numPos loopCount
      
    for ( int x = 0; x < config01.numPos; x++) {
      /*Serial.println("testcount:" + String(testCount));
      Serial.println("config01.numPos:" + String(config01.numPos));
      Serial.println("loopCount:" + String(loopCount));*/
      int lineCount = 0;
      int _value = 0;
            // Serial.print("x:");
            // Serial.println(config01.pos[x][0]);
      stepperX.moveTo(config01.pos[x][0]);
      stepperX.runToPosition();
      stepperX.setCurrentPosition(0);


            // Serial.print("y:");
            // Serial.println(config01.pos[x][1]);
      stepperY.moveTo(config01.pos[x][1]);
      stepperY.runToPosition();
      stepperY.setCurrentPosition(0);
      //      Serial.println("");

      //int depthPress = abs(config01.pos[0][2]);

      //      Serial.println("press...");

      //  Serial.println(depthPress);
  //Start pressing material
      stepperZ.moveTo(calD);
      stepperZ.runToPosition();
      stepperZ.setCurrentPosition(0);

      while (deptStep <= max){
        stepperZ.moveTo(depth);
        stepperZ.runToPosition();
        stepperZ.setCurrentPosition(0);

  //Start recording data 
      for (int l = 0; l < interval; l++){

      // record.concat(getFormattedDateTime());
      record.concat(String(deptStep));
      record.concat(",");
      record.concat(loopCount);
      record.concat(",");
      record.concat(testCount);
      record.concat(",");
      record.concat(l);
      record.concat(",");
      // record.concat(" ");

  // Read all ADS1115 channels
      readADS();

      // Read load cell
    loadCellReading = readLoadCell();
      readloadcell = loadCellReading * g * 0.001;
      record.concat(String(loadCellReading));
      record.concat(",");
      record.concat(String(readloadcell));
      // record.concat("\n");
      
      // appendFile(SD, fileName.c_str(), record.c_str());
      Serial.println(record);
      dateTimeStr = "";
      record = "";
      //delayMicroseconds(700000);
      }
      deptStep += mm;
      //scale.tare(); // reset the scale to 0

      }
      deptStep = max - mm;      

      record.concat("\n");
      
  //Stop pressing material
      while(deptStep >= 0.0){
        stepperZ.moveTo(abs(depth));
        stepperZ.runToPosition();
        stepperZ.setCurrentPosition(0);

    for (int l = 1; l <= interval; l++){

  //Start record data while load cell is hanging in the air
      // record.concat(getFormattedDateTime());
      record.concat(String(deptStep));
      record.concat(",");
      record.concat(loopCount);
      record.concat(",");
      record.concat(testCount);
      record.concat(",");
      record.concat(l);
      record.concat(",");
      // record.concat(" ");
      
      // Read all ADS1115 channels
      readADS();

      // Read load cell
    loadCellReading = readLoadCell();
      readloadcell = loadCellReading * g * 0.001;
      record.concat(String(loadCellReading));
      record.concat(",");
      record.concat(String(readloadcell));
      // record.concat("\n");
      
      // appendFile(SD, fileName.c_str(), record.c_str());
      Serial.println(record);
      dateTimeStr = "";
      record = "";
      
      scale.tare(); // reset the scale to 0
      record.concat("\n");
    }
    deptStep -= mm;
    }
      testCount++;
      stepperZ.moveTo(abs(calD));
      stepperZ.runToPosition();
      stepperZ.setCurrentPosition(0);
//End 1 step process

//Checking for complete 9 steps
    if (testCount > config01.numPos) {
     
      testCount = 1;
      moveToStart();

    }
          } 
    }

//Start 1x1 pattern
    else if (pos_->value.equals("2")){

    for ( int x = 0; x < config02.numPos; x++) {

      int _value = 0;
      //      Serial.print("x:");
      //      Serial.print(config02.pos[x][0]);
      //stepperX.moveTo(config02.pos[x][0]);
      //stepperX.runToPosition();
      //stepperX.setCurrentPosition(0);


      //      Serial.print("y:");
      //      Serial.print(config02.pos[x][1]);
      //stepperY.moveTo(config02.pos[x][1]);
      //stepperY.runToPosition();
      //stepperY.setCurrentPosition(0);
      //      Serial.println("");

      //int depthPress = abs(config02.pos[0][2]);

      //      Serial.println("press...");
  //Start press material    
        // Serial.println("depthPress");
        // Serial.println("depstep 1 :" + String(deptStep));
        // Serial.println("mm 1 :" + String(mm));
        stepperZ.moveTo(calD);
        stepperZ.runToPosition();
        stepperZ.setCurrentPosition(0);

      while (deptStep <= max){
        stepperZ.moveTo(depth);
        stepperZ.runToPosition();
        stepperZ.setCurrentPosition(0);

    //Start record data
        for (int l = 1; l <= interval; l++) {
        // record.concat(getFormattedDateTime());
        record.concat(String(deptStep));
        record.concat(",");
        record.concat(loopCount);
        record.concat(",");
        record.concat(l);
        record.concat(",");
        // record.concat(" ");

        readADS(); // Read all ADS1115 channels

        // Read load cell
        loadCellReading = readLoadCell();
        readloadcell = loadCellReading * g * 0.001;
        record.concat(String(loadCellReading));
        record.concat(",");
        record.concat(String(readloadcell));
        // record.concat("\n");

        // appendFile(SD, fileName.c_str(), record.c_str());
        Serial.println(record);
        dateTimeStr = "";
        record = "";
        //scale.tare(); // reset the scale to 0
        }
      deptStep += mm;
      // Serial.println("depstep 3 :" + String(deptStep));
      // Serial.println("mm 3 :" + String(mm));
      //scale.tare(); // reset the scale to 0

      }
      deptStep = max - mm;
      // Serial.println("depstep 4 :" + String(deptStep));
      // Serial.println("mm 4 :" + String(mm));

  //end record data for 4 seconds
      // record.concat("\n");
      //deptStep = 0.00;
      //delayMicroseconds(1000000);

  //Stop press material
  while(deptStep >= 0.0){
    stepperZ.moveTo(abs(depth));
    stepperZ.runToPosition();
    stepperZ.setCurrentPosition(0);
      for (int l = 1; l <= interval; l++){
  //Start record data while load cell is hanging in the air
        // record.concat(getFormattedDateTime());
        record.concat(String(deptStep));
        record.concat(",");
        record.concat(loopCount);
        record.concat(",");
        record.concat(l);
        record.concat(",");
        // record.concat(" ");
      
        readADS();// Read all ADS1115 channels

      // Read load cell
        loadCellReading = readLoadCell();
        readloadcell = loadCellReading * g * 0.001;
        record.concat(String(loadCellReading));
        record.concat(",");
        record.concat(String(readloadcell));
        // record.concat("\n");
        
        // appendFile(SD, fileName.c_str(), record.c_str());
        Serial.println(record);
        dateTimeStr = "";
        record = "";
        //scale.tare(); // reset the scale to 0
      }
      
      deptStep -= mm;
      //scale.tare(); // reset the scale to 0
      // Serial.println("loop1:" + String(loopCount));
  }
  // loopCount++;
  testCount++;
  stepperZ.moveTo(abs(calD));
  stepperZ.runToPosition();
  stepperZ.setCurrentPosition(0);
  // Serial.println("loop2:" + String(loopCount));
      
      //scale.tare(); // reset the scale to 0
      // record.concat("\n");
//End 1 step process

//Checking for complete test steps

    if (testCount > config02.numPos) {
      
      testCount = 1;
      //moveToStart();

    }
   
    } 
   }

//Start 300x300mm. or 30x30 pattern
    else if (pos_->value.equals("3")){

    for ( int x = 0; x < config03.numPos; x++) {

      int _value = 0;
      //      Serial.println("x: " + config03.pos[x][0]);
      stepperX.moveTo(config03.pos[x][0]);
      stepperX.runToPosition();
      stepperX.setCurrentPosition(0);


      //      Serial.println("y: " + config03.pos[x][1]);
      stepperY.moveTo(config03.pos[x][1]);
      stepperY.runToPosition();
      stepperY.setCurrentPosition(0);
      //      Serial.println("");

  //Start moving to first step
      int depthPress = abs(config03.pos[x][2]);

      //      Serial.println("depthPress: " + depthPress);

  //Start press material
      if (depthPress == 4){
        stepperZ.moveTo(depth);
        stepperZ.runToPosition();
        stepperZ.setCurrentPosition(0);

  //Start record data for 4 seconds      
        for (int z = 0; z < interval; z++){
        
      // record.concat(getFormattedDateTime());
      record.concat(String(mm));
      record.concat(",");
      record.concat(loopCount);
      record.concat(",");
      record.concat(testCount);
      record.concat(",");
      record.concat(" ");
      

  // Read all ADS1115 channels
     readADS();

      // Read load cell
    loadCellReading = readLoadCell();
      readloadcell = loadCellReading * g * 0.001;
      record.concat(String(loadCellReading));
      record.concat(",");
      record.concat(String(readloadcell));
      record.concat("\n");
      
      // appendFile(SD, fileName.c_str(), record.c_str());
      Serial.println(record);
      dateTimeStr = "";
      record = "";
      //delayMicroseconds(700000);
      }      
  //End record data for 4 seconds
      record.concat("\n");

  //Stop press material    
      stepperZ.moveTo(abs(depth));
      stepperZ.runToPosition();
      stepperZ.setCurrentPosition(0);

      delayMicroseconds(1000000);

  //Start record data while load cell is hanging in the air
      // record.concat(getFormattedDateTime());
      record.concat(String(mm));
      record.concat(",");
      record.concat(loopCount);
      record.concat(",");
      record.concat(testCount);
      record.concat(",");
      record.concat(" ");
      testCount++;
      
// Read all ADS1115 channels
      readADS();

      // Read load cell
    loadCellReading = readLoadCell();
      readloadcell = loadCellReading * g * 0.001;
      record.concat(String(loadCellReading));
      record.concat(",");
      record.concat(String(readloadcell));
      record.concat("\n");
      
      // appendFile(SD, fileName.c_str(), record.c_str());
      Serial.println(record);
      dateTimeStr = "";
      record = "";
      
      scale.tare(); // reset the scale to 0
      record.concat("\n");

    }
//End 1 step process

//Checking for complete test steps   
    if (testCount > config03.numPos) {
      
      testCount = 1;
      moveToStart();

    }
    
      } 
    }

    // Increment the loop count and update the main label with the new count
    loopCount++;
    // Serial.println("loop3:" + String(loopCount));
    ESPUI.updateLabel(mainLabel, String(loopCount)); // Update the main label with the new loop count

    // Update the group label with the current file name
    ESPUI.updateLabel(grouplabel2, String(fileName)); // Update the group label with the current file name

  } else {
    // If we are not running, reset the loop count and flag
    isStopStart = false;
    loopCount = 1;
  }
  

}