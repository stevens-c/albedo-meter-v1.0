#include <Adafruit_GFX.h>      /* For the display */
#include <Adafruit_PCD8544.h>  /* For the display */
#include <SPI.h>  /* Serial Communication */
#include <Wire.h> /* I2C Communication */

#define SPEC_CHANNELS 256

Adafruit_PCD8544 display = Adafruit_PCD8544(8, 6, 4, 3, 2); // For the LCD display

void recAlbedo();
void recvWithStartEndMarkers();
bool printAlbedo = false;

const int buttonPin = 10;
int buttonState = 1;
char buttonCom[5] = {0};
char darkCurrentCom[5] = {0};
int darkCurrentState = 0;

//Temp Arrays for receiving data
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
//char ab[84];
char measType = '0';

//Initilization of Parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
char dataCom[5] = {0};
int dataState = 0;
uint16_t xPixel[84];
uint16_t yPixel[84];
float wavelength[SPEC_CHANNELS];

//float rollDark = 0.0;
float rollSpec1;
float pitchSpec1;
float albedo1;
float albedo2[72];
String Status;
float OrigPeak;

boolean newData = false;
boolean newByte = false;
boolean isLevel = false;
boolean GotPeak = false;

void setup() {
  pinMode(buttonPin, INPUT);

  display.begin();
  display.setContrast(60);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  display.clearDisplay();  //For the display
  display.setCursor(0,0);  //For the display
  display.println("Press the\nbutton \nto begin \nmeasurements");
  display.display();
  buttonState = digitalRead(buttonPin);
  while (buttonState == LOW)
  {
    //buttonState = HIGH;
    itoa(buttonState, buttonCom, 10);
    buttonState = LOW;
    Serial.println(buttonCom);
    Serial1.write(buttonCom);
    delay(100);

     float rollDark = 0.0;
    //  Read in dark
    while (rollDark <= 80)
    {
      //Read in Roll
      measType = 'r';
      recvWithStartEndMarkers();
      //Display Roll
      if (newData == true)
      {
        strcpy(tempChars, receivedChars);
        rollDark = atof(tempChars);
        display.clearDisplay();  //For the display
        display.setCursor(0,0);  //For the display
        display.println("Get Roll to 80 to Begin");
        display.print("Roll:");
        display.println(rollDark);
        display.display();
        newData = false;
      }
    }    // end of dark message



    //Read in everything
      
      while(isLevel == false){
        measType = 'z';
        recvWithStartEndMarkers();
       if (newData == true){   
        strcpy(tempChars, receivedChars);
        float NumberCode = atof(tempChars);

              if (NumberCode == 45){
                display.clearDisplay();  //For the display
                display.setTextSize(1);
                display.setCursor(12,20);  //For the display
                display.println("Dark Taken");
                display.display();
                newData = false;
                } 
              if (NumberCode == 50){
                display.clearDisplay();  //For the display
                display.setTextSize(1);
                display.setCursor(25,15);  //For the display
                display.println("Begin\n  Leveling...");
                display.display();
                newData = false;
                } 
              if (NumberCode == 60){
                display.clearDisplay();  //For the display
                display.setTextSize(1);
                display.setCursor(25,15);  //For the display
                display.println("Saving\n   Photo...");
                display.display();
                newData = false;
                } 
              if (NumberCode == 70){
                display.clearDisplay();  //For the display
                display.setTextSize(1);
                display.setCursor(8,20);  //For the display
                display.println("Flip over...");
                display.display();
                newData = false;
                } 
              if (NumberCode == 80){
                display.clearDisplay();  //For the display
                display.setTextSize(1);
                display.setCursor(12,15);  //For the display
                display.println("Saving Raw\n    Data...");
                display.display();
                newData = false;
                } 
              if (NumberCode == 90){
                display.clearDisplay();  //For the display
                display.setTextSize(1);
                display.setCursor(12,15);  //For the display
                display.println("Calculating\n    Albedo...");
                display.display(); 
                newData = false;
                break;
                } 
              if (NumberCode == 111){
                display.clearDisplay();  //For the display
                display.setCursor(0,0);  //For the display
                display.setTextSize(1);
                display.println("Taking\nSpectrometer\nmeasurement");
                display.setTextSize(2);
                display.setCursor(20,29);  //For the display
                display.print("1 UP");
                display.display();
                newData = false;
                } 
              if (NumberCode == 221){
                display.clearDisplay();  //For the display
                display.setCursor(0,0);  //For the display
                display.setTextSize(1);
                display.println("Taking\nSpectrometer\nmeasurement");
                display.setTextSize(2);
                display.setCursor(8,29);  //For the display
                display.print("2 DOWN");
                display.display();
                newData = false;
                } 
              if (NumberCode == 121){
                display.clearDisplay();  //For the display
                display.setCursor(0,0);  //For the display
                display.setTextSize(1);
                display.println("Taking\nSpectrometer\nmeasurement");
                display.setTextSize(2);
                display.setCursor(8,29);  //For the display
                display.print("1 DOWN");
                display.display();
                newData = false;
                } 
              if (NumberCode == 211){
                display.clearDisplay();  //For the display
                display.setCursor(0,0);  //For the display
                display.setTextSize(1);
                display.println("Taking\nSpectrometer\nmeasurement");
                display.setTextSize(2);
                display.setCursor(20,29);  //For the display
                display.print("2 UP");
                display.display();
                newData = false;
                }  
            }
       }
     while(GotPeak == false){
         measType = 'q';
         recvWithStartEndMarkers();
         if (newData == true){   
             strcpy(tempChars, receivedChars);
             OrigPeak = atof(tempChars);
             GotPeak = true;
         }
     }

     int z = 0; 
     float peak = 0;
     while(z<72)
     { 
      measType = 'a';
      recvWithStartEndMarkers(); 
      if (newData == true)
      {
        //recvWithStartEndMarkers();
        strcpy(tempChars, receivedChars);
        albedo1 = atof(tempChars);
        albedo2[z] = albedo1;
        z++;
        //memset(tempChars, 0, sizeof(tempChars));
        //memset(receivedChars, 0, sizeof(receivedChars));
        delay(10);
        newData = false;
      }
     }
        //Display albedo
        display.clearDisplay();  //For the display
        display.setCursor(0,41);  //For the display
        display.setTextSize(1);
        display.setTextColor(BLACK);
        display.print("Peak=");
        display.println(OrigPeak);     // Displays Peak value on screen
      for (int i = 0; i < 72; i++){
        Serial.println(albedo2[i]);
        display.drawPixel(i, albedo2[i], BLACK);  // draw a single pixel
      }
         display.display(); 
         delay(5000); // Hold Albedo plot on display for 5 Seconds
     
    buttonState = HIGH; 
    
 } // End button state
} // End Loop

//Function that recieves data with starting marker * and end marker &
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = measType;
  char endMarker = '&';
  char rc;

  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void recByte()
{
  int i = 0;
  if(Serial1.available() >0 && newByte == false)
  {
    while (Serial1.available() && i<5)
    {
      dataCom[i++] = Serial1.read();
    }
    dataCom[i++] = '\0';
    newByte = true;
  }
  if (newByte == true)
  {
    dataState = atoi(dataCom);
  }
}

