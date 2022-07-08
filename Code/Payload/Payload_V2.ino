/* Albedometer, Teensy 3.6 */
/* Handheld version,
   Dark being taken and Flip required
   Teensy avg 32, high gain, autogain on, 100 int time always
   Autogain Working
*/

//Libraries
#include <SoftwareSerial.h>
#include <TinyGPS.h>           /* For the GPS     */
#include <Adafruit_Sensor.h>   /* Basic Library for all Adafruit Sensors */
#include <Adafruit_BME280.h>   /* For Temp/Pressure/Humidity Sensor */
#include <Adafruit_MLX90614.h> /* For IR Sensor */
#include <Adafruit_VC0706.h>   /* For Camera */
#include <Adafruit_BNO055.h>   /* For Absolute Orient - Level Sensor */
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include <utility/imumaths.h>


/******* Sensor Set Up ********/
//Set Up GPS
TinyGPS gps;
#define HWSERIALgps Serial4               // Serial port used for the NMEA GPS 
//Set Up Camera
#define HW Serial3  // Serial port used for the camera   
Adafruit_VC0706 cam = Adafruit_VC0706(&HW);
//Set up SD Card
File myFile;                              // Create file object
#define chipSelect  BUILTIN_SDCARD        // Assign memory card pin
char filename[] = "00000000.txt";         // Format for filename
//Initialize Temp/Pressure/Humidity Sensor
Adafruit_BME280 bme;
//Set Up Level, Absolute Orient
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define BNO055_SAMPLERATE_DELAY_MS (100)  //Set sample rate for level
//IR Sensor Initialization
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
//Set up Spectrometers
#define SPEC_GAIN        2                // Pin 2 on Teensy, recognized as an int.
#define SPEC_ST          21               // Could also be pin A7 instead. 21
#define SPEC_CLK1        22               // Could also be pin A8 instead. 22
#define SPEC_VIDEO1      23               // Could also be pin A9 instead. 23
#define SPEC_CLK2        35               // Could also be pin A16 instead. 35
#define SPEC_VIDEO2      36               // Could also be pin A17 instead. 36
#define SPEC_CHANNELS    256              // Number of channels sampled by spectrometer

/********** Function Calls **********/
//Spectrometer Functions
void readWavelength(int SpecNum);
void subtractDark(int SpecNum, float data[SPEC_CHANNELS], float darkCurrent[SPEC_CHANNELS], int x);
void TransferFunction(float spec2DataDown[SPEC_CHANNELS], float spec1DataDown[SPEC_CHANNELS], float spec2DataUp[SPEC_CHANNELS], float spec1DataUp[SPEC_CHANNELS]);
void divideIntTime(int SpecNum, float data[SPEC_CHANNELS], uint16_t intTime, int x);
void spectraRatio(int sR, float data1[SPEC_CHANNELS], float data2[SPEC_CHANNELS]);
void calculateAlbedo(float data1[SPEC_CHANNELS], float data2[SPEC_CHANNELS]);
void readDark(int SpecNum);
void readSpectrometer(int SpecNum, int x);
void autoGain();
//Communication Functions
void sendByte();
//Instrument Functions
void getAmbientTempDown();
void getObjectTempDown();
void getAmbientTempUp();
void getObjectTempUp();
void getTemp();
void getHumidity();
void getPressure();
void getAltitude();
void getGPS();
void takePicture();
void gpsdump(TinyGPS &gps);
time_t getTeensy3Time();
unsigned long processSyncMessage();
//Data Functions
void saveData();
void saveRAWSpecData(float wavelength[SPEC_CHANNELS], float dark1[SPEC_CHANNELS], float dark2[SPEC_CHANNELS], float Spec1_1[SPEC_CHANNELS], float Spec2_2[SPEC_CHANNELS], float Spec1_2[SPEC_CHANNELS], float Spec2_1[SPEC_CHANNELS], uint16_t integrationTime1_1, uint16_t integrationTime1_2, uint16_t integrationTime2_1, uint16_t integrationTime2_2);

/********** Variable Declarations *********/
float pressure;         //hPa
float ambientTempUp;    //C*
float objectTempUp;     //C*
float ambientTempDown;  //C*
float objectTempDown;   //C*
float temp;             //C*
float alt;              //m
float humid;            //%

char tempArray[9] = {0};
char roll[9] = {0};
char pitch[9] = {0};
float rollSpec1;
float pitchSpec1;

// First level data
//Stores Instrument orientation for initial measurements
float RollBeforeSpec1UP;
float RollAfterSpec1UP;
float RollBeforeSpec2DOWN;
float RollAfterSpec2DOWN;
float PitchBeforeSpec1UP;
float PitchAfterSpec1UP;
float PitchBeforeSpec2DOWN;
float PitchAfterSpec2DOWN;

// Second level data (flipped over)
//Stores Instrument orientation for flipped measurements
float RollBeforeSpec1DOWN;
float RollAfterSpec1DOWN;
float RollBeforeSpec2UP;
float RollAfterSpec2UP;
float PitchBeforeSpec1DOWN;
float PitchAfterSpec1DOWN;
float PitchBeforeSpec2UP;
float PitchAfterSpec2UP;

float albedo[SPEC_CHANNELS];
float Hlam[SPEC_CHANNELS];

float AlbedoUncertainty[SPEC_CHANNELS]; // Final albedo uncertainty
float Spectra_Added[SPEC_CHANNELS]; // First part of albedo uncertainty calculation

// Arrays that hold the spectra from the spectrometer for all 256 channels
float spec1DataUp[SPEC_CHANNELS];
float spec1DataDown[SPEC_CHANNELS];

float spec2DataUp[SPEC_CHANNELS];
float spec2DataDown[SPEC_CHANNELS];

float spectra_Ratio1[SPEC_CHANNELS];
float spectra_Ratio2[SPEC_CHANNELS];

// Arrays that hold the wavelengths calulated from the characteristic polynomial equation
float wavelength1[SPEC_CHANNELS];
float wavelength2[SPEC_CHANNELS];

// Arrays that hold the dark spectra from the spectrometers
float darkData1[SPEC_CHANNELS];
float darkData2[SPEC_CHANNELS];

float kk1 = 0;
float kk2 = 0;

uint16_t intTimeSpec1 = 100;
uint16_t intTimeSpec2 = 100;
uint16_t intTimeDark1 = 100;
uint16_t intTimeDark2 = 100;
uint16_t integrationTime1_1 = 100; // Spec 1, UP
uint16_t integrationTime1_2 = 100; // Spec 1, DOWN
uint16_t integrationTime2_1 = 100; // Spec 2, UP
uint16_t integrationTime2_2 = 100; // Spec 2, DOWN

uint16_t Dark1 = 950;  // For Spectrometer C12666MA
uint16_t Dark2 = 950;  // For Spectrometer C12666MA

int darkCurrentState = 0;
char darkCurrentCom[5] = {0};

// Unused Variables
int dataState = 0;
char dataCom [5] = {0};

int buttonState = 0;
char buttonCom[5] = {0};

// Bools
bool AutoGain = true;
boolean newData = false;
boolean isLevel = false;

//Sets Sea Level Pressure
#define SEALEVELPRESSURE_HPA (1013.25)

/******************GPS Variables*********************/
float gpslat = -99.99          ; /* decimal degrees */
float gpslon = -99.99          ; /* decimal degrees */
float gpsalt = -99.99          ; /* meters */
float gpsspeed = -99.99        ; /* kilometers / hour */
float gpscourse = -99.99       ; /* degrees */
int   gpsyear = -99            ; /* e.g. 2016 */
int   gpsmon = 99              ; /* 1 thru 12 UTC */
int   gpsday = 99              ; /* 1 thru 31 UTC */
int   gpshour = 99             ; /* 0 thru 59 UTC */
int   gpsmin = 99              ; /* 0 thru 59 UTC */
int   gpssec = 99              ; /* 0 thru 59 UTC */

void setup() {
  HW.begin(9600);           // Serial to camera
  Serial.begin(9600);       // Open Serial to Teensy
  Serial5.begin(9600);      // Serial to Radio
  HWSERIALgps.begin(9600);  // GPS

  bme.begin();              //Initialize Sensors
  bno.begin();
  mlx.begin();
  setSyncProvider(getTeensy3Time);

  /*****Set up the Teensy analog to digital conversion******/
  analogReadResolution(13); // Do 13 bit analog read resolution on the Teensy.
  analogReadAveraging(32);  // Do 32 measurements and average them for every analog input measurement.

  //pinMode(SPEC_EOS, INPUT);
  pinMode(SPEC_GAIN, OUTPUT);       //Sets pin modes for Spectometes 1 & 2
  pinMode(SPEC_ST, OUTPUT);
  pinMode(SPEC_CLK1, OUTPUT);
  pinMode(SPEC_CLK2, OUTPUT);
  //pinMode(SPEC_CLK3, OUTPUT);

  digitalWrite(SPEC_ST, HIGH);
  digitalWrite(SPEC_CLK1, HIGH);
  digitalWrite(SPEC_CLK2, HIGH);
  //digitalWrite(SPEC_CLK3, HIGH);
  digitalWrite(SPEC_GAIN, LOW); //set low for HIGH Gain set high for LOW Gain

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }

  //Sets Image Size
  //cam.setImageSize(VC0706_640x480);      // biggest
  cam.setImageSize(VC0706_320x240);        // medium
  //cam.setImageSize(VC0706_160x120);      // small
  //uint8_t imgsize = cam.getImageSize();

  autoGain();
} // End of Set up

void loop() {

  //Set Real Time Clock on the Teensy
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }  // End of Time processSync

  int i = 0;
  //Reads in a button push over radio as an array and outputs as an int
  if (Serial5.available() > 0 && newData == false) {
    delay(100); //allows all serial sent to be received together
    while (Serial5.available() && i < 5) {
      Serial.println("Data is writing");
      buttonCom[i++] = Serial5.read();
    }
    buttonCom[i++] = '\0';
    newData = true;
    Serial.println(buttonCom);
  }
  if (newData == true) {
    buttonState = atoi(buttonCom);
    Serial.println(buttonState);
    buttonState = HIGH;
    isLevel = false;
  } // End of initial button push on 3.2

  while (buttonState == HIGH) // Giant while button is pushed, begin taking data
  {

    //Wavelength calibration
    readWavelength(1);
    readWavelength(2);

    float rollDark = 00.000;
    while (rollDark <= 80)   // Take dark until roll = 80 satisfied.
    {
      sensors_event_t darkMeasure;
      bno.getEvent(&darkMeasure);
      //Take Roll
      rollDark = (float)darkMeasure.orientation.y;
      //Parse Roll
      sprintf(roll, "r%02.02f&", rollDark);
      //Write Roll
      Serial5.write(roll);
      Serial.println(roll);
      delay(100);
    }

    readDark(1); // Spec 1 Dark
    readDark(2); // Spec 2 Dark

    darkCurrentState = 1;   //Confirmation that dark was taken
    itoa(darkCurrentState, darkCurrentCom, 10);

    // Send "Dark taken..."
    char StatusDark[20] = "z45&";
    Serial5.write(StatusDark);
    delay(5000);

    //autoGain();
    // Send Status: leveling
    char StatusLeveling[20] = "z50&";
    Serial5.write(StatusLeveling);

    do {
      sensors_event_t spec1;
      bno.getEvent(&spec1);

      //Take Roll
      rollSpec1 =  (float)spec1.orientation.y;
      sprintf(roll, "r%02.02f&", rollSpec1);
      //Serial5.write(roll);
      Serial.println(roll);
      Serial.flush();
      delay(100);

      //Take Pitch
      pitchSpec1 = (float)spec1.orientation.z;
      sprintf(pitch, "p%02.02f&", pitchSpec1);
      //Serial5.write(pitch);
      Serial.println (pitch);
      Serial.flush();
      delay(100);

      if (rollSpec1 <= 2 && rollSpec1 >= -2) {
        if (pitchSpec1 <= 2 && pitchSpec1 >= -2) isLevel = true;
      }
    } while (isLevel == false);
    isLevel = false;

    // Send Status Spec 1 UP
    char StatusSpec11[10] = "z111&";
    Serial5.write(StatusSpec11);

    // Get and Store Level
    sensors_event_t spec2;
    bno.getEvent(&spec2);
    RollBeforeSpec1UP =  (float)spec2.orientation.y;
    PitchBeforeSpec1UP = (float)spec2.orientation.z;

    readSpectrometer(1, 1);  // Spectrometer 1, UP

    // get and store level
    sensors_event_t spec3;
    bno.getEvent(&spec3);
    RollAfterSpec1UP =  (float)spec3.orientation.y;
    PitchAfterSpec1UP = (float)spec3.orientation.z;

    // Send Status Spec 2 DOWN
    char StatusSpec22[20] = "z221&";
    Serial5.write(StatusSpec22);

    // get and store level
    sensors_event_t spec4;
    bno.getEvent(&spec4);
    RollBeforeSpec2DOWN =  (float)spec4.orientation.y;
    PitchBeforeSpec2DOWN = (float)spec4.orientation.z;

    readSpectrometer(2, 2);   // Spectrometer 2, DOWN

    // get and store level
    sensors_event_t spec5;
    bno.getEvent(&spec5);
    RollAfterSpec2DOWN =  (float)spec5.orientation.y;
    PitchAfterSpec2DOWN = (float)spec5.orientation.z;

    // Get everything else
    getAmbientTempDown();
    getObjectTempDown();
    getTemp();
    getHumidity();
    getPressure();
    getAltitude();
    getGPS();
    // Send Status: Saving Photo
    char StatusPhoto[20] = "z60&";
    Serial5.write(StatusPhoto);
    takePicture();
    delay(100);


    /******** FLIP INSTRUMENT ********/
    // Send Status: Flip Instrument
    char StatusFlip[20] = "z70&";
    Serial5.write(StatusFlip);
    delay(2500);

    // Send Status: Leveling
    //char StatusLeveling[20] = "z50&";
    Serial5.write(StatusLeveling);
    do
    {
      sensors_event_t spec1;
      bno.getEvent(&spec1);

      //Take Roll
      rollSpec1 =  (float)spec1.orientation.y;
      sprintf(roll, "r%02.02f&", rollSpec1);
      //Serial5.write(roll);
      Serial.println(roll);
      Serial5.flush();
      delay(100);

      //Take Pitch
      pitchSpec1 = (float)spec1.orientation.z;
      sprintf(pitch, "p%02.02f&", pitchSpec1);
      //Write Pitch & Roll
      //Serial5.write(pitch);
      Serial.println (pitch);
      Serial5.flush();
      delay(100);

      if (rollSpec1 <= 2 && rollSpec1 >= -2)
      {
        if ((pitchSpec1 <= -178 && pitchSpec1 >= -180) || (pitchSpec1 >= 178 && pitchSpec1 <= 180)) isLevel = true;
      }
    } while (isLevel == false);

    // Send Status: Spec 1 DOWN
    char StatusSpec12[20] = "z121&";
    Serial5.write(StatusSpec12);

    // get and store level
    sensors_event_t spec6;
    bno.getEvent(&spec6);
    RollBeforeSpec1DOWN =  (float)spec6.orientation.y;
    PitchBeforeSpec1DOWN = (float)spec6.orientation.z;

    readSpectrometer(1, 2);  // Spectrometer 1, DOWN

    // get and store level
    sensors_event_t spec7;
    bno.getEvent(&spec7);
    RollAfterSpec1DOWN =  (float)spec7.orientation.y;
    PitchAfterSpec1DOWN = (float)spec7.orientation.z;

    // Send Status: Spec 2 UP
    char StatusSpec21[20] = "z211&";
    Serial5.write(StatusSpec21);

    // get and store level
    sensors_event_t spec8;
    bno.getEvent(&spec8);
    RollBeforeSpec2UP =  (float)spec8.orientation.y;
    PitchBeforeSpec2UP = (float)spec8.orientation.z;

    readSpectrometer(2, 1);   // Spectrometer 2, UP

    // get and store level
    sensors_event_t spec9;
    bno.getEvent(&spec9);
    RollAfterSpec2UP =  (float)spec9.orientation.y;
    PitchAfterSpec2UP = (float)spec9.orientation.z;

    delay(100);

    // Get everything else
    getAmbientTempUp();
    getObjectTempUp();
    // Send Status: Saving Photo
    Serial5.write(StatusPhoto);
    takePicture();
    delay(100);
    /**** End Measurements ****/

    // Send Status: Saving Raw
    char StatusRaw[20] = "z80&";
    Serial5.write(StatusRaw);

    /* Save all of the raw data: wavelength, darks, 4 raw spectras */
    saveRAWSpecData(wavelength1, darkData1, darkData2, spec1DataUp, spec2DataDown, spec1DataDown, spec2DataUp, integrationTime1_1, integrationTime1_2, integrationTime2_1, integrationTime2_2);

    /* Do all of the calculations */
    // clear screen and Send Status: Calculating Albedo
    char StatusAlbedo[20] = "z90&";
    Serial5.write(StatusAlbedo);

    /* Subtract the dark current */
    subtractDark(1, spec1DataUp, darkData1, 1);   // Spec #, data, dark, 1 for UP
    subtractDark(2, spec2DataDown, darkData2, 2); // Spec #, data, dark, 2 for DOWN
    subtractDark(1, spec1DataDown, darkData1, 2); // Spec #, data, dark, 2 for DOWN
    subtractDark(2, spec2DataUp, darkData2, 1);   // Spec #, data, dark, 1 for UP

    /* Uncertainty calculation PT.1 */
    for (i = 0; i < SPEC_CHANNELS; i++) {
      Spectra_Added[i] = (0.5) * sqrt(1.0 / abs(spec1DataUp[i]) + 1.0 / abs(spec2DataDown[i]) + 1.0 / abs(spec1DataDown[i]) + 1.0 / abs(spec2DataUp[i]));
    }

    /* Divide by the integration time */
    divideIntTime(1, spec1DataUp, integrationTime1_1, 1);   // Spec #, data, integration time, 1 for UP
    divideIntTime(2, spec2DataDown, integrationTime2_2, 2); // Spec #, data, integration time, 2 for DOWN
    divideIntTime(1, spec1DataDown, integrationTime1_2, 2); // Spec #, data, integration time, 2 for DOWN
    divideIntTime(2, spec2DataUp, integrationTime2_1, 1);   // Spec #, data, integration time, 1 for UP

    /* Transfer Function calculation H() */
    TransferFunction(spec2DataDown, spec1DataDown, spec2DataUp, spec1DataUp);

    /* Spectra ratio calculation */
    spectraRatio(2, spec2DataUp, spec1DataDown); // Spec ratio (1 or 2), Spec 2 UP minus dark, Spec 1 down minus dark
    spectraRatio(1, spec1DataUp, spec2DataDown); // Spec ratio (1 or 2), Spec 1 UP minus dark, Spec 2 down minus dark

    /* Albedo Calculation */
    calculateAlbedo(spectra_Ratio1, spectra_Ratio2);

    /* Uncertainty calculation PT.2 */
    for (i = 0; i < SPEC_CHANNELS; i++) {
      AlbedoUncertainty[i] = Spectra_Added[i] * albedo[i];
    }

    saveData();

    /** Format and send Albedo **/
    float PeakSpec = 0;
    float MinSpec = 1000000;
    char yPixel[84];
    char Pixel[9];
    float constrainedSpectra[SPEC_CHANNELS];
    float OrigPeakSpec = 0;
    char Peak[9] = {0};

    for (int j = 36; j < SPEC_CHANNELS; j++) {    // Finds peak value in original Albedo
      if (albedo[j] > OrigPeakSpec) {
        OrigPeakSpec = albedo[j];
      }
    }
    sprintf(Peak, "q%02.02f&", OrigPeakSpec);
    Serial5.write(Peak);

    for (int j = 12; j < 84; j++) {    // Finds peak value and minimum value in constrained spectra
      constrainedSpectra[j] = ((albedo[3 * j] + albedo[3 * j + 1] + albedo[3 * j + 2]) / 3); // Averages every 3 values in array
      if (constrainedSpectra[j] > PeakSpec) {
        PeakSpec = constrainedSpectra[j];
      }
      if (constrainedSpectra[j] < MinSpec) {
        MinSpec = constrainedSpectra[j];
      }
    }
    Serial.println(PeakSpec);
    Serial.println(MinSpec);

    float den = PeakSpec - MinSpec;
    for (int j = 0; j < 84; j++) {
      yPixel[j] = 40 - (40 * (PeakSpec - constrainedSpectra[j]) / den);
      sprintf(Pixel, "a%0d&", yPixel[j]);
      Serial5.write(Pixel);
      Serial.println(Pixel);
      delay(100);
    }

    delay(100);
    buttonState = LOW;
    newData = false;
    Serial.println(buttonState);
  } // End Big while button pressed

}  // End of Main Loop

//Populates wavelength arrays from characteristic polynomial equations
// **** NEED TO UPDATE CONSTANT VALUES ****
void readWavelength(int SpecNum) {
  if (SpecNum == 1) {
    float A_0 = 3.157284930e2;
    float B_1 = 2.382758890;
    float B_2 = -4.532653713e-4;
    float B_3 = -9.592362306e-6;
    float B_4 = 2.283177122e-8;
    float B_5 = -2.319360095e-11;
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      wavelength1[i] = A_0 + B_1 * i + B_2 * pow(i, 2) + B_3 * pow(i, 3) + B_4 * pow(i, 4) + B_5 * pow(i, 5);
    }
  }
  else if (SpecNum == 2) {
    float A_0 = 3.187695698e2;
    float B_1 = 2.384544110;
    float B_2 = -6.291373514e-4;
    float B_3 = -7.755157198e-6;
    float B_4 = 1.487105328e-8;
    float B_5 = -1.104337883e-11;
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      wavelength2[i] = A_0 + B_1 * i + B_2 * pow(i, 2) + B_3 * pow(i, 3) + B_4 * pow(i, 4) + B_5 * pow(i, 5);
    }
  }

}  // End of Read Wavelength

//Subtracts the dark spectra from the raw spectra and saves it in the original spectra array
//Loops thru Spec 1 Up, Spec 1 Down, Spec 2 Up, Spec 2 Down
void subtractDark(int SpecNum, float data[SPEC_CHANNELS], float darkCurrent[SPEC_CHANNELS], int x) {
  for (int i = 0; i < SPEC_CHANNELS; i++)
  {
    data[i] = data[i] - darkCurrent[i];
  }
  // Return data to the correct array
  if (SpecNum == 1 && x == 1) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec1DataUp[i] = data[i];
    }
  }
  if (SpecNum == 1 && x == 2) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec1DataDown[i] = data[i];
    }
  }
  if (SpecNum == 2 && x == 1) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec2DataUp[i] = data[i];
    }
  }
  if (SpecNum == 2 && x == 2) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec2DataDown[i] = data[i];
    }
  }
} // End of subtractDark


void TransferFunction(float spec2DataDown[SPEC_CHANNELS], float spec1DataDown[SPEC_CHANNELS], float spec2DataUp[SPEC_CHANNELS], float spec1DataUp[SPEC_CHANNELS]) {
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    Hlam[i] = sqrt(((float)spec2DataDown[i] / (float)spec1DataDown[i]) * ((float)spec2DataUp[i] / (float)spec1DataUp[i]));
  }
}

// Scales spectra data by the integration time
// Loops thru Spec 1 Up, Spec 1 Down, Spec 2 Up, Spec 2 Down
void divideIntTime(int SpecNum, float data[SPEC_CHANNELS], uint16_t intTime, int x) {

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    data[i] = (data[i] / (float)intTime);
  }

  // Return data to the correct array
  if (SpecNum == 1 && x == 1) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec1DataUp[i] = data[i];
    }
  }
  if (SpecNum == 1 && x == 2) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec1DataDown[i] = data[i];
    }
  }
  if (SpecNum == 2 && x == 1) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec2DataUp[i] = data[i];
    }
  }
  if (SpecNum == 2 && x == 2) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spec2DataDown[i] = data[i];
    }
  }
} // End of divide by intTime

// Populates Spectra_Ratio1 & Spectra_Ratio2 arrays with spec1down/spec2up & spec2down/spec1up
void spectraRatio(int sR, float data1[SPEC_CHANNELS], float data2[SPEC_CHANNELS]) {
  if (sR == 1) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spectra_Ratio1[i] = (data2[i] / data1[i]);
    }
  }
  if (sR == 2) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      spectra_Ratio2[i] = (data2[i] / data1[i]);
    }
  }
} // End of spectraRatio

// Calculates Albedo by taking the square root of Spectra_Ratio1 * Spectra_Ratio2
void calculateAlbedo(float data1[SPEC_CHANNELS], float data2[SPEC_CHANNELS]) {

  for (int i = 0; i < SPEC_CHANNELS; i++)
  {
    albedo[i] = sqrt(data1[i] * data2[i]);
  }
}

// Takes Spectrometer 1&2 readings while they are covered in order to account for noise in the system
// The dark spectras are returned in an array
void readDark(int SpecNum) {
  uint16_t intTimeNew;       // Initialize integration time variable, for Step 3 and auto gain.
  int delay_time = 1;        // delay per half clock (in microseconds).  This ultimately controls the integration time.
  int read_time = 35;        // Amount of time that the analogRead() procedure takes (in microseconds) (different micros will have different times)
  int accumulateMode = false;
  float data[SPEC_CHANNELS];
  int idx = 0;
  int SPEC_CLK = 0;
  int SPEC_VIDEO = 0;



  if (SpecNum == 1) { // Determine which spectrometer is in use to set variables to match:
    SPEC_CLK = SPEC_CLK1;
    SPEC_VIDEO = SPEC_VIDEO1;
    intTimeNew = intTimeDark1;
  }
  else if (SpecNum == 2) {
    SPEC_CLK = SPEC_CLK2;
    SPEC_VIDEO = SPEC_VIDEO2;
    intTimeNew = intTimeDark2;
  }

  // Step 1: start leading clock pulses
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }

  // Step 2: Send start pulse to signal start of integration/light collection
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, HIGH);
  delayMicroseconds(delay_time);

  // Step 3: Integration time -- sample for a period of time determined by the intTime parameter
  int blockTime = delay_time * 8;
  long int numIntegrationBlocks = ((long)intTimeNew * (long)1000) / (long)blockTime;
  for (int i = 0; i < numIntegrationBlocks; i++) {
    // Four clocks per pixel
    // First block of 2 clocks -- measurement
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK1, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);

    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }


  // Step 4: Send start pulse to signal end of integration/light collection
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, HIGH);
  delayMicroseconds(delay_time);

  // Step 5: Read Data 2 (this is the actual read, since the spectrometer has now sampled data)
  idx = 0;
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    // Four clocks per pixel
    // First block of 2 clocks -- measurement
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);

    // Analog value is valid on low transition
    if (accumulateMode == false) {
      //      unsigned long stTime = micros() ;
      data[idx] = analogRead(SPEC_VIDEO);
      //      unsigned long eTime = micros() ;
      //      unsigned long del=eTime-stTime;
      //      Serial.print("microsecs for read =");
      //      Serial.println(del)   ;

    } else {
      data[idx] += analogRead(SPEC_VIDEO);
    }
    idx += 1;
    if (delay_time > read_time) delayMicroseconds(delay_time - read_time);   // Read takes about 135uSec

    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);

    // Second block of 2 clocks -- idle
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }

  // Step 6: trailing clock pulses
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }
  // Return dark data to the correct array
  if (SpecNum == 1) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      darkData1[i] = data[i];
    }
  }
  if (SpecNum == 2) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      darkData2[i] = data[i];
    }
  }
} // End of readDark

// Generic Function that drives a Hamamatsu spectrometer
// Returns Spectra in an array
void readSpectrometer(int SpecNum, int x) // Spectrometer (1 or 2), UP (1) or DOWN (2)
{
  uint16_t intTimeNew;       // Initialize integration time variable, for Step 3 and auto gain.
  //uint16_t PeakSpec = 0;   // Initialize Spectral peak variable to determine integration time variable.
  uint16_t Dark = 0;         // Initialize dark variable, changes based on spectrometer being used.
  bool PeakCheck = false;    // Initialize the while loop to find spectra peak value.
  //int delay_time = 35;     // delay per half clock (in microseconds).  This ultimately controls the integration time.
  int delay_time = 1;        // delay per half clock (in microseconds).  This ultimately controls the integration time.
  int read_time = 35;        // Amount of time that the analogRead() procedure takes (in microseconds) (different micros will have different times)
  int accumulateMode = false;
  float data[SPEC_CHANNELS];
  int idx = 0;
  int k = 0;
  int SPEC_CLK = 0;
  int SPEC_VIDEO = 0;

  // Determine which spectrometer is in use to set variables to match:

  if (SpecNum == 1) {
    SPEC_CLK = SPEC_CLK1;
    SPEC_VIDEO = SPEC_VIDEO1;
    Dark = Dark1;  // Dark 1 is background and comes from global variables.
    intTimeNew = intTimeSpec1;
  }
  else if (SpecNum == 2) {
    SPEC_CLK = SPEC_CLK2;
    SPEC_VIDEO = SPEC_VIDEO2;
    Dark = Dark2; //  Dark 2 is background and comes from global variables.
    intTimeNew = intTimeSpec2;
  }

  while ((PeakCheck == false) && (k < 5)) {
    k++;

    // Step 1: start leading clock pulses
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
    }

    // Step 2: Send start pulse to signal start of integration/light collection
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    digitalWrite(SPEC_ST, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    digitalWrite(SPEC_ST, HIGH);
    delayMicroseconds(delay_time);

    // Step 3: Integration time -- sample for a period of time determined by the intTime parameter
    //Serial.print("IntTimeNew =");
    //Serial.println(intTimeNew);
    int blockTime = delay_time * 8;
    long int numIntegrationBlocks = ((long)intTimeNew * (long)1000) / (long)blockTime;
    for (int i = 0; i < numIntegrationBlocks; i++) {
      // Four clocks per pixel
      // First block of 2 clocks -- measurement
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK1, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);

      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
    }


    // Step 4: Send start pulse to signal end of integration/light collection
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    digitalWrite(SPEC_ST, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    digitalWrite(SPEC_ST, HIGH);
    delayMicroseconds(delay_time);

    // Step 5: Read Data 2 (this is the actual read, since the spectrometer has now sampled data)
    idx = 0;
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      // Four clocks per pixel
      // First block of 2 clocks -- measurement
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, LOW);

      // Analog value is valid on low transition
      if (accumulateMode == false) {
        //      unsigned long stTime = micros() ;
        data[idx] = analogRead(SPEC_VIDEO);
        //      unsigned long eTime = micros() ;
        //      unsigned long del=eTime-stTime;
        //      Serial.print("microsecs for read =");
        //      Serial.println(del)   ;

      } else {
        data[idx] += analogRead(SPEC_VIDEO);
      }
      idx += 1;
      if (delay_time > read_time) delayMicroseconds(delay_time - read_time);   // Read takes about 135uSec

      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);

      // Second block of 2 clocks -- idle
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
    }

    // Step 6: trailing clock pulses
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delay_time);
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delay_time);
    }

    // Return data to the correct array

    if (SpecNum == 1 && x == 1) {
      for (int i = 0; i < SPEC_CHANNELS; i++) {
        spec1DataUp[i] = data[i];
      }
    }
    if (SpecNum == 1 && x == 2) {
      for (int i = 0; i < SPEC_CHANNELS; i++) {
        spec1DataDown[i] = data[i];
      }
    }
    if (SpecNum == 2 && x == 2) {
      for (int i = 0; i < SPEC_CHANNELS; i++) {
        spec2DataDown[i] = data[i];
      }
    }
    if (SpecNum == 2 && x == 1) {
      for (int i = 0; i < SPEC_CHANNELS; i++) {
        spec2DataUp[i] = data[i];
      }
    }
  }

}   // End of readSpectrometer()

//Function that will return an integration time that will allow for maximum counts on low Gain
//Currently set up to test integration times require for Spec 1&2 in Up/Down configuration
void autoGain()
{
  uint16_t Dark = 950;
  float intTimeNew = 100;
  bool Spec1_int = false;
  bool Spec2_int = false;
  if (AutoGain == false) return; // When Autogain is not used, just return the measured spectra.

  float PeakSpec1_up = 0;
  float PeakSpec1_down = 0;
  float PeakSpec2_up = 0;
  float PeakSpec2_down = 0;
  
    readSpectrometer(1, 1);
    readSpectrometer(2, 1);
  
    // Finds the peak value in the spectra.
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      if (spec1DataUp[i] > PeakSpec1_up)
      {
        PeakSpec1_up = spec1DataUp[i];
      }
      if (spec1DataDown[i] > PeakSpec1_down)
      {
        PeakSpec1_down = spec1DataDown[i];
      }
      if (spec2DataUp[i] > PeakSpec2_up)
      {
        PeakSpec2_up = spec2DataUp[i];
      }
      if (spec2DataDown[i] > PeakSpec2_down)
      {
        PeakSpec2_down = spec2DataDown[i];
      }
    }
    while(Spec1_int == false){
      readSpectrometer(1, 1);
       for (int i = 0; i < SPEC_CHANNELS; i++) {
          if (spec1DataUp[i] > PeakSpec1_up)
          {
            PeakSpec1_up = spec1DataUp[i];
          }
       }
      if (PeakSpec1_up >= 3000 && PeakSpec1_up <= 3470) { // Changed for low gain, to be below saturation, and reduced range with hope for more consistent albedos
         intTimeSpec1 = 3400 * (intTimeSpec1)/ PeakSpec1_up;
         intTimeSpec2 = intTimeSpec1;
         Spec1_int = true;
      } else if (PeakSpec1_up < 3000){
          intTimeSpec1 = 3400 * (intTimeSpec1)/ PeakSpec1_up;
   
          Serial.print(PeakSpec1_up);
          Serial.print(",");
          Serial.println(intTimeSpec1);
      } 
    }
    Serial.println("Finding Spec2");
    while(Spec2_int == false){
      readSpectrometer(2, 1);
        for (int i = 0; i < SPEC_CHANNELS; i++) {
          if (spec2DataUp[i] > PeakSpec2_up)
          {
            PeakSpec2_up = spec2DataUp[i];
          }
       }
      if (PeakSpec2_up >= 3000 && PeakSpec2_up <= 3470) { // Changed for low gain, to be below saturation, and reduced range with hope for more consistent albedos
         intTimeSpec2 = 3400 * (intTimeSpec2)/ PeakSpec2_up;
         Spec2_int = true;
      } else if (PeakSpec2_up < 3000){
          intTimeSpec2 = 3400 * (intTimeSpec2)/ PeakSpec2_up;
          
          Serial.print(PeakSpec2_up);
          Serial.print(",");
          Serial.println(intTimeSpec2);
      }
    }

  Serial.print(" Peak Spec1_Up = ");
  Serial.println(PeakSpec1_up);
  Serial.print(" Peak Spec1_Down = ");
  Serial.println(PeakSpec1_down);
  Serial.print(" Peak Spec2_Up = ");
  Serial.println(PeakSpec2_up);
  Serial.print(" Peak Spec2_Down = ");
  Serial.println(PeakSpec2_down);
  Serial.print(" Integration Time = ");
  Serial.println(intTimeNew);
  Serial.print(" Integration Time 1 = ");
  Serial.println(intTimeSpec1);
  Serial.print(" Integration Time 2 = ");
  Serial.println(intTimeSpec2);
} //End Autogain

/******** Everything else subroutines ********/

// Writes a single byte over serial communication
// Used in radio communication between controller and payload
/****** Currently Unused! *****/
void sendByte()
{
  dataState = HIGH;
  itoa(dataState, dataCom, 10);
  Serial5.write(dataCom);
  Serial.println(dataCom);
  delay(100);
  dataState = LOW;
}

// Saves raw data to a continuous text file
// Saves dark spectra, raw spectra up and down as well as integration time
void saveRAWSpecData(float wavelength[SPEC_CHANNELS], float dark1[SPEC_CHANNELS], float dark2[SPEC_CHANNELS], float Spec1_1[SPEC_CHANNELS], float Spec2_2[SPEC_CHANNELS], float Spec1_2[SPEC_CHANNELS], float Spec2_1[SPEC_CHANNELS], uint16_t integrationTime1_1, uint16_t integrationTime1_2, uint16_t integrationTime2_1, uint16_t integrationTime2_2) {
  File myFile;
  myFile = SD.open("RawData.txt", FILE_WRITE);
  if (myFile)
  {
    myFile.println("\n");
    myFile.print("RTC Date & Time:");
    if (month() < 10) myFile.print('0');
    myFile.print(month());
    myFile.print("/");
    if (day() < 10) myFile.print('0');
    myFile.print(day());
    myFile.print("/");
    myFile.print(year());
    myFile.print("  ");
    if (hour() < 10) myFile.print('0');
    myFile.print(hour());
    myFile.print(":");
    if (minute() < 10) myFile.print('0');
    myFile.print(minute());
    myFile.print(":");
    if (second() < 10) myFile.print('0');
    myFile.println(second());
    myFile.print("Integration Time for Spec 1 UP: ");
    myFile.println(integrationTime1_1);
    myFile.print("Integration Time for Spec 1 DOWN: ");
    myFile.println(integrationTime1_2);
    myFile.print("Integration Time for Spec 2 UP: ");
    myFile.println(integrationTime2_1);
    myFile.print("Integration Time for Spec 2 DOWN: ");
    myFile.println(integrationTime2_2);
    myFile.println("Wavelength, Dark_Current1, Dark_Current2, Spectra1_Up, Spectra2_Down, Spectra1_Down, Spectra2_Up");
    for (int i = 0; i < SPEC_CHANNELS; i++)
    {
      myFile.print(wavelength[i]);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(dark1[i]);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(dark2[i]);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(Spec1_1[i], 5);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(Spec2_2[i], 5);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(Spec1_2[i], 5);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(Spec2_1[i], 5);
      myFile.println("\n");
    }
  }
  myFile.close();
}

// Gets downward ambient temperature from the IR sensor
void getAmbientTempDown()
{
  ambientTempDown = mlx.readAmbientTempC();
}

// Gets downward object temperature from the IR sensor
void getObjectTempDown()
{
  objectTempDown = mlx.readObjectTempC();
}

// Gets upward ambient temperature from the IR sensor
void getAmbientTempUp()
{
  ambientTempUp = mlx.readAmbientTempC();
}

// Gets upward object temperature from the IR sensor
void getObjectTempUp()
{
  objectTempUp = mlx.readObjectTempC();
}

// Gets temperature from the temperature/humidity/pressure sensor
void getTemp()
{
  bme.takeForcedMeasurement();
  temp = bme.readTemperature();
}

// Gets humidity from the temperature/humidity/pressure sensor
void getHumidity()
{
  bme.takeForcedMeasurement();
  humid = bme.readHumidity();
}

// Gets pressure from the temperature/humidity/pressure sensor
void getPressure()
{
  bme.takeForcedMeasurement();
  pressure = (bme.readPressure() / 100.0F);
}

// Gets altitude as a function of pressure from the temperature/humidity/pressure sensor
void getAltitude()
{
  bme.takeForcedMeasurement();
  alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

// Parses raw GPS data from NMEA format
void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);

  gps.f_get_position(&flat, &flon, &age);


  gps.get_datetime(&date, &time, &age);


  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);


  gpslat = flat                                 ;
  gpslon = flon                                 ;
  gpsalt   = gps.f_altitude()                   ;
  gpsspeed = gps.f_speed_kmph()                 ;
  gpscourse = gps.f_course()                    ;
  gpsyear   = year                              ;
  gpsmon    = static_cast<int>(month)           ;
  gpsday    = static_cast<int>(day)             ;
  gpshour   = static_cast<int>(hour)            ;
  gpsmin    = static_cast<int>(minute)          ;
  gpssec    = static_cast<int>(second)          ;


  gps.stats(&chars, &sentences, &failed);
}

// Checks for new GPS data and reads it in to preset variables
void getGPS()
{
  bool newdata = false             ;
  unsigned long startup = millis() ;
  while (millis() - startup < 1000L) {
    if (HWSERIALgps.available()) {
      char c = HWSERIALgps.read();
      if (gps.encode(c)) {
        newdata = true    ;
        break             ; /* Obtained data, bail out of the while loop */
      }
    }

  }
  if (newdata) gpsdump(gps);
}

void saveData()
{
  filename[0] = hour() / 10 + '0';
  filename[1] = hour() % 10 + '0';
  filename[2] = '-';
  filename[3] = minute() / 10 + '0';
  filename[4] = minute() % 10 + '0';
  filename[5] = '-';
  filename[6] = second() / 10 + '0';
  filename[7] = second() % 10 + '0';

  myFile = SD.open(filename, FILE_WRITE);
  if (myFile)
  {
    myFile.println("GPS_Date,GPS_Time, Latitude, Longitude, Altitude, Course_Degrees");
    myFile.print(gpsday);
    myFile.print("/");
    myFile.print(gpsmon);
    myFile.print("/");
    myFile.print(gpsyear);
    myFile.print(",");
    myFile.print(gpshour);
    myFile.print(":");
    myFile.print(gpsmin);
    myFile.print(":");
    myFile.print(gpssec);
    myFile.print(",");
    myFile.print(gpslat);
    myFile.print(",");
    myFile.print(gpslon);
    myFile.print(",");
    myFile.print(gpsalt);
    myFile.print(",");
    myFile.println(gpscourse);
    myFile.print("\0");

    myFile.print("RTC Date & Time:");
    if (month() < 10) myFile.print('0');
    myFile.print(month());
    myFile.print("/");
    if (day() < 10) myFile.print('0');
    myFile.print(day());
    myFile.print("/");
    myFile.print(year());
    myFile.print("  ");
    if (hour() < 10) myFile.print('0');
    myFile.print(hour());
    myFile.print(":");
    if (minute() < 10) myFile.print('0');
    myFile.print(minute());
    myFile.print(":");
    if (second() < 10) myFile.print('0');
    myFile.println(second());
    myFile.print("Temperature: ");
    myFile.println(temp);
    myFile.print("Pressure: ");
    myFile.println(pressure);
    myFile.print("Humidity: ");
    myFile.println(humid);
    myFile.print("Altitude: ");
    myFile.println(alt);
    myFile.println("IR Sensor Looking Down: ");
    myFile.print("Ambient Temp: ");
    myFile.println(ambientTempDown);
    myFile.print("Object Temp: ");
    myFile.println(objectTempDown);
    myFile.println("IR Sensor Looking Up: ");
    myFile.print("Ambient Temp: ");
    myFile.println(ambientTempUp);
    myFile.print("Object Temp: ");
    myFile.println(objectTempUp);
    myFile.println("Roll & Pitch (roll, pitch): ");
    myFile.print("Before Spec1 UP: ");
    myFile.print("(");
    myFile.print(RollBeforeSpec1UP);
    myFile.print(",");
    myFile.print(PitchBeforeSpec1UP);
    myFile.println(")");
    myFile.print("After Spec1 UP: ");
    myFile.print("(");
    myFile.print(RollAfterSpec1UP);
    myFile.print(",");
    myFile.print(PitchAfterSpec1UP);
    myFile.println(")");
    myFile.print("Before Spec2 DOWN: ");
    myFile.print("(");
    myFile.print(RollBeforeSpec2DOWN);
    myFile.print(",");
    myFile.print(PitchBeforeSpec2DOWN);
    myFile.println(")");
    myFile.print("After Spec2 DOWN: ");
    myFile.print("(");
    myFile.print(RollAfterSpec2DOWN);
    myFile.print(",");
    myFile.print(PitchAfterSpec2DOWN);
    myFile.println(")");
    myFile.print("Before Spec1 DOWN: ");
    myFile.print("(");
    myFile.print(RollBeforeSpec1DOWN);
    myFile.print(",");
    myFile.print(PitchBeforeSpec1DOWN);
    myFile.println(")");
    myFile.print("After Spec1 DOWN: ");
    myFile.print("(");
    myFile.print(RollAfterSpec1DOWN);
    myFile.print(",");
    myFile.print(PitchAfterSpec1DOWN);
    myFile.println(")");
    myFile.print("Before Spec2 UP: ");
    myFile.print("(");
    myFile.print(RollBeforeSpec2UP);
    myFile.print(",");
    myFile.print(PitchBeforeSpec2UP);
    myFile.println(")");
    myFile.print("After Spec2 UP: ");
    myFile.print("(");
    myFile.print(RollAfterSpec2UP);
    myFile.print(",");
    myFile.print(PitchAfterSpec2UP);
    myFile.println(")");
    myFile.print("K Spectrometer 1 (UP): ");
    myFile.println(kk1);
    myFile.print("K Spectrometer 2 (DOWN): ");
    myFile.println(kk2);
    myFile.println("");

    myFile.println("Wavelength, Transfer_Function, Surface_Albedo, Albedo_Uncertainty, Spectra1_Up, Spectra2_Down, Spectra1_Down, Spectra2_Up, Dark_Current1, Dark_Current2");
    for (int i = 0; i < SPEC_CHANNELS; i++)
    {
      myFile.print(wavelength1[i]);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(Hlam[i], 5);
      myFile.print(",");
      myFile.print("     ");
      myFile.print(albedo[i], 5);
      myFile.print(",");
      myFile.print("    ");
      myFile.print(AlbedoUncertainty[i], 10);
      myFile.print(",");
      myFile.print("    ");
      myFile.print(spec1DataUp[i], 5);
      myFile.print(",");
      myFile.print("    ");
      myFile.print(spec2DataDown[i], 5);
      myFile.print(",");
      myFile.print("    ");
      myFile.print(spec1DataDown[i], 5);
      myFile.print(",");
      myFile.print("    ");
      myFile.print(spec2DataUp[i], 5);
      myFile.print(",");
      myFile.print("    ");
      myFile.print(darkData1[i]);
      myFile.print(",");
      myFile.print("    ");
      myFile.print(darkData2[i]);
      myFile.println("");
    }
    myFile.println("\n");
  } else {
    Serial.println("Error!");
  }
  myFile.close();
}

void takePicture()
{
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  //delay(100);
  delay(3000);

  if (! cam.takePicture())
    Serial.println("Failed to snap!");
  else
    Serial.println("Picture taken!");

  char imgfile[] = "00000000.JPG";  // Camera
  for (int i = 0; i < 1; i++) {
    imgfile[0] = hour() / 10 + '0';
    imgfile[1] = hour() % 10 + '0';
    imgfile[2] = '_';
    imgfile[3] = minute() / 10 + '0';
    imgfile[4] = minute() % 10 + '0';
    imgfile[5] = '_';
    imgfile[6] = second() / 10 + '0';
    imgfile[7] = second() % 10 + '0';
    if (! SD.exists(imgfile)) {
      break;
    }
  }

  myFile = SD.open(imgfile, FILE_WRITE);
  uint16_t jpglen = cam.frameLength();

  while (jpglen > 0) {
    uint8_t *buffer;
    uint8_t bytesToRead = min(64, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    myFile.write(buffer, bytesToRead);
    jpglen -= bytesToRead;
  }
  myFile.close();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
}
