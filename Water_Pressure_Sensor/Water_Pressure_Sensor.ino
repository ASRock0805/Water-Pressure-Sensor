/*
  Water Pressure Sensor (DFRobot SEN0257)
      The output voltage of the sensor is converted by ADC to obtain the water pressure.

  Created 28 Sept. 2020
  by Yi-Xuan Wang

  References:
  https://wiki.dfrobot.com/Gravity__Water_Pressure_Sensor_SKU__SEN0257
*/

/*--- Preprocessor ---*/
#define sigPin A0 // Voltage meter signal pin w/ ADC

#define N 800     // Measurment sampling number for smoothing

/*--- Constants ---*/
const unsigned long baudSpeed = 115200UL; // Sets the data rate in bits per second (baud) for serial data transmission
const unsigned long period = 1000UL;      // The value is a number of milliseconds

const byte vIn = 5;                                             // Supply voltage from Arduino
const byte resBits = 10;                                        // Resolution of ADC (10 bits)
const float vConv = (float)(vIn / (pow(2.0, resBits) - 1.0f));  // Voltage of ADC level (2^bits)

// Spec. of water pressure sensor, Range: 0 - 16 MPa, Output: 0.5 - 4.5 V
const float pgMax = 16.0f;             // Upper limit of pressure sensor
const float pgMin = 0.0f;              // Lower limit of pressure sensor
const float pgVmax = 4.5f;             // Maximum output voltage of pressure sensor
const float pgVmin = 0.5f;             // Minimum output voltage of pressure sensor
const float offSet = 0.471772766113f;  // Norminal value is 0.5 V

/*--- Global Variables ---*/
unsigned long startTime;    // Start time
unsigned long currentTime;  // Current time

float vOut;                 // Output of the ADC
float waterPres;            // Value of water pressure

/*--- Function Prototype ---*/
float getwaterPres(float );
void waterPressure(byte );
void setup(void);
void loop(void);

/*--- Functions Definition ---*/
// Implementation of Water Pressure Calculation
float getwaterPres(float volt) {
  return ((volt - offSet) * ((pgMax - pgMin) / (pgVmax - pgVmin))) + pgMin;
}

// Water Pressure Sensor
void waterPressure(byte signalPin) {
  for (unsigned int i = 0U; i < N; ++i) {      // Get samples for smooth the value
    vOut = vOut + analogRead(signalPin);
    delay(1UL);                                 // delay in between reads for stability
  }
  vOut = (vOut * vConv) / N;                    // ADC of voltage meter output voltage

  waterPres = getwaterPres(vOut);              // Calculate water pressure

  if (isinf(waterPres) || isnan(waterPres)) {
    waterPres = -1;
  }
}

/*--- Initialization ---*/
void setup(void) {
  Serial.begin(baudSpeed);  // Initializes serial port
  pinMode(sigPin, INPUT);   // Initializes potentiometer pin
  startTime = millis();     // Initial start time
  
  // Water Pressure Sensor Initialization
  vOut = 0.0f;
  waterPres = 0.0f;
}

/*--- Measurement ---*/
void loop(void) {
  // Every second, calculate and print the measured value
  currentTime = millis();                     // Get the current "time"

  if ((currentTime - startTime) >= period) {  // Test whether the period has elapsed
    // Water Pressure Sensor
    waterPressure(sigPin);

    /*--- Sensor prompt ---*/
    Serial.print("Voltage: ");
    Serial.print(" V, ");
    Serial.print(vOut, 12);
    // Unit converter for pressure, raw unit: MPa
    Serial.print("Pressure: ");
    Serial.print(waterPres * 1000.0f, 1);    // 1 : 1000
    Serial.print(" kPa, ");
    Serial.print(waterPres, 2);              // Raw
    Serial.print(" MPa, ");
    Serial.print(waterPres * 10.1972f, 1);   // 1 : 10.1972
    Serial.print(" kg/cm^2, ");

    /*--- System Return ---*/
    startTime = currentTime;  // Save the start time of the current state
  } else {
    return;
  }
}
