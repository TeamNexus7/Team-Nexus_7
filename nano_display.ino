#include <Servo.h>
// Samples per second
#define SAMPLE_RATE 500

// Make sure to set the same baud rate on your Serial Monitor/Plotter
#define BAUD_RATE 115200

// Change if not using A0 analog pin
#define INPUT_PIN A0

// envelopee buffer size
// High value -> smooth but less responsive
// Low value -> not smooth but responsive
#define BUFFER_SIZE 100

// Servo pin (Change as per your connection)
#define SERVO_PIN 2 // Pin2 for Muscle BioAmp Shield v0.3
#define VIB_PIN 11
#define POWER_PIN 5

// EMG Threshold value, different for each user
// Check by plotting EMG envelopee data on Serial plotter
#define EMG_THRESHOLD 30 //servo activation value

// Servo open & close angles
#define SERVO_START 60 //forward
#define SERVO_MID 90  //center
#define SERVO_END 120 //backword

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);


const int voltageSensorPin = A3;          // sensor pin
float vIn;                                // measured voltage (3.3V = max. 16.5V, 5V = max 25V)
float vOut;
float voltageSensorVal;                   // value on pin A3 (0 - 1023)
const float factor = 5.128;               // reduction factor of the Voltage Sensor shield
const float vCC = 5.00;

int circular_buffer[BUFFER_SIZE];
int data_index, sum;
int flag=0;
Servo servo;
// Servo angle
int angle = 90;
// Last gesture timestamp
unsigned long lastGestureTime = 40;
// Delay between two actions
unsigned long gestureDelay = 90;

void setup() {
  // Serial connection begin
  Serial.begin(BAUD_RATE);

   pinMode(POWER_PIN, INPUT);
  // Attach servo
  servo.attach(SERVO_PIN);
  servo.write(angle);

display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.print("NEXUS_6");
  display.display();
  delay(5000); 
   display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);

  // Display battery voltage
  display.setCursor(0, 0);
  display.print("Battery: ");
  display.println(vIn);
}

void loop() {  
  // Calculate elapsed time
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

// Display battery voltage
  voltageSensorVal = analogRead(voltageSensorPin);    // read the current sensor value (0 - 1023) 
  vOut = (voltageSensorVal / 1024.0) * vCC;           // convert the value to the real voltage on the analog pin
  vIn =  vOut * factor;

  // Sample and get envelope
  if(timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    
    // RAW EMG Values
    int sensor_value = analogRead(INPUT_PIN);
    
    // Filtered EMG
    int signal = EMGFilter(sensor_value);
    
    // EMG envelopee
    int envelope = getEnvelope(abs(signal));
    
    // Move servo
    if(envelope > EMG_THRESHOLD) 
    {
      servo.write(SERVO_MID);
    }
    else if (digitalRead(POWER_PIN) == HIGH)
    {
      servo.write(SERVO_MID);
    }
    else
    {
      servo.write(SERVO_MID);
    }

    // EMG Raw signal
    Serial.print(signal);
    // Data seprator
    Serial.print(",");
    // EMG envelopee
    Serial.println(envelope);
  }
  if (vIn < 8.5) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Healthy Battery!");
     display.display();
    

  }
}

// envelope detection algorithm
int getEnvelope(int abs_emg){
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum/BUFFER_SIZE) * 2;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
