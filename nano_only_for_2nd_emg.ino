// Samples per second
#define SAMPLE_RATE 500

// Make sure to set the same baud rate on your Serial Monitor/Plotter
#define BAUD_RATE 115200

// Change if not using A0 analog pin
#define INPUT_PIN A0

// envelopee buffer size
// High value -> smooth but less responsive
// Low value -> not smooth but responsive
#define BUFFER_SIZE 150 // smooth graph

#define POWER_PIN 5 //
// EMG Threshold value, different for each user
// Check by plotting EMG envelopee data on Serial plotter
#define EMG_THRESHOLD 30 //servo activate

int circular_buffer[BUFFER_SIZE];
int data_index, sum;
int flag=0;

void setup() {
  // Serial connection begin
  Serial.begin(BAUD_RATE);

 //power
  pinMode(POWER_PIN, OUTPUT);  // Set POWER_PIN as an output
  digitalWrite(POWER_PIN, LOW);  // Initially, turn off the POWER
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
    if(envelope > EMG_THRESHOLD) //changes
    {
      digitalWrite(POWER_PIN, LOW);
    }
    else
    {
      digitalWrite(POWER_PIN, LOW);
    }

    // EMG Raw signal
    Serial.print(signal);
    // Data seprator
    Serial.print(",");
    // EMG envelopee
    Serial.println(envelope);
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
