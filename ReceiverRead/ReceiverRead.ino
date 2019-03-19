int channel[] = {0, 9, 2, 3, 4, 5, 6, 7, 8};  // The input pins for each channel i.e. Ch1 is 9
int filtDur[9];                               // Array for filtered pulselength the same length as channel array
int dur[9];                                   // Array for pulselength the same length as channel array
int timer = 5;                                // delay in ms
float weight = 0.2;                           // weight for low pass filter

void setup()
{
  Serial.begin(9600);                         // begin serial communication
  for (int i = 0; i < 9; i++){                // set up each channel pin as an input
    pinMode(channel[i], INPUT);
  }
}

void loop()
{
  for (int x = 1; x < 9; x++){
    dur[x] = pulseIn(channel[x], HIGH);       // set duration array value to the pulse duration
    filtDur[x] = (filtDur[x] * weight) + (dur[x] * (1 - weight));   // apply a simple low pass filter to duration and assign to array
    Serial.print(filtDur[x]);                 // print pulse duration to serial                                         
//    Serial.print(filtDur[x] + ((x - 1) * 1000));    // use this line instead of previous to add offset to each channel for plotting
    Serial.print(", ");
  }
  Serial.println();
  delay(timer);
}
