int channel[] = {0, 9, 2, 3, 4, 5, 6, 7, 8};
int filtDur[9];
int dur[9];
int timer = 5;        //delay in ms
float weight = 0.2;

void setup()
{
  Serial.begin(9600);
  for (int i = 0; i < 9; i++){
    pinMode(channel[i], INPUT);
  }
}

void loop()
{
  for (int x = 1; x < 9; x++){
    dur[x] = pulseIn(channel[x], HIGH);
    filtDur[x] = (filtDur[x] * weight) + (dur[x] * (1 - weight));
    Serial.print(filtDur[x]);
//    Serial.print(filtDur[x] + ((x - 1) * 1000));
    Serial.print(", ");
  }
  Serial.println();
  delay(timer);
}
