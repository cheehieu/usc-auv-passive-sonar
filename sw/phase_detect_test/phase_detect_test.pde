/*
 phase_detect_test
 Reads an analog input on pin A0, prints the (running median) result to the serial monitor
 */
#include <RunningMedian.h>
#include <cppfix.h>

RunningMedian samples = RunningMedian();

void setup() {
  Serial.begin(9600);  //baud rate (9600, 14400, 19200, 28800, 38400, 57600, 115200)
  Serial.print("Running Median Size: ");
  Serial.println(MEDIAN_SIZE);
}

void loop() {
  int x = analogRead(A0);
  samples.add(x);
  long median_value = samples.getMedian();
  float median_voltage = 5 * float(median_value) / 1023;    //convert to volts
  Serial.print("median_voltage:\t");
  printFloats(median_voltage,3);	//Serial.print(median_voltage,3);
  delay(200);	//play with delay...
}



void printFloats(float value, int places) {
  // this is used to cast digits
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d/= 10.0;    
  // this small addition, combined with truncation will round our values properly
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }


  // write out the negative if needed
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);

  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  Serial.print('.');  

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0;
    digit = (int) tempfloat;
    Serial.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit;
  }
  Serial.println("");  //print carriage return, newline
}


