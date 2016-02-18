// Capstone Team M4

#include <I2C.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f

#define PIN_LED 13  // built in led
#define PIN_PIEZO 9
#define PIN_USOUND A0

#define LIDAR_THRESH 100
#define USOUND_THRESH 25

// Feedback Enables
#define ENABLE_LED 1
#define ENABLE_PIEZO 1

// Lidar

void lidar_init() {
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
}

int lidar_read() {
  // copied from Lidar-lite demo code
  
     //Serial.println("enter loop?");
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
    //Serial.println("np");
      //  Serial.println(nackack);
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
    //Serial.println("yp");

  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  
  // Print Distance
  //Serial.println("test?");
//  Serial.println(distance);
  return distance;
}


// Ultrasound

void usound_init() {
  // nothing atm
}

int usound_read() {
   // read the input on analog pin 0:
  int sensorValue = analogRead(PIN_USOUND);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  // convert that voltage to range according to the datasheet of mV -> inches
  float range = voltage / .0098;
  return (int) range;
}

// Feedback:LED

void led_init() {
  pinMode(PIN_LED, OUTPUT);
}
void led_on() {
  if (ENABLE_LED)
    digitalWrite(PIN_LED, HIGH);
}
void led_off() {
  if (ENABLE_LED)
    digitalWrite(PIN_LED, LOW);
}

// Feedback:Piezo
void piezo_init() {
  pinMode(PIN_PIEZO, OUTPUT);
}

void piezo_on() {
  // 440 is an 'A'
  if (ENABLE_PIEZO)
    tone(PIN_PIEZO, 440);
}
void piezo_off() {
  if (ENABLE_PIEZO)
    noTone(PIN_PIEZO);
}

// Feedback
 
void feedback_init() {
  led_init();
  piezo_init();
}

// TODO: currently this causes the feedback to be repeated
// with noticeable gaps, according to the delay used. It
// would be better to provide continuous feedback.
void feedback_trigger() {
  const int feedback_len = 250;
  led_on();
  piezo_on();
  
  delay(feedback_len);
  
  led_off();
  piezo_off();
}

// Main

void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.

  lidar_init();
  usound_init();
  feedback_init();
}

void loop() {

  int lidar_dist = lidar_read();
  int usound_dist = usound_read();
  if (lidar_dist < LIDAR_THRESH || usound_dist < USOUND_THRESH) {
    feedback_trigger();
  }
  Serial.println(lidar_dist);
  Serial.println(usound_dist);
  delay(10);

  
  
  
  
}
