#include "WiFi.h"
#include "AsyncUDP.h"
#include <ESP32Servo.h>

#define SERVO1 9
#define SERVO2 10
#define THRUST1 A2
#define THRUST2 A3
 
const char * ssid = "*********";
const char * password = "*********";
Servo servo1;
Servo servo2; 

AsyncUDP udp;
float joy_data[4] = {0.0, 0.0, 0.0, 0.0};
volatile bool joy_ready = false;
volatile unsigned long time_now; 

void unpack_joystick(float *dat, const unsigned char *buffer) {
  int num_floats = 4;
  int num_bytes = 4;
  int i, j;

  for(i = 0; i < num_floats; i++) {
    char temp[4] = {0, 0, 0, 0};
    for(j = 0; j < num_bytes; j++) {
      temp[j] = buffer[4*i + j];
    }
    dat[i] = *((float*) temp);
    // if(i == 1 || i == 3){
    //   dat[i] = -*((float*) temp);
    // } else {
    //   dat[i] = *((float*) temp);
    // }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while(1) {
      delay(1000);
    }
  }

  // set the motor out pins as outputs
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);// Standard 50hz servo
  servo2.setPeriodHertz(50);// Standard 50hz servo
  servo1.attach(SERVO1, 450, 2550);
  servo2.attach(SERVO2, 450, 2550);


  pinMode(THRUST1, OUTPUT);
  pinMode(THRUST2, OUTPUT);

  if(udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());

    // setup callback functions of the udp
    udp.onPacket([](AsyncUDPPacket packet) {
      joy_ready = false;
      time_now = millis();
      unsigned char *buffer = packet.data();
      unpack_joystick(joy_data, buffer);
      joy_ready = true;
      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());
    });
  }
}

void loop() {
  if(joy_ready && millis() - time_now <= 1000){
    servo1.write((int) joy_data[2]);
    servo2.write((int) joy_data[3]);
    analogWrite(THRUST1, (int) floor(joy_data[0]));
    analogWrite(THRUST2, (int) floor(joy_data[1]));

    Serial.print((int) joy_data[2]);
    Serial.print(' ');
    Serial.print((int) joy_data[3]);
    Serial.print(' ');
    Serial.print((int) joy_data[0]);
    Serial.print(' ');
    Serial.print((int) joy_data[1]);
    Serial.println(' ');
  }
}
