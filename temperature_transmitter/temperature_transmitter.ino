/*
 Send and receive measured temperature over UDP
 */

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
//#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

// MAC and IP addresses of Arduino transmitter
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress local_ip(192, 168, 1, 137); // IP of Arduino
IPAddress remote_ip(192, 168, 1, 147);

unsigned int local_Port = 8888;      // local port to listen on
unsigned int remote_Port = 8888;      // local port to send to

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

float R_ref = 3286; // the thermistor model is in kohms
//float R_ref = 9950; // the thermistor model is in ohms

int ThermistorPin = 0;
float log_thermistor, R_thermistor, V_out;
float T = 0.0;
int ADC_readout;
float V_in = 5;

// thermistor model in kohms
//float a =  31.65087152;
//float b = 0.04536822;
//float c = 0.43440881;

// thermistor model in ohms
float a = 31650;
float b = 0.04536822;
float c = 434.40881;
//3.16508731e+04 4.73682307e-02 4.34409432e+02

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void setup() {
  
  // start the Ethernet and UDP:
  Ethernet.begin(mac, local_ip);
  Udp.begin(local_Port);
  Serial.begin(9600);

  // print local IP address:
  Serial.println("Arduino local IP address :");
  Serial.println(Ethernet.localIP());

  // print remote IP address:
  Serial.println("Remote IP address :");
  Serial.println(remote_ip); 
}

void loop() {
  // reading temperature

  ADC_readout = analogRead(ThermistorPin);

  Serial.print("ADC_readout: "); 
  Serial.print(ADC_readout);
  Serial.println(" / 1023"); 

  V_out = float(ADC_readout) * V_in / 1023.0;

  Serial.print("V_out: "); 
  Serial.print(V_out);
  Serial.println(" V"); 
  
  R_thermistor = R_ref * (V_in / V_out - 1.0);

  Serial.print("R_thermistor: "); 
  Serial.print(R_thermistor);
  Serial.println(" kohms"); 
  
  log_thermistor = log((R_thermistor - c) / a);

  T = -1 * log_thermistor / b;

  // conversion of temperature to char array for the UDP package
  char Temperature[3];
  itoa(int(T), Temperature, 10);
  
  Serial.print("Temperature: "); 
  Serial.print(Temperature);
  Serial.println(" C"); 
  
  Serial.println("******** Sending data package ********");

  // send data package
  Udp.beginPacket(remote_ip, remote_Port);
  Udp.write(Temperature);
  Udp.endPacket();
  
  Serial.println("******** Delay ********");
  // short pause
  delay(1000);
}
