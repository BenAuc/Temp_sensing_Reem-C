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
unsigned int remote_Port = 7777;      // local port to send to

float R_ref = 3286; // the thermistor model is in kohms
//float R_ref = 9950; // the thermistor model is in ohms

int nb_sensors = 3;
int Pin0 = 0;
float log_thermistor, R_thermistor, V_out;
float T = 0.0;
int ADC_readout;
float V_in = 5;
char separator[2] = "//";
char temp_string[3];

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
 
  char Temperature[nb_sensors*5];
  int nb_sensors = 3;
  for (int i = 0; i < nb_sensors; i++) {
    ADC_readout = analogRead(i);
  
    V_out = float(ADC_readout) * V_in / 1023.0;
    
    R_thermistor = R_ref * (V_in / V_out - 1.0);
    
    log_thermistor = log((R_thermistor - c) / a);
  
    T = -1 * log_thermistor / b;

    itoa(int(T), temp_string, 10);
    if (i == 0) {
      strcpy(Temperature, temp_string);
    }
    else {
      strcat(Temperature, temp_string);
    }
    if (i < nb_sensors - 1) {
      strcat(Temperature, separator);
    }
  }
  
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
  //delay(500);
}
