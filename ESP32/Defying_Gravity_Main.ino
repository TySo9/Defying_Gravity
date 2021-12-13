// @file :      Defying_Gravity_Main.ino
// Description: This code serves as the "brain" of the Defying Gravity: Slip and Trip Simulator Version 2, designed for EE 491 Senior Capstone Spring 2021
// Author:      Tyler Sovar
// Date:        11/28/2021

// -------------------------------------------------------------------------------------------
// WiFi Initiation | BEGIN |
// -------------------------------------------------------------------------------------------

// Load Wi-Fi library
#include <WiFi.h>
#include <WiFiUdp.h>

// Initialize UDP object
WiFiUDP UDP;

// WiFi Constants
const char* ssid     = "espWiFi";   // WiFi Name
const char* password = "123456789"; // Wifi Password
unsigned int localUdpPort = 4210;   // local port to listen on

// UDP Client Info
IPAddress client_IP;
unsigned int client_Port;
bool UDP_connected;

// -------------------------------------------------------------------------------------------
// WiFi Initiation | END |
// -------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------
// ADC Initiation | BEGIN |
// -------------------------------------------------------------------------------------------

// Load ADC Library
//#include "ADC088S102.h" //OLD ADCs (burned up, changed to meet deadlines)
#include "MCP3008.h"

// ADC Objects
// Change depending on which ADCs are being used
//ADC088S102 ADC_1;
//ADC088S102 ADC_2;
//ADC088S102 ADC_3;
MCP3008 ADC_1;
MCP3008 ADC_2;
MCP3008 ADC_3;


// SPI Chip Select pins
#define CS_1  27
#define CS_2  26
#define CS_3  25

const int EDGES = 11;
const int THRESHOLD = 500;  // 51 Ohm

// -------------------------------------------------------------------------------------------
// ADC Initiation | END |
// -------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------
// VESC Control Initiation | BEGIN |
// -------------------------------------------------------------------------------------------

// Code from SolidGeek on GitHub, as recommended by Flipsky (vesc manufacturer)
// License in VescUart folder
#include "VescUart.h"

// Initialize VESC UART Object
VescUart UART_1;
VescUart UART_2;

String command;
float value;
int throttle = 127;

// SET UART Pins
#define U2_RX 16
#define U2_TX 17
#define U1_RX 9
#define U1_TX 10

// Brakes control pin
#define brakes_CNTRL 32

// -------------------------------------------------------------------------------------------
// VESC Control Initiation | END |
// -------------------------------------------------------------------------------------------

// Runs once at power on, initializes objects
void setup() {
  Serial.begin(115200);
  while(!Serial);

  init_wifi();
  
  init_ADCs();
  init_VESC();
  engage_brakes();

  delay(500);
  
}

// Main sketch code, runs on a loop until power off
void loop(){
  
  // Recieve UDP Message, convert to int
  String rcv = UDP_receive();
  int rotations = rcv.toInt();

  // Ensure that # of rotations is an acceptable number (too high will go too far, unsafe)
  if(rotations > 0 && rotations <= 60){
    
    // Send pre-perturbation foot location to iPad via UDP  
    send_surface();

    // Calculate tachometer steps to achieve desired number of rotations
    //    IMPORTANT: Use this once iPad is configureed to do so:   int mv = meters_to_tach(rotations);
    //        OR configure iPad to run this calculation
    int tach_steps = 60 * rotations;
    
    // Induce perturbation based on UDP input
    //    Currently set up to just customize distance
    //    Need to also change force, determine force characteristics in VescTool program
    motor_burst(220, tach_steps, false);
    
    // Send post-perturbation foot location to iPad via UDP  
    send_surface();
  }

  // Print foot location on serial line
  //    Remove when iPad is ready to recieve foot location data via UDP
  print_pressure_grid();

  delay(250);
}

// Unfinished code to parse a UDP recieved string into "command:int" to ensure correct behavior
// Possibly unneccessary if iPad app follows particular UDP send/recive sequence, leaving just in case
void get_UDP_cmd() {
  String rcv = UDP_receive();
  if( rcv != "" ) {
    int colon_index = rcv.indexOf(':');
    if(colon_index >= 0 ) {
      String command = rcv.substring(0, colon_index);
      if( command == "move") {
        int comma_index = rcv.indexOf(',');
        int first_value = rcv.substring(colon_index + 1, comma_index).toInt();
        int second_value = rcv.substring(colon_index + 1).toInt();
      }
    }
  }
  delay(500);
}



// -------------------------------------------------------------------------------------------
// iPad Communication | BEGIN |
// -------------------------------------------------------------------------------------------

void init_wifi() {

  // Set up offline WiFi output
  //    Pretends to be a router
  Serial.print("Setting soft-AP ... ");
  // WIFI INIT FORMAT:  WiFi.softAP(ssid, psk, channel, hidden, max_connection)
  Serial.println(WiFi.softAP(ssid, password) ? "Ready" : "Failed!");

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

  delay(250);
  
  // Begin UDP
  UDP.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}


// Get UDP message, return as String
//    Bug: String appears to have a space newline at the end of the String, causes issues with comparing expected strings
//          - Was not a problem when converting to int, only worry about fixing if using "command:int" method
String UDP_receive(){
  
  char incomingPacket[255];   // Buffer for incoming packets
  
  // Wait until data is available, then read the packet
  int packetSize = UDP.parsePacket();
  if (packetSize)
  {
    // Receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, UDP.remoteIP().toString().c_str(), UDP.remotePort());
    
    int len = UDP.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = '\0';
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
    String msg(incomingPacket);
    
    return msg;
  }

  return "";
}

// Send message over UDP
void UDP_send(char msg[], IPAddress client_IP = UDP.remoteIP(), unsigned int client_port = localUdpPort) {
  
  // send back a reply, to the IP address and port we got the packet from
  UDP.beginPacket(client_IP, client_port);
  
  Serial.printf("Begin reply back to %s, port %d\n", client_IP.toString().c_str(), client_port);
  for(int i=0; i<strlen(msg); i++){
    Serial.print(msg[i]);
    UDP.write(msg[i]);
  }
  Serial.println("\nEnd reply...");
  UDP.endPacket();
}

// Check iPad connection/IP/port via broadcast, unfinished
//    Possibly unneccessary since iPad will be the only device on network
//    Was not neccessary at this point in the project, leaving code in case it is neccessary in the future
bool UDP_check(){
  
  // Send broadcast
  IPAddress broadcast_ip(255, 255, 255, 255);
  UDP_send("Hello?", broadcast_ip, localUdpPort);
  
  // Listen for return
  String returned_msg;
  //int t_i = millis();
  while(returned_msg != "Here!"){
    returned_msg = UDP_receive();
//    int t_f = millis();
//    if(t_f - t_i >= 30000){
//      return false;
//    }
  }
  
  // Check return validity, gather client data
  //if (returned_msg == "Here!"){
    //client_IP = UDP.remoteIP();
    //client_Port = UDP.remotePort();

    Serial.printf("Connected to %s, port %d\n", client_IP.toString().c_str(), client_Port);

    //return true;
  //}

  //return false;
  return true;
}

// -------------------------------------------------------------------------------------------
// iPad Communication | END |
// -------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------
// Pressure Sensor | BEGIN |
// -------------------------------------------------------------------------------------------

// Initialize ADC objects using predetermined pinout (at top of code)
void init_ADCs() {
  ADC_1.begin(CS_1);
  ADC_2.begin(CS_2);
  ADC_3.begin(CS_3);
}

// Print pressure grid to Serial (USB connection)
void print_pressure_grid(){
  int adc_Y[EDGES];
  int adc_X[EDGES];

  bool grid[EDGES+1][EDGES+1] = { false };

  // Y-Axis of pressure grid
  // First ADC
  adc_X[0]  = ADC_1.readADC(0);
  adc_X[1]  = ADC_1.readADC(1);
  adc_X[2]  = ADC_1.readADC(2);
  adc_X[3]  = ADC_1.readADC(3);
  adc_X[4]  = ADC_1.readADC(4);
  adc_X[5]  = ADC_1.readADC(5);
  adc_X[6]  = ADC_1.readADC(6);
  adc_X[7]  = ADC_1.readADC(7);
  // Second ADC
  adc_X[8]  = ADC_2.readADC(0);
  adc_X[9]  = ADC_2.readADC(1);
  adc_X[10] = ADC_2.readADC(2);

  // Y-Axis of pressure grid
  // Second ADC
  adc_Y[0]  = ADC_2.readADC(3);
  adc_Y[1]  = ADC_2.readADC(4);
  adc_Y[2]  = ADC_2.readADC(5);
  adc_Y[3]  = ADC_2.readADC(6);
  adc_Y[4]  = ADC_2.readADC(7);
  // Third ADC
  adc_Y[5]  = ADC_3.readADC(0);
  adc_Y[6]  = ADC_3.readADC(1);
  adc_Y[7]  = ADC_3.readADC(2);
  adc_Y[8]  = ADC_3.readADC(3);
  adc_Y[9]  = ADC_3.readADC(4);
  adc_Y[10] = ADC_3.readADC(5);


  // "Battleship" method for finding foot location
  // TODO: fix "ghosting" issue caused by diagonal foot locations using "cluster" sizes of trigger intersections based on patient foot size
  for( int i = 0; i < EDGES; i++ ){
    for( int j = 0; j < EDGES; j++ ){
      if( (adc_Y[i] >= THRESHOLD) && (adc_X[j] >= THRESHOLD) ){
        grid[i][j] = true;
      }
      else{
        grid[i][j] = false;
      }
    }
  }

  Serial.print(" -------------------------------------------");
  Serial.println();

  // Print boolean grid to show "X" where foot is present
  for( int i = 0; i < EDGES; i++ ){
    Serial.print("|");
     for( int j = EDGES-1; j >= 0; j-- ){
        if( grid[i][j] ){
          Serial.print(" X |");
        }
        else{
          Serial.print("   |");
        }
     }
     Serial.println();
     Serial.println(" -------------------------------------------");
  }
}

// Send pressure grid over UDP
// Never got to test since iPad app wasn't ready. Should work, but do testing before assuming no bugs.
// UDP Message Format Example: "<time>;adc_x[0], adc_x[1]; adc_y[0], adc_y[1];"
void send_surface(){
  int adc_Y[EDGES];
  int adc_X[EDGES];
  int grid[EDGES][EDGES];
  String kinda_csv = "";

  unsigned long t = millis();
  kinda_csv += String(t) + ";";

  // Y-Axis of pressure grid
  // First ADC
  adc_X[0]  = ADC_1.readADC(0);
  adc_X[1]  = ADC_1.readADC(1);
  adc_X[2]  = ADC_1.readADC(2);
  adc_X[3]  = ADC_1.readADC(3);
  adc_X[4]  = ADC_1.readADC(4);
  adc_X[5]  = ADC_1.readADC(5);
  adc_X[6]  = ADC_1.readADC(6);
  adc_X[7]  = ADC_1.readADC(7);
  // Second ADC
  adc_X[8]  = ADC_2.readADC(0);
  adc_X[9]  = ADC_2.readADC(1);
  adc_X[10] = ADC_2.readADC(2);

  // Y-Axis of pressure grid
  // Second ADC
  adc_Y[0]  = ADC_2.readADC(3);
  adc_Y[1]  = ADC_2.readADC(4);
  adc_Y[2]  = ADC_2.readADC(5);
  adc_Y[3]  = ADC_2.readADC(6);
  adc_Y[4]  = ADC_2.readADC(7);
  // Third ADC
  adc_Y[5]  = ADC_3.readADC(0);
  adc_Y[6]  = ADC_3.readADC(1);
  adc_Y[7]  = ADC_3.readADC(2);
  adc_Y[8]  = ADC_3.readADC(3);
  adc_Y[9]  = ADC_3.readADC(4);
  adc_Y[10] = ADC_3.readADC(5);

  // Fill x-axis data 
  for( int i=0; i < EDGES; i++ ){
      kinda_csv += String(adc_X[i]) + ",";
  }
  kinda_csv += ";";

  // Fill y-axis data
  for( int i=0; i < EDGES; i++ ){
      kinda_csv += String(adc_Y[i]) + ",";
  }
  kinda_csv += ";";

  Serial.println("SENDING UDP:\t" + kinda_csv);
  char* c = strcpy(new char[kinda_csv.length() + 1], kinda_csv.c_str());
  UDP_send(c);
  
}

// -------------------------------------------------------------------------------------------
// Pressure Sensor | END |
// -------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------
// VESC Control | BEIGN |
// -------------------------------------------------------------------------------------------

// Initialize VESC Motor controllers
void init_VESC(){
  // Configure UART interfaces
  Serial1.begin(115200, SERIAL_8N1, U1_RX, U1_TX);
  Serial2.begin(115200, SERIAL_8N1, U2_RX, U2_TX);
  
  while (!Serial1) {;}
  while (!Serial2) {;}
  
  // Define which ports to use as UART
  UART_1.setSerialPort(&Serial1);
  UART_2.setSerialPort(&Serial2);

  // Initialize Brakes Control
  pinMode(brakes_CNTRL, OUTPUT);
  digitalWrite(brakes_CNTRL, HIGH);
  
  Serial.println("BEGIN VESC CONTROL...");
}

// Check if VESCs are connected via UART, print telemetry data for testing, fill bool array to verify connection elsewhere
bool VESC_connected[2] = {false, false};
void check_vescs(){
  if ( UART_1.getVescValues() ) {
      Serial.println("UART 1:------");
      Serial.println("RPM:\t" + String(UART_1.data.rpm));
      Serial.println("Voltage:\t" + String(UART_1.data.inpVoltage));
      Serial.println("Current:\t" + String(UART_1.data.ampHours));
      Serial.println("Tachometer:\t" + String(UART_1.data.tachometerAbs));
      VESC_connected[0] = true;
  }
  else
  {
    Serial.println("U1: Failed to get data!");

    VESC_connected[0] = false;
  }

  if ( UART_2.getVescValues() ) {
      Serial.println("UART 2:------");
      Serial.println("RPM:\t" + String(UART_2.data.rpm));
      Serial.println("Voltage:\t" + String(UART_2.data.inpVoltage));
      Serial.println("Current:\t" + String(UART_2.data.ampHours));
      Serial.println("Tachometer:\t" + String(UART_2.data.tachometerAbs));
      VESC_connected[1] = true;
  }
  else
  {
    Serial.println("U2: Failed to get data!");

    VESC_connected[1] = false;
  }
  delay(500);
}

// Engage solenoid brakes
void engage_brakes() { 
  Serial.println("ENGAGE BRAKES");
  digitalWrite(brakes_CNTRL, HIGH);
  return;
}

// Disengage solenoid brakes
void disengage_brakes() {
  Serial.println("DIS-ENGAGE BRAKES");
  digitalWrite(brakes_CNTRL, LOW);
  delay(500);
  return;
}

// Engage in-motor brakes sequence
void motor_stop() {
    // Move motors in reverse to brake
    UART_1.nunchuck.valueY = 0;
    UART_2.nunchuck.valueY = 0;
    UART_1.setNunchuckValues();
    UART_2.setNunchuckValues();

    // Change to make brake time longer or faster
    delay(2000);

    // Set motors to neutral
    UART_1.nunchuck.valueY = 127;
    UART_2.nunchuck.valueY = 127;
    UART_1.setNunchuckValues();
    UART_2.setNunchuckValues();
    
    Serial.println("STOPPED!");
}

// Convert meters to tachometer rotation values
int meters_to_tach(float meters){
  // Wheel diameter:  90 mm
  // Full Rotation:   282.743339 (wheel circumference) = 60 tachs
  // Conversion:      1 tach = 4.71239 mm, 
  //                  1 mm = 0.21220659 tachs
  //                  1 meter = 212.20659 tachs
  
  int wheel_d   = 90;
  int rot_tach  = 60;
  
  float rot_mm  = 3.14 * wheel_d;
  float conv    = (rot_tach/rot_mm) * 1000;

  // Round because Tachometer only deals with whole numbers
  int tach_steps = round(conv * meters);
  
  return tach_steps;
}

// Induce perturbation
void motor_burst(int throttle, int tach_steps, bool show_data) {

  // Ensure VESC connection, end if failed
  check_vescs();
  if( (VESC_connected[0] && VESC_connected[1]) == false ) {
    Serial.println("ERROR: NO VESCS CONNECTED");
    return;
  }
  else if ( VESC_connected[0] == false ) {
    Serial.println("ERROR: VESC 1 NOT CONNECTED");
    return;
  }
  else if ( VESC_connected[1] == false ) {
    Serial.println("ERROR: VESC 2 NOT CONNECTED");
    return;
  }

  // Turn off solenoid brakes before motor movement
  disengage_brakes();

  // Check throttle value
  //    127     = Neutral
  //    0-126   = Backwards, smaller is more force 
  //    128-255 = Forwards, larger is more force
  if ( throttle >= 0 && throttle <= 255) {
    
    // Get initial tachometer value to determine distance moved
    UART_1.getVescValues();
    int init_tach = UART_1.data.tachometerAbs;

    // Set motors to desired throttle (force)
    UART_1.nunchuck.valueY = throttle;
    UART_2.nunchuck.valueY = throttle;
    UART_1.setNunchuckValues();
    UART_2.setNunchuckValues();

    Serial.println(throttle);
    Serial.println("STARTED");
    
    int tach = 0;
    // Show data for troubleshooting if specified in function call
    if (show_data){ 
      // Loop until desired distance moved has been met
      while(tach <= init_tach + tach_steps)
      {
        // Get current tachometer value to be compared
        UART_1.getVescValues();
        tach = UART_1.data.tachometerAbs;

        // Print telemetry data
        Serial.println("RPM:\t" + String(UART_1.data.rpm));
        Serial.println("Voltage:\t" + String(UART_1.data.inpVoltage));
        Serial.println("Current:\t" + String(UART_1.data.ampHours));
        Serial.println("Tachometer:\t" + String(UART_1.data.tachometerAbs));

        // Software E-Stop
        if (UDP_receive().toInt() == -999){
          engage_brakes();
          motor_stop();
          break;
        }    
      }
    }
    
    // Normal operation
    else{
      // Loop until desired distance moved has been met
      while(tach <= init_tach + tach_steps)
      {
        // Get current tachometer value to be compared
        UART_1.getVescValues();
        tach = UART_1.data.tachometerAbs;

        // Software E-Stop
        if (UDP_receive().toInt() == -999){
          engage_brakes();
          motor_stop();
          break;
        }
      }
    }

    // Initiate stop sequence
    engage_brakes();  // Solenoid brakes
    motor_stop();     // In-motor brakes
  }

  // Account for user error to not send invalid UART data to VESCs
  else
  {
    Serial.println("Unvalid throttle value");
    engage_brakes();
    motor_stop(); 
  }
}

// -------------------------------------------------------------------------------------------
// VESC Control | END |
// -------------------------------------------------------------------------------------------
