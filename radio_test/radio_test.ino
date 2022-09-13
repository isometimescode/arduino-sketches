/**
 * Simple test sketch for a new board using MySensors and RFM69 radio
 */

// Enable debug
#define MY_DEBUG
// #define MY_DEBUG_VERBOSE_RFM69

//-------
// Arduino Pro Mini + RFM69HCW radio
//-------
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_915MHZ

//-------
// MySensors Node config
//-------
#define MY_NODE_ID         200
#define MY_PARENT_ID       0
#define MY_ENCRYPTION_SIMPLE_PASSWD "1234567812345678"

#include <MySensors.h>

MyMessage msg(MY_NODE_ID, V_TRIPPED);
int STATE = 0;

void setup() {
  // initialize serial:
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting setup");
  Serial.println();

  present(MY_NODE_ID, S_DOOR);
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Debug Sensor", "1.0");
}

void loop()
{
  Serial.print("Starting loop: ");
  Serial.println(STATE);
  send(msg.set(STATE, true));
  STATE = STATE == 1 ? 0 : 1;
  wait(5000);
}
