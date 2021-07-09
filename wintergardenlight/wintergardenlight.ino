/***************************************************************************
  Program to synchronise ESP8266 connected LED strips through a mesh network
  for the Winter Garden Light Project by the Light Nelson Collective
 ***************************************************************************/


// This is the main library to handle the mesh network, available though the
// library manager in the Arduino IDE or from Github
// https://www.arduino.cc/reference/en/libraries/painless-mesh/

#include <painlessMesh.h>

// The Arafruit library makes it easy to control addressable LEDs
// There are other libraries, but this is one of the most used
// Once the library is installed in the IDE you will find examples to work with
// https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use

#include <Adafruit_NeoPixel.h>

// The minimum number of stars it needs to connect with to run the sequence
// Changing this to '0' will mean it doesn't have to connect to others to run
// the sequences

#define MIN_NODES 1

// Some setup values (these can be modified, as long as each device has the same)
#define   MESH_SSID       "WinterGardenLight"
#define   MESH_PASSWORD   "L1ghtN3lson"
#define   MESH_PORT       5555

// The pin that the dat pina of the LED strip is connected to
#define LED_PIN    D2

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 40
const uint16_t  ANIMATION_CYCLES = 800000/LED_COUNT; // Millseconds per animation cycle

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

const uint16_t NUM_SEQUENCES = 8;
const uint16_t SEQUENCE_COUNT = LED_COUNT * NUM_SEQUENCES; // Assumes each sequence will loop based on number of LEDs

// Routines called from the mesh - kept in for debugging
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void delayReceivedCallback(uint32_t from, int32_t delay);

// The Mesh object
painlessMesh  mesh;

// A list to keep track of the connected nodes (stars)
SimpleList<uint32_t> nodes;

// this routing is called first
void setup() {
  // Useful for debuggin when connected to a computer (can send output to the Serial Monitor in 'Tools' menu)
  Serial.begin(115200);

// Setup the mesh
  mesh.setDebugMsgTypes(ERROR | DEBUG);  // set before init() so that you can see error messages

  mesh.init(MESH_SSID, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  strip.begin();
  delay(1000);
  strip.show();
  strip.setBrightness(100);

  randomSeed(analogRead(A0));
}

bool bShow = false;

const uint16_t MAX_NODES = 10;

// The main loop, check the mesh and then update the LEDs based on the mesh time (or show a status LED of non connected)
void loop() {
  mesh.update();
  checkNodes();
}

uint16_t statusPixelIndex = 0;

#define STATUS_LED_DELAY 1000

unsigned long lLastTime = 0;

// When no stars are connected, a single LED flashes
bool bStatusLEDOn = false;

// Check how many stars are connected
void checkNodes() {
  uint16_t iNumNodes = nodes.size();
  if (iNumNodes >= MIN_NODES) {
    
    // At least one star is connected
    iNumNodes = MAX_NODES;
    uint16_t brightness = 55 + (200 * iNumNodes) / MAX_NODES;
    strip.setBrightness(brightness);
    bShow = true;
    handleSequence();
  }
  else {
    
    // No stars connected
    if (bShow) {
      bShow = false;
      statusPixelIndex = 0;
      lLastTime = millis();
      clearPixels();
    }
    else {
      
      // Flash a pixel on and off (with a delay of one second)
      unsigned long lTime = millis();
      unsigned long lTimeDiff = lTime - lLastTime;
      if (bStatusLEDOn && (lTimeDiff > STATUS_LED_DELAY * 2)) {
        strip.setPixelColor(statusPixelIndex, 0x000000);
        strip.show();
        lLastTime = lTime;
        bStatusLEDOn = false ;

        if (statusPixelIndex == (LED_COUNT - 1))
          statusPixelIndex = 0;
        else
          statusPixelIndex++;
      }
      else if (!bStatusLEDOn && (lTimeDiff > STATUS_LED_DELAY)) {
        
        //statusPixelIndex = (LED_COUNT*(lTimeDiff-STATUS_LED_DELAY))/STATUS_LED_DELAY;
        strip.setPixelColor(statusPixelIndex, 0xff0000);
        strip.show();
        bStatusLEDOn = true;
      }
    }
  }
}

// set all the pixels off
void clearPixels() {
  setPixels(0);
}

// Set all the pixels to a set colour
void setPixels(uint32_t c) {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}

uint32_t lastPixel = 100;

// Fade to a set colour
void fade(uint8_t r, uint8_t g, uint8_t b, uint8_t sequencePixel) {
  uint32_t c = 0;
  uint8_t index;
  if (sequencePixel < (LED_COUNT/2)) {
    index = (sequencePixel * 2);
  }
  else {
    index = ((LED_COUNT - sequencePixel) * 2) ;
  }

  // A colour is a 24 bit number based on three 8 bit numbers representing red, green and blue 
  c |= (((uint32_t)r) * index)/ LED_COUNT << 16;
  c |= (((uint32_t)g) * index)/ LED_COUNT << 8;
  c |= (((uint32_t)b) * index)/ LED_COUNT;
  setPixels(c);
}

// Show a shifting rainbow - will shift all the colour hues along the strip
void showRainbow(uint8_t sequencePixel) {
  for (int i = 0; i < LED_COUNT; i++) {
    int iDiff = i - sequencePixel;
    if (iDiff < 0)
      iDiff = -iDiff;
    uint32_t c = (65536 / LED_COUNT) * iDiff;
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(c)));
  }
}

// Flash white
void flash(uint8_t sequencePixel) {
  bool ison = (sequencePixel % 2) == 1;
  if (ison) {
    setPixels(0x888888);
  }
  else {
    setPixels(0x0);
  }
}

// Random Colours
void randomRainbow() {
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t w = random(3); // Select Red Green or Blue colour
    uint32_t c = random(0x22, 0xff) << (w * 8); // Set the colour brightness to be a brigtness 0x22 to 0xff

    strip.setPixelColor(i, c);
  }
}

// Glow white
void lightGlow(uint8_t sequencePixel) {
  fade(0x22, 0x22, 0x22, sequencePixel);
}

// Show the current sequence 
void handleSequence() {

  // The mesh keeps a common time, a number that will repeat in 50 days (approx)
  // We can use this to synchronize the stars
  uint32_t nodeTime = mesh.getNodeTime();
  
  // Calculate a point along the timeline based on the number of pixels and the number of sequences
  uint32_t sequencePoint = (nodeTime / (ANIMATION_CYCLES * NUM_SEQUENCES)) % SEQUENCE_COUNT;
  
  if (lastPixel == sequencePoint) // If it's not chanced then no point in updating
    return;
  lastPixel = sequencePoint;

  // Calculate the current sequence 
  uint8_t sequence = sequencePoint / LED_COUNT;

  // Calculate the pixel within that sequence
  uint8_t sequencePixel = sequencePoint - (sequence * LED_COUNT);

  // Run the routing corresponsing to the sequence
  switch (sequence) {
    case 0:
      flash(sequencePixel);
      break;
    case 1:
    case 2:
      showRainbow(sequencePixel);
      break;
    case 3: // Fade to Red
      fade(0xff, 0x00, 0x00, sequencePixel);
      break;
    case 4: // Fade to Green
      fade(0x00, 0xff, 0x00, sequencePixel);
      break;
    case 5: // Fade to Blue
      fade(0x00, 0x00, 0xff, sequencePixel);
      break;
    case 6: 
      randomRainbow();
      break;
    case 7:
      lightGlow(sequencePixel);
      break;
  }

  strip.show();
}

// Mesh routines below, mainly for debugging/interest
//====================================================================================

void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
  Serial.printf("New Connection, %s\n", mesh.subConnectionJson(true).c_str());
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");

  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}
