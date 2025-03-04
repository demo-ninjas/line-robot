/************************************************************************************************************************************
 *                                                                                                                                  *                     
 *                                            Hub Line Robot Starter Code                                                           *
 *                                                                                                                                  *                
 * This is the starter code for working with the Hub Line Robot Development Board with an attached Arduino MKR 1010.                *
 *                                                                                                                                  *
 * To get started, install the Arduino IDE, and then install the WiFiNINA library (open the Library panel and search for WiFiNINA)  *
 *                                                                                                                                  *
 * The Base Code provides a set of classes and functions to help you get started with the Hub Line Robot Development Board.         *
 *                                                                                                                                  *     
 * The main classes are:                                                                                                            *
 * - RobotState: A class that manages the state of the robot (IDLE, PRIMED, DRIVING)                                                *                    
 * - RobotBoard: A class that manages the hardware of the robot (Motors, Sensors, etc)                                              *
 *                                                                                                                                  *
 *                                                                                                                                  *
 * The robot has three states:                                                                                                      *                       
 * - IDLE      [When the robot is not doing anything]                                                                               *
 * - PRIMED    [When the button is pushed in - hold the button in and it will remain in primed state until you release the button]  *
 * - DRIVING   [When the robot is driving]                                                                                          *
 *                                                                                                                                  *                 
 * You can attach your own code to handle entering and exiting these states, as well as code to run while in each state.            *
 *                                                                                                                                  *                         
 * The base code also provides a simple HTTP server (on port 80) that you can connect to, allowing you to provide the ability to    *
 *  send commands to the robot or obtain information via HTTP requests.                                                             *
 *                                                                                                                                  *               
 * If an OLED display is connected, the default base code will display the IP Address of the device, along with the current state.  *
 *                                                                                                                                  *                 
 ************************************************************************************************************************************/


#include <Arduino.h>
#include "src/hub-line-robot-base.h"


/**
Example Custom State for your robot (overriding the default state object)
**/
class MyRobotState : public RobotState {
  public:
    long my_variable = 0;
    
    MyRobotState() : RobotState() { }
};

/**
 * Global Variables
 */
static const long start_millis = millis();
RobotBoard* board = nullptr;
MyRobotState* state = nullptr;


/**
 * Example function that will be called when the robot enters the IDLE state
 * 
 * Attach your own code here to handle entering the IDLE state.
 */
void onIdleBegin() {
  Serial.println("IDLE Start");
  board->setTrafficLight(true, false, false);
  board->clearOLED();
  board->printOLEDHeader();
  board->setOLEDLine(3, "I D L E", true);
  board->setSpeedOnBothMotors(0);  // Turn off both motors
}

/**
 * Example function that will be called when the robot enters the PRIMED state
 * 
 * Attach your own code here to handle entering the PRIMED state.
 * 
 * The PRIMED state is when the button is pushed in - if you hold the button in it will remain in primed state until you release the button
 */
void onPrimedBegin() {
  Serial.println("Primed Start");
  board->setTrafficLight(false, true, false);
  board->clearOLED();
  board->printOLEDHeader();
  board->setOLEDLine(3, "P R I M E D", true);   
}

/**
 * Example function that will be called when the robot enters the DRIVING state
 * 
 * Attach your own code here to handle entering the DRIVING state.
 */
void onDrivingBegin() {
  Serial.println("Driving Start");
  board->clearOLED();
  board->printOLEDHeader();
  board->setTrafficLight(false, false, true);
  board->setOLEDLine(1, "D R I V I N G", true);
}

/**
 * Example function that will be called when the robot exits the DRIVING state
 * 
 * Attach your own code here to handle exiting the DRIVING state.
 */
void onDrivingEnd() {
  Serial.println("Driving End");
  board->setSpeedOnBothMotors(0);  // Turn off both motors
  delay(200);
}



/**
 * Example function that will be called regularly when the robot is in the IDLE state
 * 
 * Attach your own code here to handle actions during the IDLE state.
 */
void idleActions() {
    // Nothing...
}

/**
 * Example function that will be called regularly when the robot is in the PRIMED state
 * 
 * Attach your own code here to handle actions during the PRIMED state.
 */
void primedActions() {
  // Nothing...
}

/**
 * Example function that will be called regularly when the robot is in the DRIVING state
 * 
 * Attach your own code here to handle actions during the DRIVING state.
 * 
 * This is the place to put your driving code :)
 */
void drivingActions() {
  // If driving for more than 30s, stop driving...
  if (state->poseX > 0.88) {  // If the car is held vertically - then stop driving immediately
    state->setIdle();
    return;
  }
  
  // This example will drive forward for 2s, pause for half a second, then drive backward for 2s
  board->setSpeedOnBothMotors(250);
  delay(2000);
  board->setSpeedOnBothMotors(0);
  delay(500);
  board->setSpeedOnBothMotors(-250);
  delay(2000);
  board->getRobotState()->setIdle();  
}


/**
 * This is the setup function that will be called once when the board is powered on (or after a reset)
 * 
 * Attach any setup code you need here.
 */
void setup() {
  
  // We'll initialise the Serial port and give it some time to connect before continuing (Having serial COM is useful for debugging and monitoring when attachd to a computer)
  Serial.begin(9600);
  while (!Serial && millis() - start_millis < 200) {
    delay(10);     // Pause until serial console opens (for a few millis anyway...)
  }


  Serial.println("Innovation Hub - Line Robot - Initialising...");

  // Setup the Robot State object to be the customer state object defined above
  state = new MyRobotState();
  
  // There are three states in the RobotState: 
  //  - IDLE      [When the robot is not doing anything]
  //  - PRIMED    [When the button on the board is pushed in - if you hold the button in it will remain in primed state until you release the button]
  //  - DRIVING   [When the robot is driving]
  // You set a callback function to be invoked whenever you enter or exit one of these states, using the `set[State]Start` and `set[State]End` methods.
  // eg. state->setIdleStart(onIdleBegin);  <-- The provided `onIdleBegin` function will be called whenever the robot enters the IDLE state
  state->setOnIdleStart(onIdleBegin);
  state->setOnPrimedStart(onPrimedBegin);
  state->setOnDrivingStart(onDrivingBegin);
  state->setOnDrivingEnd(onDrivingEnd);

  // Configure + Initialise the Robot board - you will be using version 1.1 of the board, so call the `initBoardV1_1` method
  // There are some switches on the board that you can use to configure the I2C addresses for the Accelerometer and ADC
  // The OLED display is optional, and if you attach one, you must specify the type of OLED display you have connected (OLED_TYPE_GROVE or OLED_TYPE_SSD1306) [These are the two types of display we have]
  board = new RobotBoard();
  board->initBoardV1_1(state, 0x19, 0x48, true, OLED_TYPE_SSD1306);  // 0x19 for the Accelerometer, 0x17 for ADC, true for WiFi connection, OLED_TYPE_SSD1306 for the OLED display

  // The Arduino has a built in wifi, so it can connect to the Devices network (if you don't know the password, just ask any of the Hub team for it).
  //  - You must have the correct SSID and Password in the `arduino_secrets.h` file for the connection to work.
  //  - If the Wifi connection is successful, the LED in the centre of the Arduino will be set to green, and the IP address of the device will be printed to both the serial console and the OLED display (if one is attached)
  //  - If the Wifi connection fails, the LED will be set to red, and an error message will be printed to the serial console.
  //  - If you don't want to connect to the Wifi network, you can set the `connect_wifi` parameter above to false.
  // 
  // Assuming a successful connection to the WIFI, the Board will launch a small HTTP Server, so you can connect to the device and send commands to it via HTTP.
  // The Server will be listening on port 80, and you can add your own handlers to respond to different paths.
  // Here's an example HTTP Handler that will respond on the `/set-traffic-light` path by setting the state of the traffic light LEDs at the front of the board.
  board->addHttpHandler("/set-traffic-light", [](WiFiClient* client, HttpRequest req, RobotBoard* board) {
    std::string msg = "Set Traffic Lights to: \n";
    bool red = false, yellow = false, green = false;
    if (req.query.find("red") != req.query.end()) {
      red = req.query["red"] == "true";
      msg += " -    RED: ";
      msg += (red ? "ON" : "OFF");
      msg += "\n";
    }
    
    if (req.query.find("yellow") != req.query.end()) {
      yellow = req.query["yellow"] == "true";
      msg += " - YELLOW: ";
      msg += (yellow ? "ON" : "OFF");
      msg += "\n";
    } else if (req.query.find("orange") != req.query.end()) {
      yellow = req.query["yellow"] == "true";
      msg += " - ORANGE: ";
      msg += (yellow ? "ON" : "OFF");
      msg += "\n";
    }
    
    if (req.query.find("green") != req.query.end()) {
      green = req.query["green"] == "true";
      msg += " -  GREEN: ";
      msg += (green ? "ON" : "OFF");
      msg += "\n";
    }
    board->setTrafficLight(red, yellow, green);

    HttpResponse resp = HttpResponse();
    resp.status = 200;
    resp.body = String(msg.c_str());
    resp.headers["Content-Type"] = "text/plain";
    return resp;
  });

  // Add any additional HTTP Handlers here...

  // Add any additional setup code here...

  // (optional) Scan for I2C devices, and Print their addresses to the serial port (useful for confirming that you've specified the correct Accelerometer and ADC addresses)
  scan_i2c();

  // We'll run the Idle Begin Callback Once now (as we're technically entering the IDLE state now - we could've done this during init, but but not doing this automatically, it provides more flexibility - perhaps you don't want to start in the IDLE state...)
  onIdleBegin();

  // Take a breather for a bit before we start the main loop
  delay(100);
}

/**
 * This is the main loop function that will be called repeatedly after the setup function has completed
 * 
 * Attach any code that you want to run repeatedly here.
 * 
 */
void loop() {  
  // Step 1: Tick the Board (update sensor data, handle HTTP server connections, etc...)
  board->tick();

  // Step 1.5: Update the LEDs based on the IR sensor values (to provide a nice visual indication of the IR sensor values)
  if (state->driveState() <= 10 && board->getTickCounter() & 0x11 == 0x11) {
    for (int i = 0; i < 4; i++) {
      InfraredSensor* sensor = board->getIRSensor(i);
      if (sensor->changed_in_last_update) {
        board->setLED(sensor->indicator_led_num, sensor->triggered);
      }
    }
  }

  // Step 2: Perform Action (based on current State)
  if (board ->getTickCounter() & 0x1 != 0x1) {                  // IDLE is when the robot is not doing anything
    idleActions();
  } else if (state->isPrimed()) {                               // PRIMED is when the button is pushed in - if you hold the button in it will remain in primed state until you release the button
    primedActions();
  } else if (state->isDriving()) {                              // DRIVING, the robot is actively driving, so do whatever needs to be done to continue driving for this tick
    drivingActions();
  }

  // Add any additional loop code here...
}
