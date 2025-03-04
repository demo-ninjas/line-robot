/********************************************************************************************************
 *                                                                                                      *
 * Hub Line Robot Base Library                                                                          *
 *                                                                                                      * 
 * This library provides a base set of functionality for the Hub Line Robot.                            *
 *                                                                                                      *
 * It provides two key classes:                                                                         *
 *                                                                                                      *
 * - RobotState: A class that manages the state of the robot                                            *            
 * - RobotBoard: A class that manages the hardware of the robot (Motors, Sensors, etc)                  *
 *                                                                                                      * 
 * The library also provides some utility functions for working with the Wifi and I2C devices.          *
 *                                                                                                      * 
 ********************************************************************************************************/

#ifndef HubLineRobotBase_h
#define HubLineRobotBase_h

#include <tuple>
#include <tuple>
#include <memory>
#include <functional>
#include <string>
#include <stdexcept>
#include <map>
#include <vector>

#include <Arduino.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h>

#include "libs/SparkFunLIS3DH.h"
#include "libs/GBALib_DCMotor.h"
#include "libs/i2c_adc_ads7828.h"
#include "libs/HC595.h"
#include "serial-proxy.hpp"

#define MAX_SERVER_CLIENTS 4

/**
 * Base class for holding the Robot State.
 * 
 * Override this class to provide your own custom state.
 */
class RobotState {
  protected:
    int drive_state;

    int btn_state;
    bool btn_state_processed;
    unsigned long btn_time_in_state;
    unsigned long btn_time_in_prev_state;

    unsigned long btn_prev_time_in_down;
    unsigned long btn_prev_time_in_up;

    std::function<void()> idleStartFn;
    std::function<void()> idleEndFn;
    std::function<void()> primedStartFn;
    std::function<void()> primedEndFn;
    std::function<void()> drivingStartFn;
    std::function<void()> drivingEndFn;

    std::function<void()> btnPressedFn;
    std::function<void(long)> btnLongPressedFn;
    std::function<void()> btnDoublePressedFn;

    void handleStateChange(int to_state);
    void btnDown(bool enteringState, long duration, long timeInPrevState);
    void btnUp(bool enteringState, long duration, long timeInPrevState);
  public:
    int ir1;
    int ir2;
    int ir3;
    int ir4;

    int ir1_raw;
    int ir2_raw;
    int ir3_raw;
    int ir4_raw;

    int ir1_hard_threshold;
    int ir2_hard_threshold;
    int ir3_hard_threshold;
    int ir4_hard_threshold;

    int ir1_soft_threshold;
    int ir2_soft_threshold;
    int ir3_soft_threshold;
    int ir4_soft_threshold;

    float poseX;
    float poseY;
    float poseZ;

    RobotState();
    
    void setOnIdleStart(std::function<void()> callback);
    void setOnIdleEnd(std::function<void()> callback);
    void setOnPrimedStart(std::function<void()> callback);
    void setOnPrimedEnd(std::function<void()> callback);
    void setOnDrivingStart(std::function<void()> callback);
    void setOnDrivingEnd(std::function<void()> callback);

    void setOnBtnPressed(std::function<void()> callback);
    void setOnBtnLongPressed(std::function<void(long)> callback);
    void setOnBtnDoublePressed(std::function<void()> callback);
    
    void updateBtnState(int btnState);
    void setIdle();
    void setPrimed();
    void setDriving(); 
    bool isIdle();
    bool isPrimed();
    bool isDriving();
    void setState(int state);
    int driveState();
    void setPose(float x, float y, float z);
    
};;


typedef enum
{
  OLED_TYPE_NONE = 0,           // No display connected
  OLED_TYPE_GROVE = 1,          // Grove 0.96 OLED Display 
	OLED_TYPE_SSD1306 = 2,	      // eg. Adafruit 1306 Mini OLED Display
} OledDisplayType;



class HttpRequest {
  public:
    String method;
    String path;
    String body;
    std::map<String, String> headers;
    std::map<String, String> query;
    HttpRequest();
};

class HttpResponse {
  public:
    int status;
    String body;
    std::map<String, String> headers;

    HttpResponse();
};

class InfraredSensor {
  public: 
    int value;
    int raw_value;
    int baseline;
    int threshold;
    int hard_threshold;
    ADS7828Channel* channel;
    bool triggered;
    bool changed_in_last_update;
    int ir_led_num;
    int indicator_led_num;

    InfraredSensor(ADS7828Channel* channel, int ir_led_num, int indicator_led_num, int baseline, int threshold, int hard_threshold);
    void update(bool updateChannel);
};


/**
 * Main class for managing the Robot Board.
 * 
 * This class provides a set of functions for interacting with the robot development board.
 */
class RobotBoard { 
    protected: 
        RobotState* state;
        unsigned long tick_counter = 0;
        unsigned long last_tick_duration = 0;

        DCMotor* motor1;
        DCMotor* motor2;  
        int motor1_speed = 0;
        int motor2_speed = 0;

        LIS3DH* imu;
        ADS7828* adc;
        
        InfraredSensor* irs[4];
        uint8_t num_infrared_sensors = 4;
        int adc_address;

        HC595  sr = HC595(2, 1, 10, 0); // (chips, latch, clock, data)

        OledDisplayType oled_type;
        int oled_start_line = 0;

        WiFiServer server = WiFiServer(80);
        WiFiClient* serverClients = new WiFiClient[MAX_SERVER_CLIENTS];
        std::map<String, std::function<HttpResponse(WiFiClient*, HttpRequest&, RobotBoard*)>> httpHandlers;
        

        int init_oled();
        void init_imu();
        int readIRSignal(ADS7828Channel* channel, int led);
        void readAccelerometerData();
        void handleServerConnections();
        void inner_init(RobotState* state, bool connect_wifi);

        void continueBaselining();

        void onBtnPressed();
        void onBtnLongPressed(long duration);
        void onBtnDoublePressed();
        void add_default_http_routes();
    public: 
        // Pin mappings for the Board (will be configured by the initBoard methods, and may vary depending on the version of the board being used)
        int MOTOR_1_F;
        int MOTOR_1_R;
        int MOTOR_1_EN;
        int MOTOR_2_F;
        int MOTOR_2_R;
        int MOTOR_2_EN;
        int BOARD_BUTTON;

        bool debug = true;
        SerialProxy logger = SerialProxy(10);

        RobotBoard(){};

        ~RobotBoard();

        /**
         * Perform a tick on the board.
         * 
         * This will update the sensor data, handle server connections, etc...
         * 
         * You should call this function at the start of your main loop.
         */
        void tick();

        /**
         * Get the amount of free memory available on the board.
         * 
         * @return The amount of free memory available
         */
        int availableMemory();

        /**
         * Get the current tick counter.
         */
        long getTickCounter();

        /**
         * Get the duration in milliseconds of the last board tick.
         */
        long getLastTickDuration();

        /**
         * Set **both** motors to the specified speed.
         * 
         * @param speed The speed to set the motors to (-255 to 255)
         */
        void setSpeedOnBothMotors(int speed);

        /**
         * Set the speed of Motor 1.
         * 
         * @param speed The speed to set the motor to (-255 to 255)
         */
        void setMotorSpeedL(int speed);

        /**
         * Set the speed of Motor 2.
         * 
         * @param speed The speed to set the motor to (-255 to 255)
         */
        void setMotorSpeedR(int speed);

        /**
         * Get the current speed of Motor 1.
         * 
         * @return The current speed of Motor 1 (-255 to 255)
         */
        int motor1Speed();

        /**
         * Get the current speed of Motor 2.
         * 
         * @return The current speed of Motor 2 (-255 to 255)
         */
        int motor2Speed();

        /**
         * Check if the button on the robot development board is pressed.
         * 
         * @return True if the button is pressed, False otherwise
         */
        bool isButtonPressed();

        /**
         * Check if an OLED display is connected to the board.
         * 
         * @return True if an OLED display is connected, False otherwise
         */
        bool isOLEDConnected();

        /**
         * Write a message to the OLED display.
         * 
         * @param line The line number to write the message to (will be relative to the first line below the header) - set to negative to overwrite the header region
         * @param msg The message to write to the display
         * @param centered Whether to center the message on the display or not
         */
        void setOLEDLine(int line, const String& msg, bool centered = true);

        /**
         * Clear the OLED display.
         */
        void clearOLED();

        /**
         * Display the header on the OLED display.
         */
        void printOLEDHeader();

        /**
         * Initialise a Robot Development Board version 1.0.
         * 
         * 
         * @param state The Robot State object to use for managing the state of the robot
         * @param acceleromoeter_address The I2C address of the accelerometer (default: 0x19)
         * @param adc_address The I2C address of the ADC (default: 0x13)
         * @param connect_wifi Whether to connect to the Wifi network or not (default: true)
         * @param oled_type The type of OLED display connected (default: OLED_TYPE_NONE)
         */
        void initBoardV1_0(RobotState* state, int acceleromoeter_address = 0x19, int adc_address = 0x13, bool connect_wifi = true, OledDisplayType oled_type = OLED_TYPE_NONE );

        /**
         * Initialise a Robot Development Board version 1.1.
         * 
         * 
         * @param state The Robot State object to use for managing the state of the robot
         * @param acceleromoeter_address The I2C address of the accelerometer (default: 0x19)
         * @param adc_address The I2C address of the ADC (default: 0x13)
         * @param connect_wifi Whether to connect to the Wifi network or not (default: true)
         * @param oled_type The type of OLED display connected (default: OLED_TYPE_NONE)
         */
        void initBoardV1_1(RobotState* state, int acceleromoeter_address = 0x19, int adc_address = 0x13, bool connect_wifi = true, OledDisplayType oled_type = OLED_TYPE_NONE );

        /**
         * Starts the process for baselining the ambient IR readings on each of the IR Sensors, enabling the IR readings to be more precisely representative of the ground reflectance.
         */
        void startBaselining();
        
        /**
         * Set the IR Baselines for the IR Sensors.
         */
        void stabiliseIRBaseline();

        /**
         * Read all the sensor data from the board and update the board state with the latest readings.
         */
        void updateSensorData();

        InfraredSensor* getIRSensor(int sensor);

        uint16_t getADCValue(int channel);

        /**
         * Set the state of the Robot State object.
         * 
         * @param state The Robot State object to set
         */
        void setRobotState(RobotState* state);

        /**
         * Get the current Robot State object.
         * 
         * @return The current Robot State object
         */
        RobotState* getRobotState();

        /**
         * Set the state of the RGB LED on the Arduino board.
         * 
         * @param red The intensity of the red LED (0 to 255)
         * @param green The intensity of the green LED (0 to 255)
         * @param blue The intensity of the blue LED (0 to 255)
         */
        void setArduinoRGB(int red, int green, int blue);

        /**
         * Set the state of the Traffic Light LEDs on the board.
         * 
         * These LEDs are at the front of the board (in the direction of travel) - layed out in a traffic light format (R -> Y -> G).
         * 
         * @param red Whether the red LED should be on or off
         * @param yellow Whether the yellow LED should be on or off
         * @param green Whether the green LED should be on or off
         */
        void setTrafficLight(bool red, bool yellow, bool green);

        /**
         * Sets the state of the specified LED on the Shift Registers.
         * 
         * All the LEDs on the board are connected to the Shift Registers, including the Traffic Light LEDs + the IR Transmission LEDs.
         * 
         * @param led The LED to set (0 to 15)
         * @param on Whether the LED should be on or off
         */
        void setLED(int led, bool on);

        /**
         * Set the state of the IR Transmission LEDs on the board.
         * 
         * @param on Whether the IR Transmission LEDs should be on or off
         */
        void setIRLamps(bool on);

        /**
         * Add an HTTP Handler to the built in HTTP Server.
         * 
         * This will allow you to respond to HTTP requests on the specified path.
         * 
         * The path matching is exact, so you must provide the full path to match, and there is no query string handling (Querystrings will be considered part of the path).
         * 
         * Your handler function should take a `WiFiClient*` and a `HttpRequest` object as arguments, and return a `HttpResponse` object.
         * 
         * Here is an example of a handler function:
         * 
         * ```
         * HttpResponse myHandler(WiFiClient* client, HttpRequest req) {
         *  HttpResponse resp = HttpResponse();
         *  resp.status = 200;
         *  resp.body = "Hello World!";
         *  resp.headers["Content-Type"] = "text/plain";
         *  return resp;
         * }
         * ```
         * 
         * @param path The path to respond to
         * @param handler The function to call when a request is made to the specified path
         */
        void addHttpHandler(String path, std::function<HttpResponse(WiFiClient*, HttpRequest&, RobotBoard*)> handler);
};

/**
Scan the I2C bus attempting to connect to each address.
For every device that responds the address will be written to the Serial console.
Returns the number of devices found.
**/
int scan_i2c();

/**
Connect to the configured Wifi network.
If there is an issue connecting, this function will exit after the specified timeout period.
Returns the status of the connection.
**/
int wifi_connect(int timeout_millis = 5000);
/**
Returns the current IP address of the device.
**/
String wifi_address();
/**
Returns the signal strength of the device to the connected network.
Returned value is the dBm (Decibel Milliwatts)
**/
long wifi_strength();
/**
Scan for each Wifi network that is in range.
Each network SSID will be printed to the serial console and the number of networks will be returned.
@return Number of networks found
**/
int wifi_scan();
/**
Returns the SSID (Network Name) of the network from the list of found networks at the given index.
NB: You must first call the wifi_scan() function to do an initial scan before you can use this method.
**/
String wifi_ssid(int index);


/**
 * Returns a string that is left padded with the specified character, constrained to the specified length.
 */
String padLeft(String str, const size_t num, const char padChar);

#endif  // HubLineRobotBase_h