#ifndef HUB_LINE_ROBOT_ROUTES_H
#define HUB_LINE_ROBOT_ROUTES_H

#include <vector>
#include <string>
#include "hub-line-robot-base.h"

void RobotBoard::add_default_http_routes() {
    // Default Route
    this->addHttpHandler("/", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;

      String index_html = R"rawliteral(
      <html>        
      <head>
      <title>Hub Line Robot</title>
      <link href="style.css" rel="stylesheet"></link>
      </head>
      <body>
        <div class="container">
          <h1>Innovation Hub - Line Robot</h1>
          <button onclick="callAction('/btn-press', this)" class="btn redbtn">Button Press</button>
          <button onclick="callAction('/btn-long-press', this)" class="btn redbtn">Button Long Press</button>
          <button onclick="callAction('/btn-double-press', this)" class="btn redbtn">Button Double Press</button>
          <br/>
          <button onclick="callAction('/ir', this)" class="btn">Infrared</button>
          <button onclick="callAction('/adc', this)" class="btn">ADC</button>
          <button onclick="callAction('/adc', this)" class="btn">Pose</button>
          <button onclick="callAction('/state', this)" class="btn">State</button>
          <br/>
          <button onclick="callAction('/set-state?state=0', this)" class="btn greenbtn">Set Idle</button>
          <button onclick="callAction('/set-state?state=1', this)" class="btn greenbtn">Set Primed</button>
          <button onclick="callAction('/set-state?state=2', this)" class="btn greenbtn">Set Driving</button>
          &nbsp;&nbsp;
          <button onclick="callAction('/start-baseline', this)" class="btn yellowbtn">Baseline</button>
          <br />
          <button onclick="callAction('/log', this)" class="btn">Get Log</button>
        </div>
        <div id="respDiv"></div>
        <hr />
        <h3>Routes:</h3>
        <ul id="routes"></ul> 
        <script type="text/javascript" src="/script.js"></script>
      </body>
      </html>)rawliteral";
      resp.body = index_html;
      resp.headers["Content-Type"] = "text/html";
      return resp;
    });

    this->addHttpHandler("/script.js", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;

      String script_text = R"rawliteral(
      function callAction(actionPath, el) {
            let elMsg = '';
            if (el) {
              el.disabled = true;
              elMsg = el.innerText;
              el.innerText = "Processing...";
            }
            fetch(actionPath)
              .then(response => response.text())
              .then(data => {
                console.log(data);
                document.getElementById('respDiv').innerText = data;
              })
              .finally(() => {
                if (el) {                            
                  el.disabled = false;
                  el.innerText = elMsg;
                }
              });
      }

      fetch('/routes')
        .then(response => response.json())
        .then(data => {
          let routes = document.getElementById('routes');
          data.forEach(route => {
            let li = document.createElement('li');
            li.innerText = route.path + ' - ' + route.description;
            routes.appendChild(li);
          });
        });
      )rawliteral";
      resp.body = script_text;
      resp.headers["Content-Type"] = "text/javascript";
      return resp;
    });

    this->addHttpHandler("/style.css", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;

      String css_text = R"rawliteral(
        body {            
          font-family: Arial, sans-serif;
          background-color: #f4f4f4;
          color: #333;
          margin: 0;
          padding: 20px;
        }
        h1 {
          color: #333;
        }
        .container {
          max-width: 950px;
          margin: 0 auto;
          padding: 20px;
          background-color: #fff;
          border-radius: 5px;
          box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
        }
        .btn {
          display: inline-block;
          padding: 10px 20px;
          background-color: #007bff;
          color: #fff;
          text-decoration: none;
          border-radius: 5px;
          transition: background-color 0.3s;
          margin-top: 2rem;
          margin-left: 1.5rem;
        }
        .btn:hover {
          background-color: #0056b3;
        }
        .btn:active {
          background-color: #004085;
        }

        .redbtn {
          background-color: #dc3545;
        }
        .greenbtn {
          background-color: #28a745;
        }
        .yellowbtn {
          background-color: #ffc107;
        }
        .redbtn:hover {
          background-color: #c82333;
        }
        .greenbtn:hover {
          background-color: #218838;
        }
        .yellowbtn:hover {
          background-color: #d39e00;
        }
        .redbtn:active {
          background-color: #bd2130;
        }
        .greenbtn:active {
          background-color: #1e7e34;
        }
        .yellowbtn:active {
          background-color: #c69500;
        }
        
        #respDiv {
          font-size: 1.1rem;
          margin-top: 2rem;
          padding: 1rem;
          background-color: #f4f4f4;
          border-radius: 5px;
          box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
        }
      )rawliteral";
      resp.body = css_text;
      resp.headers["Content-Type"] = "text/css";
      return resp;
    });

    this->addHttpHandler("/routes", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      String body = "[";
      bool first = true;
      for (const auto& handler : this->httpHandlers) {
        String path = handler.first;
        if (path == "/" || path == "/script.js" || path == "/style.css" || path == "/routes") {
          continue;
        }

        if (!first) {
          body += ",";
        } else {
          first = false;
        }
        
        if (path == "/log") {
          body += "{\"path\":\"/log\",\"description\":\"Get upto the last 10 log lines (Default 10, use ?lines=<n>)\"}";
        } else if (path == "/adc") {
          body += "{\"path\":\"/adc\",\"description\":\"Read the ADC and return the values for each of the 8 channels\"}";
        } else if (path == "/ir") {
          body += "{\"path\":\"/ir\",\"description\":\"Returns the current IR values (along with the Raw values, baselines + thresholds)\"}";
        } else if (path == "/pose") {
          body += "{\"path\":\"/pose\",\"description\":\"Returns the current pose of the robot, given as x, y, z values between 0-1\"}";
        } else if (path == "/state") {
          body += "{\"path\":\"/state\",\"description\":\"Returns the current state of the robot (eg. 0: IDLE, 1: PRIMED, 2: DRIVING, etc...)\"}";
        } else if (path == "/btn-press") {
          body += "{\"path\":\"/btn-press\",\"description\":\"Simulate a button push\"}";
        } else if (path == "/btn-long-press") {
          body += "{\"path\":\"/btn-long-press\",\"description\":\"Simulate a long press of the button (>1200ms)\"}";
        } else if (path == "/btn-double-press") {
          body += "{\"path\":\"/btn-double-press\",\"description\":\"Simulate a double press of the button\"}";
        } else if (path == "/set-ir-baselines") {
          body += "{\"path\":\"/set-ir-baselines\",\"description\":\"Sets the IR Baseline values (use ?0=<val>&1=<val>&2=<val>&3=<val>)\"}";
        } else if (path == "/set-ir-hard-thresholds") {
          body += "{\"path\":\"/set-ir-hard-thresholds\",\"description\":\"Sets the IR Hard Threshold values (use ?0=<val>&1=<val>&2=<val>&3=<val>)\"}";
        } else if (path == "/set-ir-soft-thresholds") {
          body += "{\"path\":\"/set-ir-soft-thresholds\",\"description\":\"Sets the IR Soft Threshold values (use ?0=<val>&1=<val>&2=<val>&3=<val>)\"}";
        } else if (path == "/set-state") {
          body += "{\"path\":\"/set-state\",\"description\":\"Sets the drive state of the board (use ?state=(0|1|2|etc...)), eg. ?state=1\"}";
        } else if (path == "/set-led") {
          body += "{\"path\":\"/set-led\",\"description\":\"Sets the state of a LED (use ?<num>=(0|1)), you can also specify ?all=(0|1)\"}";
        } else if (path == "/set-motor-speed") {
          body += "{\"path\":\"/set-motor-speed\",\"description\":\"Sets the speed of the motor(s) (use ?(left|right)=(-255-255)), eg. ?left=200&right=-100\"}";
        } else if (path == "/start-baseline") {
          body += "{\"path\":\"/start-baseline\",\"description\":\"Starts the baselining process\"}";
        } else {
          body += "{\"path\":\"" + path + "\",\"description\":\"No Description\"}";
        }
      }
      body += "]";
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = body;
      resp.headers["Content-Type"] = "application/json";
      return resp;
    });

    this->addHttpHandler("/log", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
        int num_lines = 0;
        if (req.query.find("lines") != req.query.end()) {
            num_lines = req.query["lines"].toInt();
        }
        
        std::vector<std::string> log = num_lines > 0 ? board->logger.tail(num_lines) : board->logger.all();
        String body = "";
        for (std::string line : log) {
            body += String(line.c_str()) + "\n";
        }

        HttpResponse resp = HttpResponse();
        resp.status = 200;
        resp.body = body;
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });
  
  
    // Retrieve the current "state" and return it
    this->addHttpHandler("/state", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      int drive_state = board->getRobotState()->driveState();
      if (drive_state == 0) {
        resp.body = String(drive_state) + ": IDLE";
      } else if (drive_state == 1) {
        resp.body = String(drive_state) + ": PRIMED";
      } else if (drive_state == 2) {
        resp.body = String(drive_state) + ": DRIVING";
      } else if (drive_state == 20) {
        resp.body = String(drive_state) + ": SETTING BASELINE";
      } else if (drive_state == 21) {
        resp.body = String(drive_state) + ": BASELINING";
      } else {
        resp.body = String(drive_state) + ": OTHER";
      }
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Read the ADC and return the values
    this->addHttpHandler("/adc", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      this->adc->updateAll();
      resp.body = "0: " + String(this->adc->channel(0)->value())
        + "\n1: " + String(this->adc->channel(1)->value())
        + "\n2: " + String(this->adc->channel(2)->value())
        + "\n3: " + String(this->adc->channel(3)->value())
        + "\n4: " + String(this->adc->channel(4)->value())
        + "\n5: " + String(this->adc->channel(5)->value())
        + "\n6: " + String(this->adc->channel(6)->value())
        + "\n7: " + String(this->adc->channel(7)->value());
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Return the current IR values, along with their raw values, baselines + thresholds
    this->addHttpHandler("/ir", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      String body = "";
      for (int i = 0; i < this->num_infrared_sensors; i++) {
        body += "IR" + String(i + 1) + ": " + String(this->irs[i]->value) + " [" + (this->irs[i]->triggered ? "ON" : "OFF") + "]" "\n";
        body += "   -        RAW: " + String(this->irs[i]->raw_value) + "\n";
        body += "   -   Baseline: " + String(this->irs[i]->baseline) + "\n";
        body += "   -  Threshold: " + String(this->irs[i]->threshold) + "\n";
        body += "   - HThreshold: " + String(this->irs[i]->hard_threshold) + "\n\n";
      }
      resp.body = body;
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Return the current pose
    this->addHttpHandler("/pose", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "X: " + String(board->getRobotState()->poseX) + "\n"
        + "Y: " + String(board->getRobotState()->poseY) + "\n"
        + "Z: " + String(board->getRobotState()->poseZ);
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
    
    
    // Simulate a button press
    this->addHttpHandler("/btn-press", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      this->onBtnPressed();
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
    // Simulate a button long press
    this->addHttpHandler("/btn-long-press", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      this->onBtnLongPressed(2000);
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
    // Simulate a button double press
    this->addHttpHandler("/btn-double-press", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      this->onBtnDoublePressed();
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the state of the robot
    this->addHttpHandler("/set-state", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      String response_str = "";
      if (req.query.find("state") != req.query.end()) {
        int state = req.query["state"].toInt();
        this->state->setState(state);
        response_str += "Set State to: " + String(state) + "\n";
      } else {
        response_str += "No state provided - please add a state query param: ?state=<val>\n";
      }
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = response_str;
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the state of the LEDs
    this->addHttpHandler("/set-led", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      String response_str = "";
  
      if (req.query.find("all") != req.query.end()) {
        String key = String("all");
        bool on = req.query[key] == "true" || req.query[key] == "1";
        for (int i = 0; i <= 15; i++) {
          board->setLED(i, on);
        }
        response_str += "All LEDs: " + String(on ? "ON" : "OFF") + "\n";
      }
  
      for (int i = 0; i <= 15; i++) {
        String key = String(i);
        if (req.query.find(key) != req.query.end()) {
          bool on = req.query[key] == "true" || req.query[key] == "1";
          board->setLED(i, on);
          response_str += "LED " + key + ": " + String(on ? "ON" : "OFF") + "\n";
        }
      }
  
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = response_str;
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set motor speed
    this->addHttpHandler("/set-motor-speed", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      String response_str = "";
  
      if (req.query.find("left") != req.query.end()) {
        int speed = req.query["left"].toInt();
        board->setMotorSpeedL(speed);
        response_str += "Left Speed: " + String(speed) + "\n";
      }
  
      if (req.query.find("right") != req.query.end()) {
        int speed = req.query["right"].toInt();
        board->setMotorSpeedR(speed);
        response_str += "Right Speed: " + String(speed) + "\n";
      }
  
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = response_str;
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Start the baselining process
    this->addHttpHandler("/start-baseline", [](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      board->startBaselining();
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok - put the car on the ground and press the button to start baseline process";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the IR Baseline Values
    this->addHttpHandler("/set-ir-baselines", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      if (req.query.find("1") != req.query.end()) {
        this->irs[0]->baseline = req.query["1"].toInt();
      }
      if (req.query.find("2") != req.query.end()) {
        this->irs[1]->baseline = req.query["2"].toInt();
      }
      if (req.query.find("3") != req.query.end()) {
        this->irs[2]->baseline = req.query["3"].toInt();
      }
      if (req.query.find("4") != req.query.end()) {
        this->irs[3]->baseline = req.query["4"].toInt();
      }
      
      HttpResponse resp = HttpResponse();  
      resp.status = 200;
      resp.body = "ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the IR HARD Threshold Values
    this->addHttpHandler("/set-ir-hard-thresholds", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      if (req.query.find("1") != req.query.end()) {
        this->irs[0]->hard_threshold = req.query["1"].toInt();
      }
      if (req.query.find("2") != req.query.end()) {
        this->irs[1]->hard_threshold = req.query["2"].toInt();
      }
      if (req.query.find("3") != req.query.end()) {
        this->irs[2]->hard_threshold = req.query["3"].toInt();
      }
      if (req.query.find("4") != req.query.end()) {
        this->irs[3]->hard_threshold = req.query["4"].toInt();
      }
      
      HttpResponse resp = HttpResponse();  
      resp.status = 200;
      resp.body = "ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the IR SOFT Threshold Values
    this->addHttpHandler("/set-ir-soft-thresholds", [this](WiFiClient* client, HttpRequest &req, RobotBoard* board) {
      if (req.query.find("1") != req.query.end()) {
        this->irs[0]->threshold = req.query["1"].toInt();
      }
      if (req.query.find("2") != req.query.end()) {
        this->irs[1]->threshold = req.query["2"].toInt();
      }
      if (req.query.find("3") != req.query.end()) {
        this->irs[2]->threshold = req.query["3"].toInt();
      }
      if (req.query.find("4") != req.query.end()) {
        this->irs[3]->threshold = req.query["4"].toInt();
      }
      
      HttpResponse resp = HttpResponse();  
      resp.status = 200;
      resp.body = "ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
}

#endif // HUB_LINE_ROBOT_ROUTES_H