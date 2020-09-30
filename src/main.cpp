#include "ESP8266WiFi.h"
#include "PropertyNode.h"
#include "PubSubClientInterface.h"
#include "SimpleCLI.h"
#include "SimpleCLIInterface.h"
#include "Ticker.h"
#include <Arduino.h>
#include <sstream>


#define PIN_STEP D2
#define PIN_ENABLE D4
#define PIN_DIR D1

SimpleCLI cli;
WiFiClient espClient;
PubSubClient client(espClient);

String buf;
PropertyNode<double> target_sps("sps", 0.0);
PropertyNode<double> ramp("ramp", 800.0);
PropertyNode<bool> enable("enable", false);

SimpleCLIInterface cliInterface(cli, Serial, true);
PubSubClientInterface mqttInterface(client);

bool step_state = false;
double current_sps = 0;
//double target_sps = 0;
//double ramp = 800; // sps per second
double schedule_time = 100;

Ticker ticker(
        []() {
            step_state = !step_state;
            digitalWrite(PIN_STEP, step_state);
        },
        0, 0, MICROS_MICROS
);

Ticker ramp_scheduler(
        []() {
            if (target_sps.get_value() == current_sps) {
                Serial.println("Schedule ends");
                ramp_scheduler.stop();
            }
            if (current_sps != 0 && (std::signbit(target_sps.get_value()) ^ std::signbit(current_sps))) {
                // dir changed. to zero and re-accelerate.
                current_sps = 0;
                ramp_scheduler.start();
            } else if (std::abs(target_sps.get_value()) > std::abs(current_sps)) {
                // accelerate
                int flag = (target_sps.get_value() - current_sps > 0 ? 1 : -1);
                current_sps += flag * ramp.get_value() * schedule_time / 1000;
                if ((flag > 0 && current_sps >= target_sps.get_value()) ||
                    (flag < 0 && current_sps <= target_sps.get_value())) {
                    current_sps = target_sps.get_value();
                    ramp_scheduler.stop();
                } else {
                    ramp_scheduler.start();
                }
            } else {
                current_sps = target_sps.get_value();
                ramp_scheduler.stop();
            }

            digitalWrite(PIN_DIR, current_sps > 0);

            if (current_sps != 0) {
                ticker.interval(1e6 / std::abs(current_sps));
            }

            if (target_sps.get_value() == 0 && current_sps == 0) {
                ticker.stop();
            }
        },
        schedule_time, 1, MILLIS
);

void setup() {
    Serial.begin(115200);
    Serial.println("Simple Stepper Pump Driver - v0.1");

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin("DCHost", "dchost000000");
    int cnt = 200;
    while (!WiFi.isConnected() && cnt-- > 0) {
        Serial.println("WiFi connecting...");
        delay(200);
    }
    if (cnt > 0) {
        Serial.print("WiFi connected: ");
        Serial.println(WiFi.localIP().toString());

        client.setServer(WiFi.gatewayIP(), 1883);
        std::stringstream convert;
        convert << ESP.getChipId();
        client.connect(convert.str().c_str(), (convert.str() + "/status").c_str(), 0, true, "off");
        cnt = 200;
        while (!client.connected() && cnt-- > 0) {
            Serial.println("Connecting to MQTT server");
            delay(200);
        }
        if (cnt > 0) {
            Serial.println("Connected to MQTT server, configuring subscriptions");
            client.publish((convert.str() + "/desc").c_str(), "Arduino Stepper Pump Driver", true);
            client.publish((convert.str() + "/status").c_str(), "on", true);
            target_sps.register_interface(mqttInterface);
            enable.register_interface(mqttInterface);
            ramp.register_interface(mqttInterface);
        } else {
            Serial.println("Failed to connect to MQTT server.");
        }

    } else {
        Serial.println("WiFi not connected. MQTT disabled.");
    }


    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);

    target_sps.register_interface(cliInterface);
    enable.register_interface(cliInterface);
    ramp.register_interface(cliInterface);

    target_sps.set_validator([](double v) {
        return std::abs(v) < 3000;
    });
    ramp.set_validator([](double v) {
        return v > 0;
    });

    target_sps.set_update_callback([](double oldVal, double newVal) {
        Serial.print("Setting sps to ");
        Serial.println(newVal);
        ramp_scheduler.start();
        ticker.start();
    });
    enable.set_update_callback([](bool oldVal, bool newVal) {
        if (!newVal) {
            Serial.println("Disable output");
            digitalWrite(PIN_ENABLE, HIGH);
        } else {
            Serial.println("Enable output");
            // Already enabled. No need to re-ramp.
            if (enable.get_value()) { return; }
            current_sps = 0;
            ramp_scheduler.start();

            digitalWrite(PIN_ENABLE, LOW);
        }
    });
}

void loop() {
    while (Serial.available()) {
        // Read out string from the serial monitor
        char ch = (char) Serial.read();
        buf += ch;
        // Parse the user input into the CLI
        if (ch == '\n') {
            cli.parse(buf);
            buf.clear();
        }
    }
    ticker.update();
    ramp_scheduler.update();
    client.loop();
}