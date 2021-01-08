#include "ESP8266WiFi.h"
#include "PropertyNode.h"
#include "PubSubClientInterface.h"
#include "SimpleCLI.h"
#include "SimpleCLIInterface.h"
#include "Ticker.h"
#include <Arduino.h>
#include <sstream>
#include "ESP8266Init.h"
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>


#define PIN_STEP D2
#define PIN_ENABLE D4
#define PIN_DIR D1

ESP8266Init esp8266Init{"DCHost", "dchost000000", "192.168.43.1", 1883,
                        "Simple stepper driver v1.1"};

SimpleCLI cli;
String buf;
PropertyNode<double> target_sps("sps", 0.0, false, true);
PropertyNode<double> ramp("ramp", 800.0, false, true);

//PropertyNode<bool> enable("enable", false);
SimpleCLIInterface cliInterface(cli, Serial, true);

PubSubClientInterface mqttInterface(esp8266Init.client);
bool step_state = false;
double current_sps = 0;
//double target_sps = 0;
//double ramp = 800; // sps per second
bool enabled = false;
double schedule_time = 100;
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

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

void set_enable(bool newVal) {
    if (!newVal) {
        Serial.println("Disable output");
        digitalWrite(PIN_ENABLE, HIGH);
        enabled = false;
    } else {
        Serial.println("Enable output");
        // Already enabled. No need to re-ramp.
        if (enabled) { return; }
        current_sps = 0;
        ramp_scheduler.start();

        digitalWrite(PIN_ENABLE, LOW);
        enabled = true;
    }

}

void configure_ota() {
    std::stringstream ss;
    ss << ESP.getChipId();

    MDNS.begin((("update_" + ss.str()).c_str()));
    httpUpdater.setup(&httpServer);
    httpServer.begin();
    MDNS.addService("http", "tcp", 80);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Simple Stepper Pump Driver - v0.1");

    // write your initialization code here
    if (esp8266Init.blocking_init() != ESP8266Init::FINISHED) {
        delay(1000);
        ESP.restart();
    }
    target_sps.register_interface(mqttInterface);
//    enable.register_interface(mqttInterface);
    ramp.register_interface(mqttInterface);
    configure_ota();

    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);

    target_sps.register_interface(cliInterface);
//    enable.register_interface(cliInterface);
    ramp.register_interface(cliInterface);

    target_sps.set_validator([](double v) {
        return std::abs(v) < 3000;
    });
    ramp.set_validator([](double v) {
        return v > 0;
    });

    target_sps.set_update_callback([](double oldVal, double newVal) {
        if (newVal == 0) {
            Serial.print("Disabling due to sps==0");
            set_enable(false);
        } else {
            if (!enabled) {
                Serial.print("Enabling due to sps != 0");
                set_enable(true);
            }
        }
        Serial.print("Setting sps to ");
        Serial.println(newVal);
        ramp_scheduler.start();
        ticker.start();
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
    esp8266Init.client.loop();
    httpServer.handleClient();
    MDNS.update();
}