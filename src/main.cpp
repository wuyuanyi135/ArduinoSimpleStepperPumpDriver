#include <Arduino.h>
#include "SimpleCLI.h"
#include "Ticker.h"
#include "PropertyNode.h"
#include "SimpleCLIInterface.h"

#define PIN_STEP D2
#define PIN_ENABLE D4
#define PIN_DIR D1

SimpleCLI cli;
String buf;
PropertyNode<double> target_sps("sps", 0.0);
PropertyNode<double> ramp("ramp", 800.0);
PropertyNode<bool> enable("enable", false);

SimpleCLIInterface cliInterface(cli, Serial, true);
bool step_state = false;
double current_sps = 0;
//double target_sps = 0;
//double ramp = 800; // sps per second
double schedule_time = 100;

Ticker ticker([]() {
    step_state = !step_state;
    digitalWrite(PIN_STEP, step_state);
}, 0, 0, MICROS_MICROS);

Ticker ramp_scheduler([]() {
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
        if ((flag > 0 && current_sps >= target_sps.get_value()) || (flag < 0 && current_sps <= target_sps.get_value())) {
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
}, schedule_time, 1, MILLIS);

void setup() {
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);

    Serial.begin(115200);
    target_sps.register_interface(cliInterface);
    enable.register_interface(cliInterface);
    ramp.register_interface(cliInterface);

    target_sps.set_validator([](double v) { return std::abs(v) < 3000; });
    ramp.set_validator([](double v) { return v > 0; });

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
            current_sps = 0;
            ramp_scheduler.start();
            digitalWrite(PIN_ENABLE, LOW);
        }
    });

    Serial.println("Simple Stepper Pump Driver - v0.1");
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

}