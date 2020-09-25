//
// Created by wuyua on 9/25/2020.
// Helper library to create reusable Serial command communication, MQTT communication, HTTP interface, etc.

#ifndef IONODE_H
#define IONODE_H

#include <Arduino.h>

#include <utility>
#include "IOInterface.h"

class Node {
protected:
    std::string name;
    std::vector<IOInterface*> interfaces;
public:
    /// By default, the name will be used as
    /// \param name
    /// \param access
    explicit Node(std::string name);

    int register_interface(IOInterface &interface);

protected:
    virtual void on_command(std::vector<std::string> &args) = 0;

protected:
    void push_message(const std::string& label, const std::string& message);
};

#endif //IONODE_H
