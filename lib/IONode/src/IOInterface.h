//
// Created by wuyua on 9/25/2020.
//

#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include <Arduino.h>

typedef std::function<void(std::vector<std::string>)> IOInterfaceCallback;

class IOInterface {
public:
    IOInterface() = default;

public:
    virtual int push_message(const std::string& name, const std::string& label, const std::string& message) = 0;

    virtual int subscribe(std::string identifier, IOInterfaceCallback cb) = 0;

    virtual void unsubscribe(std::string identifier) = 0;
};


#endif //IOINTERFACE_H
