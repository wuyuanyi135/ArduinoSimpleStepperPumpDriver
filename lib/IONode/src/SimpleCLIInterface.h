//
// Created by wuyua on 9/25/2020.
//

#ifndef SIMPLECLIINTERFACE_H
#define SIMPLECLIINTERFACE_H


#include "IOInterface.h"
#include "SimpleCLI.h"
#include <unordered_map>

class SimpleCLIInterface : public IOInterface {
protected:
    SimpleCLI &cliRef;
    Stream &out;
    bool autoNewLine;
public:
    static std::unordered_map<std::string, IOInterfaceCallback> _callbackMap;

public:
    SimpleCLIInterface(SimpleCLI &cli, Stream &out, bool autoNewLine=true);

public:
    int push_message(const std::string& name, const std::string& label, const std::string& message) override;

    int subscribe(std::string identifier, IOInterfaceCallback cb) override;

    void unsubscribe(std::string identifier) override;
};


#endif //SIMPLECLIINTERFACE_H
