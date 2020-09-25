//
// Created by wuyua on 9/25/2020.
//
#include <memory>

#include "SimpleCLIInterface.h"

SimpleCLIInterface::SimpleCLIInterface(SimpleCLI &cli, Stream &out, bool autoNewLine) : cliRef(cli), out(out),
                                                                                        autoNewLine(autoNewLine) {

}

int SimpleCLIInterface::push_message(const std::string &name, const std::string &label, const std::string &message) {
    auto str = name + " " + label + ": " + message;
    if (autoNewLine) {
        str += "\n";
    }
    out.print(str.c_str());
    return 1;
}

int SimpleCLIInterface::subscribe(std::string identifier, IOInterfaceCallback cb) {
    if (!cb) return 0;
    _callbackMap[identifier] = cb;
    cliRef.addBoundlessCommand(identifier.c_str(), [](cmd *c) {
        Command cmd(c);
        int nArgs = cmd.countArgs();
        IOInterfaceCallback cb = SimpleCLIInterface::_callbackMap[std::string(cmd.getName().c_str())];
        std::vector<std::string> vec;
        for (int i = 0; i < nArgs; ++i) {
            vec.push_back(cmd.getArg(i).getValue().c_str());
        }
        cb(vec);
    });
    return 1;
}


void SimpleCLIInterface::unsubscribe(std::string identifier) {
    // Not implemented
}

std::unordered_map<std::string, IOInterfaceCallback> SimpleCLIInterface::_callbackMap;