//
// Created by wuyua on 9/25/2020.
//
#include "IONode.h"

Node::Node(std::string name) : name(std::move(name)) {

}

int Node::register_interface(IOInterface &interface) {
    interface.subscribe(name, [&](std::vector<std::string> args) {
        on_command(args);
    });
    interfaces.push_back(&interface);
}

void Node::push_message(const std::string &label, const std::string &message) {
    for (auto &interface : interfaces) {
        interface->push_message(name, label, message);
    }
}
