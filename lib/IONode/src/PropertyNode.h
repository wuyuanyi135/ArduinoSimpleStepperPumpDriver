//
// Created by wuyua on 9/25/2020.
//

#ifndef IOPROPERTYNODE_H
#define IOPROPERTYNODE_H

#include <sstream>
#include "IONode.h"

template<typename T>
class PropertyNode : public Node {
protected:
    T prop;
    std::function<bool(T new_value)> validator;
    std::function<void(T, T)> updateCallback;

public:
    PropertyNode(const std::string &name, T prop) : Node(name),
                                                    prop(prop) {}

    void set_validator(const std::function<bool(T)> &validator);

    void set_update_callback(const std::function<void(T, T)> &cb);

    T &get_value();

protected:
    void on_command(std::vector<std::string> &args) override;

    virtual bool on_validation(T &newValue);

    virtual void on_property_updated(T &oldValue, T &newValue);

    virtual void on_get();

    virtual void on_set(std::string newValue);

    virtual void on_error(std::string error);

private:
    static std::string to_string(T val);

    static T from_string(std::string &str);
};

template<typename T>
void PropertyNode<T>::on_command(std::vector<std::string> &args) {
    unsigned int size = args.size();
    switch (size) {
        case 0:
            on_get();
            break;
        case 1:
            on_set(args[0]);
            break;
        default:
            on_error("Argument number should be 0 or 1\n");
            break;
    }
}

template<typename T>
T &PropertyNode<T>::get_value() {
    return prop;
}

template<typename T>
void PropertyNode<T>::on_get() {
    push_message("GET", to_string(prop));
}

template<typename T>
void PropertyNode<T>::on_set(std::string newValue) {
    T v = from_string(newValue);
    if (on_validation(v)) {
        T oldValue = prop;
        prop = v;
        on_property_updated(oldValue, v);
    } else {
        on_error("Validation failed");
    }
}

template<typename T>
void PropertyNode<T>::on_error(std::string error) {
    push_message("ERROR", error);
}

template<typename T>
void PropertyNode<T>::on_property_updated(T &oldValue, T &newValue) {
    if (updateCallback) {
        updateCallback(oldValue, newValue);
    } else {
        push_message("SET", to_string(newValue));
    }
}

template<typename T>
bool PropertyNode<T>::on_validation(T &newValue) {
    if (validator) {
        return validator(newValue);
    } else {
        return true;
    }
}

template<typename T>
void PropertyNode<T>::set_update_callback(const std::function<void(T, T)> &cb) {
    updateCallback = cb;
}

template<typename T>
void PropertyNode<T>::set_validator(const std::function<bool(T)> &validator) {
    this->validator = validator;
}

template<typename T>
std::string PropertyNode<T>::to_string(T value) {
    std::stringstream convert;
    convert << value;
    return convert.str();
}

template<typename T>
T PropertyNode<T>::from_string(std::string &str) {
    std::stringstream convert(str);
    T value;
    convert >> value;
    return value;
}

#endif //IOPROPERTYNODE_H
