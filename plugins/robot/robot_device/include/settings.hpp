#pragma once
#include <stdint.h>
#include <typeindex>
#include <yaml-cpp/yaml.h>

namespace ercp {
    class SettingManager;
}

class PropertyBase {
    friend class ercp::SettingManager;

public:
    std::string GetName() { return typeid(*this).name(); }

protected:
    PropertyBase(const std::type_info &type)
        : type(type)
    {
    }
    const std::type_info &type;

    static std::vector<PropertyBase *> properties;
};

template <typename T>
class Property : public PropertyBase {
    friend class ercp::SettingManager;

public:
    using value_type = T;

public:
    Property()
        : PropertyBase(typeid(T))
    {
    }
    Property(const T &value)
        : value(value)
        , PropertyBase(typeid(T))
    {
        properties.emplace_back(this);
    }
    ~Property() = default;

    virtual T &operator=(const T &f) { return value = f; }

    const T &operator()() const { return value; }

    explicit operator const T &() const { return value; }

protected:
    T value;
};

template <typename T>
class PropertyReadOnly : public Property<T> {
    friend class ercp::SettingManager;

public:
    PropertyReadOnly() = default;
    PropertyReadOnly(const T &value)
        : Property<T>(value)
    {
    }

protected:
    T &operator=(const T &f) { return this->value = f; }
};
