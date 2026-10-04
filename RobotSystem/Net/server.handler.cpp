#include <algorithm>
#include <fmt/format.h>
#include "server.handler.h"
#include "server.interface.h"

using namespace server;

/** 将允许的 JSON 类型列表转换为校验错误中的类型名称。 */
std::string format_types(const type_list &types)
{
    std::string str;
    for (const auto &type : types) {
        if (!str.empty())
            str += " | ";
        str += type.name();
    }
    return str;
}

class basic_request : public request_case {
public:
    /** 校验必填字段和各字段的 JSON 类型。 */
    std::string checker(const Json &json) override
    {
        for (const auto &k : info) {
            if (!k.optional && !json.has(k.name)) {
                return fmt::format("Request member {} is need but not given.", k.name);
            }
            if (json.has(k.name)) {
                auto &type = json.get(k.name).type();
                if (std::find(k.types.begin(), k.types.end(), type) == k.types.end()) {
                    return fmt::format("Request member {} is supposed to be {}, but {} is given.",
                                       k.name,
                                       format_types(k.types),
                                       type.name());
                }
            }
        }
        return "";
    }
};

class robot_info : public basic_request {
public:
    Json handler(const Json &) override
    {
        Json d;
        d.set("name", robot::get_robot_name());
        d.set("version", robot::get_robot_version());
        d.set("location", robot::get_robot_location());
        d.set("modules", robot::get_modules());
        return d;
    }
};

class robot_log : public basic_request {
public:
    Json handler(const Json &) override
    {
        Json d;
        d.set("status", robot::switch_log());
        return d;
    }
};

class robot_force_record : public basic_request {
public:
    Json handler(const Json &) override
    {
        Json d;
        d.set("status", robot::force_record());
        return d;
    }
};

class robot_state : public basic_request {
public:
    /** running 来自实际运行状态；modules 和 step 来自各模块状态机。 */
    Json handler(const Json &) override
    {
        Json d;
        d.set("running", robot::is_robot_running() ? 1 : 0);
        d.set("time", robot::get_time());
        Json modules;
        Json steps;
        for (const auto &module : robot::get_modules()) {
            modules.set(module, robot::get_module_state(module));
            steps.set(module, robot::get_module_step(module));
        }
        d.set("modules", modules);
        d.set("step", steps);
        return d;
    }
};

class robot_action : public basic_request {
public:
    robot_action()
    {
        info.emplace_back(keys_info{"type", {typeid(std::string)}});
        info.emplace_back(keys_info{"action", {typeid(std::string)}});
    }

    /** 校验模块名称和该模块支持的动作。 */
    std::string checker(const Json &json) override
    {
        auto ret = basic_request::checker(json);
        if (!ret.empty())
            return ret;
        const auto type = json.get("type").extract<std::string>();
        const auto modules = robot::get_modules();
        if (std::find(modules.begin(), modules.end(), type) == modules.end())
            return "Action has invalid `type`.";
        const auto action = json.get("action").extract<std::string>();
        const auto actions = robot::get_module_actions();
        if (std::find(actions.begin(), actions.end(), action) == actions.end())
            return "Action has invalid `action`.";
        return "";
    }

    Json handler(const Json &json) override
    {
        Json d;
        const auto type = json.get("type").extract<std::string>();
        const auto action = json.get("action").extract<std::string>();
        const bool ok = robot::do_module_action(type, action);
        d.set("status", ok);
        if (!ok) {
            d.set(kBusinessFailInfoKey,
                  fmt::format("Module action `{}` on `{}` failed.", action, type));
        }
        return d;
    }
};

class robot_init : public basic_request {
public:
    explicit robot_init(bool initialize)
        : initialize(initialize)
    {
    }

    /** 执行初始化或关闭，将设备执行结果传递到 HTTP 响应。 */
    Json handler(const Json &) override
    {
        Json d;
        const bool ok = initialize ? robot::init() : robot::close();
        d.set("status", ok);
        if (!ok) {
            d.set(kBusinessFailInfoKey,
                  fmt::format("Robot `{}` failed.", initialize ? "init" : "close"));
        }
        return d;
    }

private:
    const bool initialize;
};

class robot_emergency_stop : public basic_request {
public:
    robot_emergency_stop()
    {
        info.emplace_back(keys_info{"active", {typeid(bool)}});
    }

    Json handler(const Json &json) override
    {
        Json d;
        const bool active = json.get("active").extract<bool>();
        const bool ok = robot::emergency_stop(active);
        d.set("status", ok);
        if (!ok) {
            d.set(kBusinessFailInfoKey,
                  fmt::format("Emergency stop {} failed.", active ? "assert" : "clear"));
        }
        return d;
    }
};

class settings_base : public basic_request {
public:
    Json handler(const Json &) override
    {
        Json d;
        d.set("data", settings::get_setting_config());
        return d;
    }
};

class settings_data : public basic_request {
public:
    Json handler(const Json &) override
    {
        Json d;
        d.set("data", settings::get_settings());
        return d;
    }
};

class settings_update : public basic_request {
public:
    settings_update()
    {
        info.emplace_back(keys_info{"data", {typeid(std::string)}});
    }

    Json handler(const Json &json) override
    {
        Json d;
        const auto data = json.get("data").extract<std::string>();
        d.set("status", settings::update_settings(data));
        return d;
    }
};

class sensio_base : public basic_request {
public:
    /** 返回传感器和 GPIO 通道编号及名称。 */
    Json handler(const Json &) override
    {
        Json d;
        std::vector<Json> sensors;
        for (const auto id : sensor::get_sensors()) {
            Json sensor;
            sensor.set("channel", id);
            sensor.set("name", server::sensor::get_sensor_name(id));
            sensors.push_back(sensor);
        }
        d.set("sensors", sensors);
        std::vector<Json> inputs;
        for (const auto id : gpio::get_gpio_inputs()) {
            Json input;
            input.set("channel", static_cast<int>(id));
            input.set("name", gpio::get_input_name(id));
            inputs.push_back(input);
        }
        d.set("inputs", inputs);
        std::vector<Json> outputs;
        for (const auto id : gpio::get_gpio_outputs()) {
            Json output;
            output.set("channel", static_cast<int>(id));
            output.set("name", gpio::get_output_name(id));
            outputs.push_back(output);
        }
        d.set("outputs", outputs);
        return d;
    }
};

class beckhoff_isopen : public basic_request {
public:
    std::string checker(const Json &) override
    {
        return beckhoffator::isOpen() ? "" : "The Beckhoff not Open!";
    }

    Json handler(const Json &) override
    {
        Json d;
        d.set("status", beckhoffator::isOpen());
        return d;
    }
};

/** 注册机器人生命周期、模块动作、配置、传感器目录和设备连接状态接口。 */
Handler::Handler()
{
    using vt = decltype(handlers)::mapped_type;
    handlers.emplace("robot",
                     vt{{"info", std::make_shared<robot_info>()},
                        {"log", std::make_shared<robot_log>()},
                        {"status", std::make_shared<robot_state>()},
                        {"action", std::make_shared<robot_action>()},
                        {"init", std::make_shared<robot_init>(true)},
                        {"close", std::make_shared<robot_init>(false)},
                        {"emergency-stop", std::make_shared<robot_emergency_stop>()},
                        {"forcerecord", std::make_shared<robot_force_record>()}});
    handlers.emplace("settings",
                     vt{{"/", std::make_shared<settings_base>()},
                        {"data", std::make_shared<settings_data>()},
                        {"update", std::make_shared<settings_update>()}});
    handlers.emplace("sensio", vt{{"/", std::make_shared<sensio_base>()}});
    handlers.emplace("beckhoff", vt{{"isopen", std::make_shared<beckhoff_isopen>()}});
}
