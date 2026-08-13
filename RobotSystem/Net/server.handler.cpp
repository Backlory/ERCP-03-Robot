#include <Poco/Net/HTTPServerResponse.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/NumberParser.h>
#include <fmt/format.h>
#include "server.handler.h"
#include "server.interface.h"

#include <Poco/Timestamp.h>

using namespace server;
using poco_array = Poco::SharedPtr<Poco::JSON::Array>;

/**
 * @brief 将允许的 JSON 类型列表格式化为错误消息中的可读文本。
 * @details 按登记顺序连接类型名称，并使用竖线表示多个可选类型。
 */
std::string format_types(const type_list &types)
{
    std::string str;
    int i = 0;
    for (auto t = types.begin(); t != types.end(); ++t) {
        str += t->name();
        if (i < types.size() - 1)
            str += " | ";
        ++i;
    }
    return str;
}

//-----------------------------------------------------------------------------

class basic_request : public request_case {
public:
    /**
     * @brief 校验通用请求字段是否存在且类型正确。
     * @details 逐项检查必填字段和可选字段的类型，返回空字符串表示通过，否则返回面向客户端的错误原因。
     */
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

    Json handler(const Json &json) override { return Json(); }
};

//-----------------------------------------------------------------------------

class robot_info : public basic_request {
public:
    /**
     * @brief 生成机器人当前状态及各模块进度的 JSON 响应。
     * @details 先映射机器人运行级状态，再分别收集模块状态和步骤名称，保持响应结构与旧接口一致。
     */
    Json handler(const Json &json) override
    {
        Json d;
        d.set("name", robot::get_robot_name());
        d.set("version", robot::get_robot_version());
        d.set("location", robot::get_robot_location());
        d.set("modules", robot::get_modules());
        //d.set("devices", robot::get_devices());
        return d;
    }
};

class robot_log : public basic_request {
public:
    Json handler(const Json &json) override
    {
        Json d;
        d.set("status", robot::switch_log());
        return d;
    }
};

class robot_force_record : public basic_request {
public:
    Json handler(const Json &json) override
    {
        Json d;
        d.set("status", robot::force_record());
        return d;
    }
};

class robot_state : public basic_request {
public:
    /**
     * @brief 生成机器人运行状态、模块状态和当前步骤的 JSON 响应。
     * @details 先将机器人级状态映射为 running，再分别遍历模块注册表填充 modules 与 step 两组信息。
     */
    Json handler(const Json &json) override
    {
        Json d;
        if (robot::is_robot_errored()) {
            d.set("running", 2);
        } else if (robot::is_robot_running()) {
            d.set("running", 1);
        } else if (robot::is_robot_ready()) {
            d.set("running", 0);
        }
        d.set("time", robot::get_time());

        //{
        //    Json x;
        //    for (auto m : robot::get_devices()) {
        //        x.set(m, robot::get_device_state(m));
        //    }
        //    d.set("devices", x);
        //}
        {
            Json x;
            for (auto m : robot::get_modules()) {
                x.set(m, robot::get_module_state(m));
            }
            d.set("modules", x);
        }
        {
            Json x;
            for (auto m : robot::get_modules()) {
                x.set(m, robot::get_module_step(m));
            }
            d.set("step", x);
        }
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

    /**
     * @brief 校验模块动作请求中的模块名和动作名。
     * @details 先复用通用字段校验，再分别对 type 和 action 与当前注册表比对。
     */
    std::string checker(const Json &json) override
    {
        auto ret = basic_request::checker(json);
        if (!ret.empty()) {
            return ret;
        }

        {
            auto type = json.get("type").extract<std::string>();
            auto m = robot::get_modules();
            if (std::find(m.begin(), m.end(), type) == m.end()) {
                return "Action has invalid `type`.";
            }
        }

        {
            auto action = json.get("action").extract<std::string>();
            auto m = robot::get_module_actions();
            if (std::find(m.begin(), m.end(), action) == m.end()) {
                // L3: 该分支校验的是 `action` 字段,文案不再误写为 `type`
                return "Action has invalid `action`.";
            }
        }
        return "";
    }

    /**
     * @brief 将模块动作请求分派到 Robot 接口并封装业务结果。
     * @details 提取模块和动作字符串，调用统一动作入口；失败时额外写入业务错误信息供 HTTP 信封转换。
     */
    Json handler(const Json &json) override
    {
        Json d;
        auto type = json.get("type").extract<std::string>();
        auto action = json.get("action").extract<std::string>();
        const bool ok = robot::do_module_action(type, action);
        d.set("status", ok);
        if (!ok) {
            // M1: 业务失败上提为信封 status=false(见 server.cpp handleRequest)
            d.set(kBusinessFailInfoKey,
                  fmt::format("Module action `{}` on `{}` failed.", action, type));
        }
        return d;
    }
};

class robot_init : public basic_request {
public:
    robot_init(std::string action)
        : action(action)
    {
    }

    /**
     * @brief 执行机器人生命周期动作并返回成功状态。
     * @details 按 action 选择启动、关闭、初始化、中断或跳过接口，统一把结果放入 status 字段。
     */
    Json handler(const Json &json) override
    {
        Json d;

        bool ret = false;
        if (action == "start") {
            ret = robot::start();
        } else if (action == "close") {
            ret = robot::close();
        } else if (action == "init") {
            ret = robot::init();
        } else if (action == "interrupt") {
            ret = robot::interrupt();
        } else if (action == "skip") {
            ret = robot::skip();
        }
        d.set("status", ret);
        if (!ret) {
            // M1: 业务失败上提为信封 status=false(见 server.cpp handleRequest)
            d.set(kBusinessFailInfoKey, fmt::format("Robot `{}` failed.", action));
        }
        return d;
    }

public:
    const std::string action;
};

//-----------------------------------------------------------------------------

class settings_base : public basic_request {
public:
    Json handler(const Json &json) override
    {
        // Read yaml data into json
        Json d;
        d.set("data", settings::get_setting_config());
        return d;
    }
};

class settings_data : public basic_request {
public:
    Json handler(const Json &json) override
    {
        // Read yaml data into json
        Json d;
        d.set("data", settings::get_settings());
        return d;
    }
};

class settings_update : public basic_request {
public:
    settings_update()
    {
        //
        info.emplace_back(keys_info{"data", {typeid(std::string)}});
    }

    Json handler(const Json &json) override
    {
        Json d;
        auto data = json.get("data").extract<std::string>();
        d.set("status", settings::update_settings(data));
        return d;
    }
};

//-----------------------------------------------------------------------------

class sensio_base : public basic_request {
public:
    /**
     * @brief 返回传感器、GPIO 输入和 GPIO 输出的通道目录。
     * @details 分别遍历三类注册表，构造包含 channel 与 name 的 JSON 数组供客户端发现设备能力。
     */
    Json handler(const Json &json) override
    {
        Json d;
        {
            std::vector<Poco::JSON::Object> array;
            for (auto sid : sensor::get_sensors()) {
                Poco::JSON::Object s;
                s.set("channel", (int)sid);
                s.set("name", sensor::get_sensor_name(sid));
                array.push_back(s);
            }
            d.set("sensors", array);
        }
        {
            std::vector<Poco::JSON::Object> array;
            for (auto gid : gpio::get_gpio_inputs()) {
                Poco::JSON::Object s;
                s.set("channel", (int)gid);
                s.set("name", gpio::get_input_name(gid));
                array.push_back(s);
            }
            d.set("inputs", array);
        }
        {
            std::vector<Poco::JSON::Object> array;
            for (auto gid : gpio::get_gpio_outputs()) {
                Poco::JSON::Object s;
                s.set("channel", (int)gid);
                s.set("name", gpio::get_output_name(gid));
                array.push_back(s);
            }
            d.set("outputs", array);
        }
        return d;
    }
};

class sensio_action : public basic_request {
public:
    sensio_action()
    {
        info.emplace_back(keys_info{"id", {typeid(int64_t)}});
        info.emplace_back(keys_info{"value", {typeid(int64_t)}});
    }

    /**
     * @brief 校验 GPIO 输出操作中的通道编号。
     * @details 先执行通用字段校验，再确认请求编号存在于当前输出注册表。
     */
    std::string checker(const Json &json) override
    {
        auto ret = basic_request::checker(json);
        if (!ret.empty()) {
            return ret;
        }

        if (json.has("id")) {
            auto ops = gpio::get_gpio_outputs();
            auto id = (gpio_output_t)(json.get("id").extract<int64_t>());
            if (std::find(ops.begin(), ops.end(), id) == ops.end()) {
                return "Invalid GPIO output ID.";
            }
        }
        return "";
    }

    Json handler(const Json &json) override
    {
        Json d;
        int64_t value = json.get("value").extract<int64_t>();
        auto gid = (gpio_output_t)(json.get("id").extract<int64_t>());
        d.set("id", (int)gid);
        d.set("status", gpio::set_output_state(gid, value > 0));
        return d;
    }
};

//-----------------------------------------------------------------------------

class arm_status : public basic_request {
public:
    /**
     * @brief 读取机械臂位姿、关节、速度和目标等状态并生成 JSON 响应。
     * @details 每类数据只有在底层读取成功时才写入，同时补充初始化、停止和到位标志。
     */
    Json handler(const Json &json) override
    {
        Json d;

        Sophus::Vector6d data;
        if (manuplator::get_arm_pose(data)) {
            std::vector<double> v(data.data(), data.data() + 6);
            d.set("cart", v);
        }
        if (manuplator::get_arm_joints(data)) {
            std::vector<double> v(data.data(), data.data() + 6);
            d.set("pos", v);
        }
        if (manuplator::get_arm_velocity(data)) {
            std::vector<double> v(data.data(), data.data() + 6);
            d.set("vel", v);
        }
        if (manuplator::get_arm_target(data)) {
            std::vector<double> v(data.data(), data.data() + 6);
            d.set("target", v);
        }

        d.set("init", manuplator::is_arm_inited());
        d.set("stopped", manuplator::is_arm_stopped());
        d.set("arrived", manuplator::is_arm_arrived());
        return d;
    }
};

class arm_moverel : public basic_request {
public:
    using poco_array = Poco::SharedPtr<Poco::JSON::Array>;

    arm_moverel()
    {
        info.emplace_back(keys_info{"dpose", {typeid(poco_array)}});
        info.emplace_back(keys_info{"tcp", {typeid(bool)}, true});
    }

    /**
     * @brief 校验机械臂相对运动请求的位移数组。
     * @details 复用通用字段检查后验证数组长度和每个元素均为数值，确保构造 Vector6d 前输入完整。
     */
    std::string checker(const Json &json) override
    {
        auto ret = basic_request::checker(json);
        if (!ret.empty()) {
            return ret;
        }

        auto targets = *json.get("dpose").extract<poco_array>();
        if (!targets.size() == 6) {
            return "The param `dpose` should have 6 items.";
        }
        for (auto j : targets) {
            if (!j.isNumeric()) {
                return "The param `dpose` should be array of number.";
            }
        }
        return "";
    }

    /**
     * @brief 将 JSON 位移数组转换为机械臂相对运动命令并执行。
     * @details 读取 dpose 与 tcp 选项，转换为六维向量后调用操纵器接口返回执行状态。
     */
    Json handler(const Json &json) override
    {
        auto targets = *json.get("dpose").extract<poco_array>();
        std::vector<double> v;
        for (auto j : targets) {
            v.push_back(j.convert<double>());
        }

        bool tcp = json.has("tcp") ? json.get("tcp").extract<bool>() : true;

        Json d;
        Sophus::Vector6d dpose(v.data());
        d.set("status", manuplator::move_arm_rel(dpose, !tcp));
        return d;
    }
};

class arm_init : public basic_request {
public:
    Json handler(const Json &json) override
    {
        Json d;
        d.set("status", manuplator::init_arm());
        return d;
    }
};

class arm_stop : public basic_request {
public:
    Json handler(const Json &json) override
    {
        Json d;
        d.set("status", manuplator::stop_arm());
        return d;
    }
};
//-----------------------------------------------------------------------------
// 倍福读取
class beckhoff_read : public basic_request {

private:
public:
    /**
     * @brief 检查 Beckhoff 设备是否已打开，作为旧读取接口的前置条件。
     * @details 当前接口保留历史上可选字段校验的占位代码，只执行连接状态检查并返回诊断文本。
     */
    std::string checker(const Json &json) override
    {
        if (!beckhoffator::isOpen()) {
            return "The Beckhoff not Open!";
        }
        //auto ret = basic_request::checker(json);
        //if (!ret.empty()) {
        //    return ret;
        //}

        //if (json.has("type")) {
        //    auto type = json.get("type").extract<std::string>();
        //    if (valid_types.find(type) == valid_types.end()) {
        //        return "Invalid type.";
        //    }
        //}
        return "";
    }

    /**
     * @brief 保留 Beckhoff 读取接口的 JSON 响应入口。
     * @details 当前实现只返回空对象，实际字段读取逻辑仍由注释中的旧接口扩展点描述。
     */
    Json handler(const Json &json) override
    {
        Json d;
        //if (json.has("type")) {
        //    auto type = json.get("type").extract<std::string>();
        //    d.set("data", valid_types.at(type)());
        //}
        //else {
        //    for (auto t : valid_types) {
        //        d.set(t.first, t.second());
        //    }
        //}
        //        d.set("status", beckhoffator::read());

        //d.set("status", beckhoffator::read());
        return d;
    }
};

// 倍福写入

class beckhoff_write : public basic_request {

private:
public:
    /**
     * @brief 检查 Beckhoff 设备是否已打开，作为旧写入接口的前置条件。
     * @details 通过统一的设备打开状态阻止离线写入，其他历史参数校验暂未启用。
     */
    std::string checker(const Json &json) override
    {
        if (!beckhoffator::isOpen()) {
            return "The Beckhoff not Open!";
        }
        //auto ret = basic_request::checker(json);
        //if (!ret.empty()) {
        //    return ret;
        //}

        //if (json.has("type")) {
        //    auto type = json.get("type").extract<std::string>();
        //    if (valid_types.find(type) == valid_types.end()) {
        //        return "Invalid type.";
        //    }
        //}
        return "";
    }

    /**
     * @brief 保留 Beckhoff 写入接口的 JSON 响应入口。
     * @details 当前实现不执行具体写入，只返回空对象；新增写入动作应在此处接入明确的字段校验和设备接口。
     */
    Json handler(const Json &json) override
    {
        Json d;
        //if (json.has("type")) {
        //    auto type = json.get("type").extract<std::string>();
        //    d.set("data", valid_types.at(type)());
        //}
        //else {
        //    for (auto t : valid_types) {
        //        d.set(t.first, t.second());
        //    }
        //}
        //d.set("status", beckhoffator::write());
        return d;
    }
};

class beckhoff_isopen : public basic_request {

private:
public:
    /**
     * @brief 检查 Beckhoff 设备连接状态。
     * @details 设备未打开时返回前置错误；打开时由 handler 返回 status，保持旧 HTTP 响应结构。
     */
    std::string checker(const Json &json) override
    {
        if (!beckhoffator::isOpen()) {
            return "The Beckhoff not Open!";
        }
        //auto ret = basic_request::checker(json);
        //if (!ret.empty()) {
        //    return ret;
        //}

        //if (json.has("type")) {
        //    auto type = json.get("type").extract<std::string>();
        //    if (valid_types.find(type) == valid_types.end()) {
        //        return "Invalid type.";
        //    }
        //}
        return "";
    }

    /**
     * @brief 将 Beckhoff 当前连接状态封装为 JSON status 字段。
     * @details 读取统一设备接口的 isOpen 结果，不直接暴露 ADS 底层细节。
     */
    Json handler(const Json &json) override
    {
        Json d;
        //if (json.has("type")) {
        //    auto type = json.get("type").extract<std::string>();
        //    d.set("data", valid_types.at(type)());
        //}
        //else {
        //    for (auto t : valid_types) {
        //        d.set(t.first, t.second());
        //    }
        //}
        d.set("status", beckhoffator::isOpen());
        return d;
    }
};
//-----------------------------------------------------------------------------

/**
 * @brief 注册 RobotSystem HTTP 处理器。
 * @details 按 robot、settings、sensio、beckhoff 和 arm 领域建立路径到 request_case 的映射，构造后供服务器分派请求。
 */
Handler::Handler()
{
    using vt = decltype(handlers)::mapped_type;

    handlers.emplace("robot",
                     vt{
                         {"info", std::make_shared<robot_info>()}, //
                         {"log", std::make_shared<robot_log>()}, //
                         {"status", std::make_shared<robot_state>()}, //
                         {"action", std::make_shared<robot_action>()}, //
                         {"init", std::make_shared<robot_init>("init")}, //
                         {"start", std::make_shared<robot_init>("start")}, //
                         {"close", std::make_shared<robot_init>("close")}, //
                         {"interrupt", std::make_shared<robot_init>("interrupt")}, //
                         {"skip", std::make_shared<robot_init>("skip")}, //
                         {"forcerecord", std::make_shared<robot_force_record>()}, //
                     });
    handlers.emplace("settings",
                     vt{
                         {"/", std::make_shared<settings_base>()}, //
                         {"data", std::make_shared<settings_data>()}, //
                         {"update", std::make_shared<settings_update>()}, //
                     });
    handlers.emplace(
        "motor",
        vt{
            //{ "/", std::make_shared<motor_base>() }, //
            //{ "status", std::make_shared<motor_status>() }, //
            //{ "enable", std::make_shared<motor_action>(motor_action::type::enable) }, //
            //{ "disable", std::make_shared<motor_action>(motor_action::type::disable) }, //
            //{ "action", std::make_shared<motor_action>(motor_action::type::move) }, //
            //{ "clear_faults", std::make_shared<motor_action>(motor_action::type::clear_faults) } //
        });
    handlers.emplace("sensio",
                     vt{
                         {"/", std::make_shared<sensio_base>()}, //
                         //{ "status", std::make_shared<sensio_status>() }, //
                         {"action", std::make_shared<sensio_action>()} //
                     });
    handlers.emplace("beckhoff",
                     vt{{"read", std::make_shared<beckhoff_read>()},
                        {"write", std::make_shared<beckhoff_write>()},
                        {"follow", std::make_shared<beckhoff_read>()},
                        {"isopen", std::make_shared<beckhoff_isopen>()}});
    handlers.emplace("arm",
                     vt{
                         {"status", std::make_shared<arm_status>()}, //
                         {"moverel", std::make_shared<arm_moverel>()}, //
                         {"init", std::make_shared<arm_init>()}, //
                         {"stop", std::make_shared<arm_stop>()} //
                     });
}
