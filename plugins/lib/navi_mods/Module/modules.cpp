#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include "modules.h"
#include "robot_config.h"

namespace fs = boost::filesystem;

namespace navi {

    Port::Port(std::string name, Module *parent)
        : name(name)
        , parent(parent, [](Module *) {})
    {
    }

    Port::~Port() {}

    PortInfo Port::operator()() const
    {
        return PortInfo{ GetType(), GetSource(), GetScalarType() };
    }

    bool PortInfo::operator==(const PortInfo &p)
    {
        return type == p.type && source == p.source && scalar_type == p.scalar_type;
    }

    bool Port::operator==(const Port &p) { return (*this)() == p(); }

    bool Port::operator!=(const Port &p) { return !((*this)() == p()); }

    ModPtr const Port::GetModule() const { return parent; }

    void Port::UpdateTime(double t)
    {
        time_ex.store(time);
        time = t;
    }

    double Port::GetFrequency() const
    {
        if (!parent || !parent->IsRunning()) {
            return 0;
        }
        if (time_ex == time || time < 0 || time_ex < 0) {
            return 0;
        }
        return 1.0 / (time - time_ex);
    }

    //---------------------------------------------------

    void Port::RegisterConnected(std::function<void(IOPortPtr, IOPortPtr)> fn)
    {
        auto p = shared_from_this();
        if (m_conns.find(p) == m_conns.end()) {
            auto conn = OnConnected.connect(fn);
            m_conns.emplace(p, conn);
        }
    }

    void Port::RegisterDisconnected(std::function<void(IOPortPtr, IOPortPtr)> fn)
    {
        auto p = shared_from_this();
        if (m_disconns.find(p) == m_disconns.end()) {
            auto conn = OnDisconnected.connect(fn);
            m_disconns.emplace(p, conn);
        }
    }

    //---------------------------------------------------

    _InputPort::_InputPort(std::string name, Module *parent)
        : Port(name, parent)
    {
    }

    bool _InputPort::IsInput() const { return true; }

    bool _InputPort::IsOutput() const { return false; }

    //---------------------------------------------------

    _OutputPort::_OutputPort(std::string name, Module *parent)
        : Port(name, parent)
    {
    }

    _OutputPort::~_OutputPort()
    {
        std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
        OnData.disconnect_all_slots();
    }

    bool _OutputPort::IsInput() const { return false; }

    bool _OutputPort::IsOutput() const { return true; }

    boost::signals2::connection _OutputPort::SetSlot(boost::function<void(OPortPtr)> &&fn)
    {
        std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
        return OnData.connect(fn);
    }

    void _OutputPort::Disconnect(const boost::signals2::connection &conn)
    {
        std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
        conn.disconnect();
    }

    //---------------------------------------------------

    Module::Module(std::string name, bool dll, std::string alias, int threads, int verbose)
        : name(name)
        , dll(dll)
        , alias(alias.empty() ? name : alias)
        , verbose(verbose)
    {
        if (dll) {
#if USING_LOGURU
            // Add logger
            boost::algorithm::to_lower(name);
            fs::path log = fs::current_path() / "log" / name / (ilsr::Time::logtime() + ".log");
            if (!fs::exists(log.parent_path())) {
                fs::create_directories(log.parent_path());
            }
            loguru::add_file(log.string().c_str(), loguru::Append, loguru::Verbosity_MAX);
#endif
        }

        ROBOT_INFO(verbose > 0, fmt::format("[{}] constructing ...", name))
        work = std::make_shared<boost::asio::io_service::work>(this->io_service);

        if (threads <= 0) {
            ROBOT_INFO(true,
                fmt::format(
                    "NOTE: module `{}` disabled IO service, may cause module not working.", name))
        } else {
            auto worker = [this]() {
                try {
                    boost::system::error_code ec;
                    io_service.run(ec);
                } catch (std::exception e) {
                    std::cerr << "IO context: " << e.what() << std::endl;
                }
            };
            workers = std::make_shared<boost::thread_group>();
            for (int i = 0; i < threads; ++i) {
                workers->create_thread(worker);
            }
        }
    }

    Module::~Module()
    {
        work.reset();
        io_service.reset();
        // TODO: 切断全部的连接
        for (auto &i : input_ports) {
            i.second.reset();
        }
        input_ports.clear();
        for (auto &o : output_ports) {
            o.second.reset();
        }
        output_ports.clear();
        ROBOT_INFO(verbose > 0, fmt::format("[{}] deconstructed.", name))
    }

    bool Module::IsDll() const { return dll; }

    // void Module::_OnData(_InputPort &p)
    //{
    //    // TODO: 做到可以转换类型
    //    if (!IsRunning()) {
    //        return;
    //    }
    //    for (auto &iport : this->GetInputPorts()) {
    //        // 从其他模块的输出端口拷贝到本模块的输入端口
    //        IPortPtr port = this->GetInputPort(iport);
    //        if (p == (*port)) {
    //            p.Copy(port, 0.2);
    //        }
    //    }
    //}

    // void Module::_OnData(_OutputPort &p)
    //{
    //    // TODO: 做到可以转换类型
    //    if (!IsRunning()) {
    //        return;
    //    }
    //    for (auto &iport : this->GetInputPorts()) {
    //        // 从其他模块的输出端口拷贝到本模块的输入端口
    //        // FIXME:
    //        IPortPtr port = this->GetInputPort(iport);
    //        if (p == (*port)) {
    //            // 将数据备份到输入端口
    //            p.Copy(port, 0.2);
    //            // 触发模块的处理行为
    //            port->Caller();
    //        }
    //    }
    //}

    void Module::_OnData(_OutputPort &src, _InputPort &dst)
    {
        if (!IsRunning()) {
            return;
        }
        for (auto &iport : this->GetInputPorts()) {
            IPortPtr port = this->GetInputPort(iport);
            if (dst.GetName() == port->GetName() && dst == *port) {
                // 将数据备份到输入端口
                src.Copy(port, 0.2);
                // 触发模块的处理行为
                port->Caller();
            }
        }
    }

    void Module::OnData(OPortPtr src, IPortPtr dst)
    {
        using task_t = boost::packaged_task<void>;
        io_service.post(
            boost::bind(&task_t::operator(), boost::make_shared<task_t>([this, src, dst]() {
                if (src->IsOutput(), dst->IsInput()) {
                    this->_OnData(*(_OutputPort *)src.get(), *(_InputPort *)dst.get());
                }
            })));
    }

    std::string Module::GetName() const { return name; }

    std::string Module::GetReadableName() const { return alias; }

    int Module::GetVerbose() const { return verbose; }

    void Module::SetVerbose(int verbose) { this->verbose = verbose; }

    bool Module::IsConverter() const
    {
        auto p1 = GetInputPorts();
        auto p2 = GetOutputPorts();

        if (p1.size() != p2.size()) {
            return false;
        }
        for (auto p = p1.begin(); p != p1.end(); ++p) {
            auto ptr1 = GetInputPort(*p);
            auto q = p2.begin();
            for (; q != p2.end(); ++q) {
                auto ptr2 = GetInputPort(*q);
                // 找到了一对相同的接口
                if (ptr1->GetType() == ptr2->GetType()) {
                }
                break;
            }
            if (q != p2.end()) {
                p2.erase(q);
            }
        }
        return p2.empty();
    }

    bool Module::HasPort(const IOPortPtr &port) const
    {
        if (port->IsInput()) {
            for (auto &p : input_ports) {
                if (p.second.get() == port.get()) {
                    return true;
                }
            }
        } else if (port->IsOutput()) {
            for (auto &p : output_ports) {
                if (p.second.get() == port.get()) {
                    return true;
                }
            }
        }
        return false;
    }

    IPortPtr Module::GetInputPort(const std::string &name) const
    {
        if (input_ports.find(name) != input_ports.end()) {
            return input_ports.at(name);
        }
        return nullptr;
    }

    OPortPtr Module::GetOutputPort(const std::string &name) const
    {
        if (output_ports.find(name) != output_ports.end()) {
            return output_ports.at(name);
        }
        return nullptr;
    }

    std::vector<std::string> Module::GetInputPorts() const
    {
        std::vector<std::string> ports(input_ports.size());
        std::transform(
            input_ports.begin(), input_ports.end(), ports.begin(), [](auto &p) { return p.first; });
        return ports;
    }

    std::vector<std::string> Module::GetOutputPorts() const
    {
        std::vector<std::string> ports(output_ports.size());
        std::transform(output_ports.begin(), output_ports.end(), ports.begin(),
            [](auto &p) { return p.first; });
        return ports;
    }

    std::vector<IPortPtr> Module::FindInputs(DataType dt) const
    {
        std::vector<IPortPtr> ports;
        for (auto &ptr : input_ports) {
            if (ptr.second->GetType() == dt) {
                ports.push_back(ptr.second);
            }
        };
        return ports;
    }

    std::vector<OPortPtr> Module::FindOutputs(DataType dt) const
    {
        std::vector<OPortPtr> ports;
        for (auto &ptr : output_ports) {
            if (ptr.second->GetType() == dt) {
                ports.push_back(ptr.second);
            }
        };
        return ports;
    }

    IPortPtr Module::AddInputPort(const IPortPtr &port)
    {
        // TODO: 约束不能用相同类型的输入端口?
        if (port) {
            input_ports.emplace(port->GetName(), port);
            // port->SetSlot(boost::bind(&Module::OnData, this, boost::placeholders::_1));
            ROBOT_INFO(verbose > 1, fmt::format("Add input port [{}].", port->GetName()))
            return port;
        }
        return nullptr;
    }

    bool Module::RemoveInputPort(const std::string &name)
    {
        decltype(input_ports)::const_iterator port;
        if ((port = input_ports.find(name)) != input_ports.end()) {
            ROBOT_INFO(verbose > 1, fmt::format("Remove input port [{}].", port->second->GetName()))
            input_ports.erase(port);
            return true;
        }
        return false;
    }

    OPortPtr Module::AddOutputPort(const OPortPtr &port)
    {
        if (port) {
            output_ports.emplace(port->GetName(), port);
            ROBOT_INFO(verbose > 1, fmt::format("Add output port [{}].", port->GetName()))
            return port;
        }
        return nullptr;
    }

    bool Module::Load()
    {
        ROBOT_INFO(verbose > 1, fmt::format("[{}] loading ...", name))
        return false;
    }

    bool Module::Unload()
    {
        ROBOT_INFO(verbose > 1, fmt::format("[{}] unloading ...", name))
        return false;
    }

    bool Module::Start()
    {
        ROBOT_INFO(verbose > 1, fmt::format("[{}] starting ...", name))
        return false;
    }

    bool Module::Stop()
    {
        ROBOT_INFO(verbose > 1, fmt::format("[{}] stopping ...", name))
        return false;
    }

    std::string Module::GetProps() const
    {
        auto node = prop::prop_namespace::generate(props);
        std::stringstream s;
        s << node;
        return s.str();
    }

    bool Module::UpdateProps(const std::string &data)
    {
        auto node = YAML::Load(data);
        auto pairs = node.as<std::vector<YAML::Node>>();
        for (auto &p : pairs) {
            if (p["field"] && p["value"]) {
                props.set(p["field"].as<std::string>(), p["value"]);
            }
        }
        return true;
    }

    std::vector<std::string> Module::GetInteractions() const
    {
        std::vector<std::string> acts;
        for (auto &a : actions) {
            acts.push_back(a.first);
        }
        return acts;
    }

    std::string Module::Interact(const std::string &req) const
    {
        if (actions.find(req) == actions.end())
            throw std::runtime_error(fmt::format("No interaction `{}` is found.", req));

        YAML::Node data;
        auto r = actions.at(req);
        if (!GetRequest(r, data))
            throw std::runtime_error(fmt::format("Get interaction info for `{}` failed.", req));

        YAML::Node node;
        node["data"] = data;
        node["type"] = r->act_type;
        node["require"] = r->target_types;

        std::stringstream s;
        s << node;
        return s.str();
    }

    bool Module::Response(const std::string &req, const std::string &data)
    {
        if (actions.find(req) == actions.end())
            return false;

        auto node = YAML::Load(data);
        ParseResponse(actions.at(req), node);
        return true;
    }

} // namespace navi
