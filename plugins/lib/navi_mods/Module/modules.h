#pragma once
#include <typeindex>
#include <boost/signals2.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "prop.hpp"
#include "action.hpp"
#include "types.h"
#include "utils.h"

namespace navi {

    class Port;
    class _InputPort;
    class _OutputPort;
    class Module;
    class Linker;

    using IPortPtr = std::shared_ptr<_InputPort>;
    using OPortPtr = std::shared_ptr<_OutputPort>;
    using IOPortPtr = std::shared_ptr<Port>;
    using ModPtr = std::shared_ptr<Module>;

    template <typename Src>
    class OutputPort;

    struct PortInfo {
        DataType type;
        DataSource source;
        ScalarType scalar_type;
        bool operator==(const PortInfo &p);
    };

    class Port : public std::enable_shared_from_this<Port> {
    public:
        Port(const Port &) = delete;
        ~Port();

        virtual std::string GetName() const { return name; }
        virtual bool IsInput() const = 0;
        virtual bool IsOutput() const = 0;
        virtual DataType GetType() const = 0;
        virtual DataSource GetSource() const = 0;
        virtual ScalarType GetScalarType() const = 0;

        virtual void RegisterConnected(std::function<void(IOPortPtr, IOPortPtr)> fn);
        virtual void RegisterDisconnected(std::function<void(IOPortPtr, IOPortPtr)> fn);

        ModPtr const GetModule() const;
        double GetFrequency() const;

        bool operator==(const Port &p);
        bool operator!=(const Port &p);
        PortInfo operator()() const;

    protected:
        friend class Linker;
        boost::signals2::signal<void(IOPortPtr, IOPortPtr)> OnConnected;
        boost::signals2::signal<void(IOPortPtr, IOPortPtr)> OnDisconnected;

    protected:
        Port(std::string name, Module *parent);
        void UpdateTime(double t);

    protected:
        const std::string name;
        ModPtr parent;

    private:
        std::atomic<double> time = { -1 };
        std::atomic<double> time_ex = { -1 };
        std::map<IOPortPtr, boost::signals2::connection> m_conns;
        std::map<IOPortPtr, boost::signals2::connection> m_disconns;
    };

    class _InputPort : public Port {
    public:
        bool IsInput() const override;
        bool IsOutput() const override;

    protected:
        friend class Module;
        _InputPort(std::string name, Module *parent);
        virtual void Caller() = 0;
    };

    class _OutputPort : public Port {
    public:
        bool IsInput() const override;
        bool IsOutput() const override;
        virtual bool Copy(IPortPtr src, double overtime) = 0;

    protected:
        friend class Module;
        friend class Linker;

        _OutputPort(std::string name, Module *parent);
        ~_OutputPort();

        boost::signals2::connection SetSlot(boost::function<void(OPortPtr)> &&fn);
        void Disconnect(const boost::signals2::connection &conn);
        mutable std::mutex m_slot_mutex;
        boost::signals2::signal<void(OPortPtr)> OnData;
    };

    template <typename Src>
    class InputPort : public _InputPort {
    public:
        InputPort(std::string name, Module *parent)
            : _InputPort(name, parent)
        {
        }

        ~InputPort()
        {
            std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
            OnData.disconnect_all_slots();
        }

        DataType GetType() const override { return Src::data_type; };

        DataSource GetSource() const override { return Src::data_source; }

        ScalarType GetScalarType() const override { return Src::scalar_type; }

    public:
        using type = typename Src::type;
        using stamped = typename ilsr::mutex_data<type>::stamped_data;

        bool Copy(type &src, double overtime) { return m_data.TryGet(src, overtime); }

        bool Copy(stamped &src, double overtime) { return m_data.TryGet(src, overtime); }

        bool Move(type &src, double overtime) { return m_data.TryMove(src, overtime); }

        bool Move(stamped &src, double overtime) { return m_data.TryMove(src, overtime); }

    protected:
        friend class OutputPort<Src>;

        void Pass(const type &src)
        {
            m_data.Set(src);
            UpdateTime(m_data.GetTime());
        }

        void Pass(type &&src)
        {
            m_data.Set(std::move(src));
            UpdateTime(m_data.GetTime());
        }

    protected:
        friend class Module;
        using slot_t = boost::function<void(const stamped &)>;

        boost::signals2::connection SetSlot(slot_t &&fn)
        {
            std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
            return OnData.connect(fn);
        }

        void Disconnect(const boost::signals2::connection &conn)
        {
            std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
            conn.disconnect();
        }

        void Caller() override
        {
            std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
            stamped src;
            if (m_data.TryGet(src, 0.2)) {
                OnData(src);
            }
        }

    private:
        ilsr::mutex_data<type> m_data;
        mutable std::mutex m_slot_mutex;
        boost::signals2::signal<void(const stamped &src)> OnData;
    };

    template <typename Src>
    class OutputPort : public _OutputPort {
    public:
        OutputPort(std::string name, Module *parent)
            : _OutputPort(name, parent)
        {
        }

        DataType GetType() const override { return Src::data_type; };

        DataSource GetSource() const override { return Src::data_source; }

        ScalarType GetScalarType() const override { return Src::scalar_type; }

    public:
        using type = typename Src::type;
        using stamped = typename ilsr::mutex_data<type>::stamped_data;

        bool Copy(type &src, double overtime) { return m_data.TryGet(src, overtime); }

        bool Copy(stamped &src, double overtime) { return m_data.TryGet(src, overtime); }

        void Pass(const type &src)
        {
            m_data.Set(src);
            UpdateTime(m_data.GetTime());
            std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
            OnData(std::shared_ptr<_OutputPort>(this, [](_OutputPort *) {}));
        }

        void Pass(type &&src)
        {
            m_data.Set(std::move(src));
            UpdateTime(m_data.GetTime());
            std::lock_guard<decltype(m_slot_mutex)> _(m_slot_mutex);
            OnData(std::shared_ptr<_OutputPort>(this, [](_OutputPort *) {}));
        }

    protected:
        bool Copy(IPortPtr src, double overtime) override
        {
            if ((*reinterpret_cast<Port *>(src.get())) != (*reinterpret_cast<Port *>(this))) {
                throw std::runtime_error("Source and destination has diffrent data format.");
            }
            typename Src::type data;
            if (m_data.TryGet(data, overtime)) {
                reinterpret_cast<InputPort<Src> *>(src.get())->Pass(std::move(data));
                return true;
            }
            return false;
        }

    private:
        ilsr::mutex_data<type> m_data;
    };

    class Module {
    public:
        Module(
            std::string name, bool dll, std::string alias = "", int threads = 1, int verbose = 1);
        Module(const Module &) = delete;
        ~Module();

    public:
        virtual std::string GetName() const;
        virtual std::string GetReadableName() const;

        int GetVerbose() const;
        void SetVerbose(int verbose);

        bool IsDll() const;
        virtual bool IsLoaded() const = 0;
        virtual bool IsUnloadable() const = 0;
        virtual bool Load();
        virtual bool Unload();
        virtual bool IsRunning() const = 0;
        virtual bool Start();
        virtual bool Stop();
        virtual bool NeedAct() const { return false; }

        /// <summary>
        /// 获取模块属性
        /// </summary>
        /// <returns>YAML格式字符串</returns>
        std::string GetProps() const;
        /// <summary>
        /// 设置模块属性
        /// </summary>
        /// <param name="data">YAML格式字符串, 多个数组, 分别为 {field:-, value:-} </param>
        /// <returns></returns>
        bool UpdateProps(const std::string &data);

        /// <summary>
        /// 获取模块可交互操作
        /// </summary>
        std::vector<std::string> GetInteractions() const;
        /// <summary>
        /// 发起可交互操作
        /// </summary>
        /// <returns>YAML格式字符串, 多个字典, 分别为 {type: -, data: [name: {type:-, data:-}, ...
        /// ], require: [types]} </returns>
        std::string Interact(const std::string &req) const;
        /// <summary>
        /// 返回可交互数据
        /// </summary>
        /// <param name="req">请求名称</param>
        /// <param name="data">YAML格式字符串, 多个字典, 分别为 {name: {type:-, data:-} </param>
        /// <returns></returns>
        bool Response(const std::string &req, const std::string &data);

    public:
        /// <summary>
        /// 模块的输入和输出数据类型完全一致(数量和类型), 不检查数据源和标量类型
        /// </summary>
        /// <returns></returns>
        bool IsConverter() const;

        bool HasPort(const IOPortPtr &port) const;

        IPortPtr GetInputPort(const std::string &name) const;
        OPortPtr GetOutputPort(const std::string &name) const;

        std::vector<std::string> GetInputPorts() const;
        std::vector<std::string> GetOutputPorts() const;

        std::vector<IPortPtr> FindInputs(DataType dt) const;
        std::vector<OPortPtr> FindOutputs(DataType dt) const;

    protected:
        friend class Linker;
        void OnData(OPortPtr src, IPortPtr dst);

        OPortPtr AddOutputPort(const OPortPtr &port);
        IPortPtr AddInputPort(const IPortPtr &port);

        bool RemoveInputPort(const std::string &name);

        template <typename Port>
        IPortPtr AddInputPort(const std::shared_ptr<Port> &port,
            boost::function<void(typename Port::stamped const &)> &&fn)
        {
            auto ptr = this->AddInputPort(port);
            if (ptr) {
                reinterpret_cast<Port *>(ptr.get())->SetSlot(std::move(fn));
            }
            return ptr;
        }

        virtual bool GetRequest(action::req_ptr ptr, YAML::Node &node) const { return false; }
        virtual void ParseResponse(action::req_ptr ptr, const YAML::Node &node) {}

    protected:
        const std::string name;
        const std::string alias;
        // 模块属性(参数)
        prop::prop_namespace props;
        // 模块可交换操作
        std::map<std::string, action::req_ptr> actions;

    private:
        // 只看类型拷贝数据
        void _OnData(_InputPort &port);
        // 根据链接关系拷贝数据
        void _OnData(_OutputPort &src, _InputPort &dst);

    private:
        const bool dll;
        std::atomic_int verbose;
        std::map<std::string, IPortPtr> input_ports;
        std::map<std::string, OPortPtr> output_ports;

        boost::asio::io_service io_service;
        std::shared_ptr<boost::asio::io_service::work> work;
        std::shared_ptr<boost::thread_group> workers;
    };

} // namespace navi
