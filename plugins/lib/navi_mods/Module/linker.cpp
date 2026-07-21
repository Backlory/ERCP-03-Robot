#include "robot_config.h"
#include "linker.h"

namespace navi {

    LinkPtr Linker::Make(OPortPtr src, IPortPtr dst, int verbose)
    {
        if (!src || !dst) {
            throw std::runtime_error("Any of given ports is null.");
        }
        if (*src != *dst) {
            // TODO: 弱化源和值的类型约束
            throw std::runtime_error("Given ports are not compatible.");
        }
        // TODO: 生成唯一ID
        auto name = fmt::format("{}:{}-{}:{}", src->GetModule()->GetName(), src->GetName(),
            dst->GetModule()->GetName(), dst->GetName());
        return std::make_shared<Linker>(private_this{}, name, src, dst, verbose);
    }

    LinkPtr Linker::Make(ModPtr src, ModPtr dst, int verbose)
    {
        if (!src || !dst) {
            throw std::runtime_error("Any of given modules is null.");
        }

        std::vector<LinkPtr> links;
        for (auto &p : dst->GetInputPorts()) {
            auto pin = dst->GetInputPort(p);
            if (pin) {
                auto ports = src->FindOutputs(pin->GetType());
                for (auto &pout : ports) {
                    auto link = Make(pout, pin, verbose);
                    if (link) {
                        links.push_back(link);
                    }
                }
            }
        }
        if (links.size() <= 0) {
            throw std::runtime_error("Cannot make link.");
        }
        if (links.size() > 1) {
            throw std::runtime_error("Multi possible connections found, should choose only one.");
        }
        return links[0];
    }

    //-------------------------------------------------------------------------

    Linker::Linker(private_this, std::string id, OPortPtr src, IPortPtr dst, int verbose)
        : id(id)
        , source(src)
        , destin(dst)
        , verbose(verbose)

    {
        m_slot = std::make_shared<slot_t>(slot_t{
            source, source->SetSlot(boost::bind(&Linker::OnData, this, boost::placeholders::_1)) });
        info = fmt::format("{}.{} <-> {}.{}", source->GetModule()->GetName(), source->GetName(),
            destin->GetModule()->GetName(), destin->GetName());
        ROBOT_INFO(verbose > 0, fmt::format("Linker maked for `{}`.", info))

        // 回调
        src->OnConnected(src, dst);
        dst->OnConnected(dst, src);
    }

    Linker::~Linker()
    {
        ClearSlots();
        ROBOT_INFO(verbose > 0, fmt::format("Linker released for `{}`.", info))
    }

    std::string Linker::GetID() const { return id; }
    const OPortPtr Linker::GetSource() const { return source; }
    const IPortPtr Linker::GetDestination() const { return destin; }

    void Linker::OnData(OPortPtr src)
    {
        auto dst = GetDestination();
        auto m = dst->GetModule();
        if (m)
            m->OnData(src, dst);
    }

    void Linker::ClearSlots()
    {
        // 清除输入输出连接关系
        if (m_slot) {
            m_slot->port->Disconnect(m_slot->connection);
            // 回调
            source->OnDisconnected(source, destin);
            destin->OnDisconnected(destin, source);
        }
    }

} // namespace navi
