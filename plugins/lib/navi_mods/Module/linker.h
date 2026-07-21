#pragma once
#include <typeindex>
#include "modules.h"
#include "utils.h"

namespace navi {

    class Linker;
    using LinkPtr = std::shared_ptr<Linker>;

    class Linker {
    public:
        /// <summary>
        /// 制定端口匹配, 检查是否一致
        /// </summary>
        /// <param name="src"></param>
        /// <param name="dst"></param>
        /// <param name="verbose"></param>
        /// <returns></returns>
        static LinkPtr Make(OPortPtr src, IPortPtr dst, int verbose = 1);
        /// <summary>
        /// 自动检索可以匹配的输入输出端口, 要求保证唯一匹配
        /// </summary>
        /// <param name="src"></param>
        /// <param name="dst"></param>
        /// <param name="verbose"></param>
        /// <returns></returns>
        static LinkPtr Make(ModPtr src, ModPtr dst, int verbose = 1);
        /// <summary>
        /// 检索符合类型的匹配
        /// </summary>
        static LinkPtr Make(ModPtr src, ModPtr dst, DataType dt, int verbose = 1);
        static LinkPtr Make(ModPtr src, ModPtr dst, DataType dt, DataSource ds, int verbose = 1);
        static LinkPtr Make(
            ModPtr src, ModPtr dst, DataType dt, DataSource ds, ScalarType st, int verbose = 1);

    public:
        std::string GetID() const;
        const OPortPtr GetSource() const;
        const IPortPtr GetDestination() const;

    private:
        struct private_this {
        };

    public:
        Linker(private_this, std::string id, OPortPtr src, IPortPtr dst, int verbose = 1);
        Linker(const Linker &) = delete;
        ~Linker();

    protected:
        friend class Pipeline;

        /// <summary>
        /// 将两个Linker关联起来，因此prev的目标模块必须与next的源模块相同
        /// </summary>
        /// <param name="prev"></param>
        /// <param name="next"></param>
        static void AddChain(LinkPtr prev, LinkPtr next);
        /// <summary>
        /// 切断链条, 删除Linker的输出模块及后续模块
        /// </summary>
        /// <param name="link"></param>
        static void BreakChain(LinkPtr link);

    private:
        void OnData(OPortPtr port);
        void ClearSlots();

    private:
        const std::string id;
        const int verbose;
        std::string info;

        OPortPtr source = nullptr;
        IPortPtr destin = nullptr;

        struct slot_t {
            OPortPtr port;
            boost::signals2::connection connection;
        };
        std::shared_ptr<slot_t> m_slot;
    };

} // namespace navi
