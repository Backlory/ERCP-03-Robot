#pragma once
#include <type_traits>
#include "Module/types.h"
#include "Module/packet.h"
#include "robot_config.h"
#include "helper.hpp"
#include "utils.h"

namespace net {

#define NET                                                                                        \
    template <typename, typename>                                                                  \
    typename NetType

    class StreamerBase;
    using StreamPtr = std::shared_ptr<StreamerBase>;

    template <navi::DataType DT, navi::DataSource DS, navi::ScalarType ST, NET>
    class IStreamer;

    template <navi::DataType DT, navi::DataSource DS, navi::ScalarType ST, NET>
    class OStreamer;

    //-----------------------------------------------------------------------------

    namespace _detail {

        using namespace navi;
        using namespace navi::detail;

        template <NET, typename...>
        struct stream_filter;

        template <NET>
        struct stream_filter<NetType, table<>> {
            static StreamPtr execute(navi::DataType DT, navi::DataSource DS, navi::ScalarType ST,
                const std::string &address, int verbose)
            {
                return nullptr;
            }
        };

        template <NET, typename Item0, typename... Items>
        struct stream_filter<NetType, table<Item0, Items...>> {
            static StreamPtr execute(navi::DataType DT, navi::DataSource DS, navi::ScalarType ST,
                const std::string &address, int verbose)
            {
                if (DT == Item0::data_type && DS == Item0::data_source
                    && ST == Item0::scalar_type) {
                    using istream = IStreamer<Item0::data_type, Item0::data_source,
                        Item0::scalar_type, NetType>;
                    using ostream = OStreamer<Item0::data_type, Item0::data_source,
                        Item0::scalar_type, NetType>;
                    using stream = typename std::conditional<
                        std::is_same<typename NetType<int, int>::direction, _input_net_type>::value,
                        istream, ostream>::type;
                    return std::make_shared<stream>(address, verbose);
                } else {
                    return stream_filter<NetType, table<Items...>>::execute(
                        DT, DS, ST, address, verbose);
                }
            }
        };

    } // namespace _detail

    //-----------------------------------------------------------------------------

    class StreamerBase {
    public:
        virtual bool Start() = 0;
        virtual bool Stop() = 0;
        virtual bool IsRunning() const = 0;
        virtual std::string GetAddress() const { return address; };

        template <NET>
        static StreamPtr make_input_streamer(const std::string &address, navi::DataType DT,
            navi::DataSource DS, navi::ScalarType ST, int verbose = 1)
        {
            // XXX: how can <int, int> be more general ?
            static_assert(
                std::is_same<typename NetType<int, int>::direction, _input_net_type>::value,
                "Given `NetType` is not compatible with input stream.");

            StreamPtr ptr = _detail::stream_filter<NetType, _detail::type_table>::execute(
                DT, DS, ST, address, verbose);
            if (!ptr) {
                throw std::runtime_error(
                    fmt::format("Cannot make the input stream for `{}`.", address));
            }
            return ptr;
        }

        template <NET>
        static StreamPtr make_output_streamer(const std::string &address, navi::DataType DT,
            navi::DataSource DS, navi::ScalarType ST, int verbose = 1)
        {
            // XXX: how can <int, int> be more general ?
            static_assert(
                std::is_same<typename NetType<int, int>::direction, _output_net_type>::value,
                "Given `NetType` is not compatible with output stream.");

            StreamPtr ptr = _detail::stream_filter<NetType, _detail::type_table>::execute(
                DT, DS, ST, address, verbose);
            if (!ptr) {
                throw std::runtime_error(
                    fmt::format("Cannot make the output stream for `{}`.", address));
            }
            return ptr;
        }

    protected:
        StreamerBase(const std::string &address, int verbose)
            : address(address)
            , verbose(verbose)
        {
        }

        const std::string address;
        const int verbose;
    };

    template <navi::DataType DT, navi::DataSource DS, navi::ScalarType ST, NET>
    class IOStreamer : public StreamerBase {
    public:
        static constexpr navi::DataType data_type = DT;
        static constexpr navi::DataSource data_source = DS;
        static constexpr navi::ScalarType scalar_type = ST;
        using type = typename navi::Data<DT, DS, ST>;
        using dtype = typename type::type;
        using ptr = typename std::shared_ptr<IOStreamer<DT, DS, ST, NetType>>;
        using stamped = typename ilsr::mutex_data<dtype>::stamped_data;
        using net_type = typename NetType<stamped, navi::Payload<type>>;

        IOStreamer(const std::string &address, int verbose)
            : StreamerBase(address, verbose)
        {
            ROBOT_INFO(true, fmt::format("Create stream to `{}`.", address))
        }

        ~IOStreamer()
        {
            Stop();
            ROBOT_INFO(true, fmt::format("Release stream to `{}`.", address))
        }

        bool Start() override
        {
            std::lock_guard<decltype(m_mutex)> _(m_mutex);
            if (!m_stream) {
                m_stream = std::make_shared<net_type>(address, verbose);
                ROBOT_INFO(true, fmt::format("Stream `{}` opened.", m_stream->get_name()))
            }
            return m_stream != nullptr;
        }

        bool Stop() override
        {
            std::lock_guard<decltype(m_mutex)> _(m_mutex);
            if (m_stream) {
                ROBOT_INFO(true, fmt::format("Stream {} closed.", m_stream->get_name()))
                m_stream.reset();
            }
            return m_stream == nullptr;
        }

        bool IsRunning() const override
        {
            std::lock_guard<decltype(m_mutex)> _(m_mutex);
            return m_stream && m_stream->is_running();
        }

        std::string GetAddress() const override { return m_stream->get_address(); }

    protected:
        mutable std::mutex m_mutex;
        std::shared_ptr<net_type> m_stream;
    };

    template <navi::DataType DT, navi::DataSource DS, navi::ScalarType ST, NET>
    class OStreamer : public IOStreamer<DT, DS, ST, NetType>,
                      public std::enable_shared_from_this<OStreamer<DT, DS, ST, NetType>> {
    public:
        using base = IOStreamer<DT, DS, ST, NetType>;
        using self = OStreamer<DT, DS, ST, NetType>;
        using ptr = std::shared_ptr<self>;

        OStreamer(const std::string &address, int verbose)
            : base(address, verbose)
        {
        }

        ptr GetPtr() { return this->shared_from_this(); }

        bool Write(const typename base::stamped &src)
        {
            std::lock_guard<decltype(base::m_mutex)> _(base::m_mutex);
            if (!base::m_stream) {
                return false;
            }
            base::m_stream->frame.Set(src);
            return true;
        }

        bool Write(typename base::stamped &&src)
        {
            std::lock_guard<decltype(base::m_mutex)> _(base::m_mutex);
            if (!base::m_stream) {
                return false;
            }
            base::m_stream->frame.Set(std::move(src));
            return true;
        }

        static ptr CreateStreamer(const std::string &address, int verbose = 1)
        {
            // XXX: how can <int, int> be more general ?
            static_assert(
                std::is_same<typename NetType<int, int>::direction, _output_net_type>::value,
                "Given `NetType` is not compatible with output stream.");

            auto p = StreamerBase::make_output_streamer<NetType>(address, DT, DS, ST, verbose);
            if (p) {
                return reinterpret_cast<self *>(p.get())->GetPtr();
            }
            return nullptr;
        }
    };

    template <navi::DataType DT, navi::DataSource DS, navi::ScalarType ST, NET>
    class IStreamer : public IOStreamer<DT, DS, ST, NetType>,
                      public std::enable_shared_from_this<IStreamer<DT, DS, ST, NetType>> {
    public:
        using base = IOStreamer<DT, DS, ST, NetType>;
        using self = IStreamer<DT, DS, ST, NetType>;
        using ptr = std::shared_ptr<self>;

        IStreamer(const std::string &address, int verbose)
            : base(address, verbose)
        {
        }

        ptr GetPtr() { return this->shared_from_this(); }

        bool GetFrame(typename base::stamped &dst, double overtime = 0.2, bool move = true) const
        {
            std::lock_guard<decltype(base::m_mutex)> _(base::m_mutex);
            return base::m_stream
                && (move ? base::m_stream->frame.TryMove(dst, overtime)
                         : base::m_stream->frame.TryGet(dst, overtime));
        }

        static ptr CreateStreamer(const std::string &address, int verbose = 1)
        {
            // XXX: how can <int, int> be more general ?
            static_assert(
                std::is_same<typename NetType<int, int>::direction, _input_net_type>::value,
                "Given `NetType` is not compatible with input stream.");

            auto p = StreamerBase::make_input_streamer<NetType>(address, DT, DS, ST, verbose);
            if (p) {
                return reinterpret_cast<self *>(p.get())->GetPtr();
            }
            return nullptr;
        }
    };

    template <typename T, NET>
    using DataOStreamer = OStreamer<T::data_type, T::data_source, T::scalar_type, NetType>;

    template <typename T, NET>
    using DataIStreamer = IStreamer<T::data_type, T::data_source, T::scalar_type, NetType>;

#undef NET

} // namespace net

#include "encoder.hpp"
#include "decoder.hpp"
