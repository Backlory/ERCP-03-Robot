#pragma once
#include "helper.hpp"
#include "ipc_sender.hpp"

namespace net { namespace ipc {

    template <typename Src, typename Pkt>
    class ipc_reader {
    public:
        using type = Src;
        using pack_type = Pkt;
        using direction = _input_net_type;

        ipc_reader(const std::string &name, int verbose);
        ~ipc_reader();

        std::string get_name() const { return name; }
        std::string get_address() const { return name; }
        bool is_running() const { return true; }
        bool decode(Pkt &pkt, Src &src);

        const int verbose;
        ilsr::mutex_data<Src> frame;

    private:
        using seg = ipc_named_segment;
        using Hash_t = ipc_vector<size_t>;
        using Time_t = ipc_vector<double>;
        using Data_t = ipc_vectors<char>;

        void get_memory();

    private:
        const std::string name;
        std::atomic<size_t> counter;

        // TODO: Use guid name
        bool tried = false;
        mutable std::mutex mutex;
        std::shared_ptr<managed_shared_memory> memory;

        std::shared_ptr<boost::thread> m_thread;
        void DecodeWorker();
    };

    template <typename Src, typename Pkt>
    ipc_reader<Src, Pkt>::ipc_reader(const std::string &name, int verbose)
        : name(name)
        , verbose(verbose)
    {
        m_thread = std::make_shared<boost::thread>(boost::bind(&ipc_reader::DecodeWorker, this));
    }

    template <typename Src, typename Pkt>
    ipc_reader<Src, Pkt>::~ipc_reader()
    {
        if (m_thread) {
            m_thread->interrupt();
            m_thread->join();
        }
        ROBOT_INFO(true, fmt::format("Release shared memory reader `{}`.", name))
    }

    template <typename Src, typename Pkt>
    void ipc_reader<Src, Pkt>::get_memory()
    {
        if (!memory) {
            try {
                memory = std::make_shared<managed_shared_memory>(
                    open_only, (get_prefix() + name).c_str());
                ROBOT_INFO(true, fmt::format("Get shared memory reader for `{}`.", name))
                tried = false;
                return;
            } catch (std::exception &e) {
            }
            memory = nullptr;
            ROBOT_ERROR(
                verbose > 0 && !tried, fmt::format("Get shared memory failed for `{}`.", name))
            tried = true;
        }
    }

    template <typename Src, typename Pkt>
    void ipc_reader<Src, Pkt>::DecodeWorker()
    {
        ROBOT_THREADNAME(fmt::format("Dec/{}", name).c_str())

        counter = 0;
        double dtime = -1;
        double time_dump = -1;
        //size_t hash_dump = 0x00;

        std::vector<char> buffer;

        while (!boost::this_thread::interruption_requested()) {
            try {
                // 检查共享内存是否更新
                buffer.resize(0);
                try {
                    if (!memory) {
                        get_memory();
                        if (!memory) {
                            // 等一段时间再尝试
                            ilsr::Time::sleep_for(0.2);
                        }
                    }
                    if (memory) {
                        auto size = memory->find<size_t>( //
                            get_segment_name(seg::Size).c_str());
                        auto count = memory->find<size_t>( //
                            get_segment_name(seg::Counter).c_str());
                        auto times = memory->find<Time_t>( //
                            get_segment_name(seg::Time).c_str());
                        auto buffs = memory->find<Data_t>( //
                            get_segment_name(seg::Data).c_str());

                        if (size.second > 0 && size.first && //
                            count.second > 0 && count.first && //
                            times.second > 0 && times.first && //
                            buffs.second > 0 && buffs.first) {
                            const size_t ic = (*count.first) % (*size.first);
                            const double ts = (*times.first)[ic];
                            if (ts != time_dump) {
                                time_dump = ts;
                                auto &buff = *((*buffs.first).begin() + ic);
                                buffer.insert(buffer.begin(), buff.begin(), buff.end());
                                ROBOT_INFO(
                                    verbose > 1, fmt::format("Read: {:08d}, {:.06f}", ic, ts));
                            }
                        }
                    }

                } catch (std::exception &e) {
                    buffer.clear();
                    ROBOT_ERROR(verbose > 0, "[ShmemError] " << e.what());
                }

                if (buffer.size() > 0) {
                    Pkt pack;
                    // 反序列化结构体数据到数据流
                    boost::iostreams::array_source source(buffer.data(), buffer.size());
                    boost::iostreams::stream<boost::iostreams::array_source> is(source);
                    boost::archive::binary_iarchive ia(is);
                    ia >> pack;

                    Src src;
                    if (decode(pack, src)) {
                        // 更新缓存
                        frame.Set(src);
                        counter++;
                    }
                }
            } catch (const std::exception &e) {
                ROBOT_ERROR(verbose > 0, "[Error] " << e.what())
            }
            boost::this_thread::interruption_point();
            ilsr::Time::sleep_for(0.0001);
        }
    }

}} // namespace net::ipc