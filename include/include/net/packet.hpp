#pragma once
#include <stdint.h>
#include <atomic>
#include <vector>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#include "utils.h"

namespace net {

    using Buffer = std::vector<char>;

    class SplitPacketQueue_t {
    public:
        template <typename _PACK>
        class OuputPacket_t {
        public:
            ///< 重组包ID
            uint64_t frame_id = 0;
            ///< 发送端(拆分包)记录的生成时间(拆分子包时间的最小值)
            double product_timestamp = 0;
            ///< 接收端(重组包)记录的消费时间(完成重组的时刻)
            double consume_timestamp = 0;
            ///< 数据是否压缩
            bool compressed = true;
            ///< 重组的包
            _PACK packet;
        };

    public:
        class SplitPacket_t {
        public:
            uint64_t total_count = 0;
            uint64_t total_size = 0;
            uint64_t offset = 0;
            uint64_t frame_id = 0;
            uint64_t pack_id = 0;
            double product_timestamp = 0;
            Buffer packet;

        public:
            template <class Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                ar &total_count;
                ar &total_size;
                ar &offset;
                ar &frame_id;
                ar &pack_id;
                ar &product_timestamp;
                ar &packet;
            }
        };

    protected:
        struct PacketBuffer_t {
            double oldest_time = -1;
            double newest_time = -1;
            double product_time = -1;
            std::vector<bool> occupied;
            Buffer buffer;
        };

    public:
        using spack_ptr = std::unique_ptr<SplitPacket_t>;
        using queue_type = std::map<uint64_t, PacketBuffer_t>;

        template <typename _PACK>
        static std::vector<Buffer> pack(
            uint64_t fid, _PACK &pack, const uint64_t &max_pack_size = 1000)
        {
            std::vector<Buffer> buffers;
            auto ps = split(fid, pack, max_pack_size);
            for (auto &p : ps) {
                Buffer buffer;
                {
                    auto i = boost::iostreams::back_inserter(buffer);
                    boost::iostreams::stream<decltype(i)> os(i);
                    boost::archive::binary_oarchive oa(os);
                    oa << *p;
                    os.flush();
                }
                buffers.emplace_back(std::move(buffer));
            }
            return buffers;
        }

    protected:
        /**
         * @brief 拆分包，把输入结构体数据序列化并拆分成不大于max_pack_size的一系列包
         * @tparam _PACK 输入数据结构类型,需要实现对boost的serialize函数
         * @param fid 帧ID
         * @param pack 数据包
         * @param max_pack_size 最大包字节大小,限制在上限1000以内的值均可
         * @note 这里的1000不是绝对的,只要协议支持其实都可以,1000是相对保守的能兼容目前需求(UDP)的值
         * @return std::vector<spack_ptr>
         */
        template <typename _PACK>
        static std::vector<spack_ptr> split(
            uint64_t fid, _PACK &pack, const uint64_t &max_pack_size)
        {
            const double t = ilsr::Time::wall_time();
            // This may satify many network protocol requirements.
            assert(max_pack_size <= 1000);
            std::vector<spack_ptr> ret;
            // Serialize
            Buffer buffer;
            {
                auto i = boost::iostreams::back_inserter(buffer);
                boost::iostreams::stream<decltype(i)> os(i);
                boost::archive::binary_oarchive oa(os);
                oa << pack;
                os.flush();
            }
            // Create splited packets
            int pid = 0;
            const uint64_t spsize = buffer.size();
            const uint64_t spcount = std::ceil(buffer.size() * 1.0 / max_pack_size);
            for (uint64_t ins = 0; ins < buffer.size(); ins += max_pack_size) {
                uint64_t len = std::min(buffer.size() - ins, max_pack_size);
                spack_ptr p = std::make_unique<SplitPacket_t>();
                p->product_timestamp = t;
                p->frame_id = fid;
                p->pack_id = pid++;
                p->total_count = spcount;
                p->total_size = spsize;
                p->offset = ins;
                p->packet.insert(p->packet.end(), buffer.begin() + ins, buffer.begin() + ins + len);
                ret.emplace_back();
                ret.back().swap(p);
            }
            return ret;
        }

    public:
        /**
         * @brief 清除当前队列
         * @param pack 接收的新包
         */
        void clear()
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _queue.clear();
        }

        bool solve(const Buffer &data)
        {
            // 反序列化数据包
            SplitPacketQueue_t::SplitPacket_t pack;
            {
                boost::iostreams::array_source source(data.data(), data.size());
                boost::iostreams::stream<boost::iostreams::array_source> is(source);
                boost::archive::binary_iarchive ia(is);
                ia >> pack;
            }
            return this->save(pack);
        }

    protected:
        /**
         * @brief 保存接收包到队列里
         * @param pack 接收的新包
         * @return true
         * @return false
         */
        bool save(SplitPacket_t &pack)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // 创建新帧缓存区域
            if (_queue.find(pack.frame_id) == _queue.end()) {
                _queue.emplace(pack.frame_id, PacketBuffer_t());
            }
            auto &buf = _queue[pack.frame_id];
            // 保证足够的缓存空间
            if (buf.buffer.size() < pack.total_size) {
                buf.buffer.assign(pack.total_size, 0x00);
            }
            if (buf.occupied.size() < pack.total_count) {
                buf.occupied.assign(pack.total_count, false);
            }
            // 拷贝数据
            std::copy(pack.packet.begin(), pack.packet.end(), buf.buffer.data() + pack.offset);
            // 标记包ID
            buf.occupied[pack.pack_id] = true;
            // 记录时间
            buf.product_time = pack.product_timestamp;
            buf.newest_time = ilsr::Time::wall_time();
            if (buf.oldest_time <= 0) {
                buf.oldest_time = buf.newest_time;
            }
            return true;
        }

    public:
        /**
         * @brief 重组接受包，将一个帧ID的包按包ID排序重组
         *
         * @tparam _PACK 输入数据结构类型,需要实现对boost的serialize函数
         * @param dt 同一个帧ID序列的包之间的接受时间的最大间隔,超过这个时间则整个帧ID序列被丢弃
         * @return std::vector<std::unique_ptr<_PACK>>
         */
        template <typename _PACK>
        std::vector<std::unique_ptr<OuputPacket_t<_PACK>>> reform(double dt = 0.5)
        {
            const double t = ilsr::Time::wall_time();
            std::vector<std::unique_ptr<OuputPacket_t<_PACK>>> ret;
            std::vector<queue_type::iterator> _erasing;

            std::lock_guard<std::mutex> lock(_mutex);
            for (auto it = _queue.begin(); it != _queue.end(); ++it) {
                auto &buf = it->second;
                // 检查是否超时
                if (t - buf.newest_time > dt) {
                    _erasing.emplace_back(it);
                    continue;
                }
                // 检查是否完整
                if (std::find(buf.occupied.begin(), buf.occupied.end(), false) != buf.occupied.end()) {
                    continue;
                }
                // 反序列化
                _PACK pack;
                {
                    boost::iostreams::array_source source(buf.buffer.data(), buf.buffer.size());
                    boost::iostreams::stream<boost::iostreams::array_source> is(source);
                    boost::archive::binary_iarchive ia(is);
                    ia >> pack;
                }
                // 生成输出包
                auto p = std::make_unique<OuputPacket_t<_PACK>>();
                p->frame_id = it->first;
                p->product_timestamp = buf.product_time;
                p->packet = std::move(pack);
                ret.emplace_back();
                ret.back().swap(p);
                // 标记删除
                _erasing.emplace_back(it);
            }
            // 删除已经处理的帧
            for (auto &it : _erasing) {
                _queue.erase(it);
            }
            return ret;
        }

        uint64_t get_queue_size() const { return _queue.size(); }

        uint64_t get_drop_size() const { return _drop_size; }

    private:
        queue_type _queue;
        std::mutex _mutex;
        std::atomic<int> _drop_size;
    };

} // namespace net
