#pragma once
#include <stdint.h>
#include <typeindex>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace action {

    using type_list = std::set<std::type_index>;

    enum class action_types {
        select_point,
        select_curve,
        select_polygen,
        select_mesh,
        choose_item,
    };

    enum class data_types {
        point2d,
        point3d,
        curve2d,
        curve3d,
        polygen2d,
        polygen3d,
        image,
    };

    //-----------------------------------------------------------------------------

    using point2 = cv::Vec2d;
    using point3 = cv::Vec3d;
    using points2 = std::vector<cv::Vec2d>;
    using points3 = std::vector<cv::Vec3d>;

    extern std::map<data_types, type_list> type_mapping;

    std::string format_types(const type_list &types);

    //-----------------------------------------------------------------------------

    using acttypes = boost::bimaps::bimap<action_types, std::string>;
    using dattypes = boost::bimaps::bimap<data_types, std::string>;
    extern const acttypes action_mapper;
    extern const dattypes data_mapper;

    //-----------------------------------------------------------------------------

    struct _act_data_ {
        _act_data_(const std::string &n, data_types dt)
            : name(n)
            , type(dt)
        {
        }

        const std::string name;
        const data_types type;
        virtual YAML::Node get() const = 0;
        virtual void set(const YAML::Node &) = 0;
        virtual const std::type_info &get_type() const = 0;
    };

    //-----------------------------------------------------------------------------

    template <typename T>
    class act_data : public _act_data_ {
    public:
        using value_type = T;

    public:
        act_data(const std::string &n, data_types dt)
            : _act_data_(n, dt)
            , type(typeid(T))
        {
        }

        act_data(const std::string &n, data_types dt, const T &value)
            : _act_data_(n, dt)
            , value(value)
            , type(typeid(T))
        {
        }

        YAML::Node get() const override { return YAML::Node(value); }

        void set(const YAML::Node &n) { value = n.as<T>(); }

        const std::type_info &get_type() const override { return type; }

        T cast() const { return value; }

        void set(const T &val) { value = val; }

    protected:
        T value;
        const std::type_info &type;
    };

    //-----------------------------------------------------------------------------

    using data_ptr = std::shared_ptr<_act_data_>;

    data_ptr create(data_types dt, const std::string &name = "");

    //-----------------------------------------------------------------------------

    class request : std::enable_shared_from_this<request> {
    public:
        request(const std::string &name, action_types at, const std::vector<data_types> &st,
            const std::vector<data_types> &tt)
            : name(name)
            , act_type(at)
            , source_types(st)
            , target_types(tt)
        {
        }
        request(const request &) = delete;

        YAML::Node generate(std::vector<data_ptr> data);

        std::vector<data_ptr> parse(const YAML::Node &node);

    public:
        const std::string name;
        const action_types act_type;
        const std::vector<data_types> source_types;
        const std::vector<data_types> target_types;
    };

    using req_ptr = std::shared_ptr<request>;

    namespace detail {

        template <data_types T>
        data_ptr create();

    } // namespace detail

} // namespace action

namespace YAML {

    using namespace action;

    std::string decode64(const std::string &val);

    std::string encode64(uint8_t *const ptr, size_t size);

    template <int N>
    struct convert<cv::Vec<double, N>> {
        static Node encode(const cv::Vec<double, N> &p)
        {
            std::vector<double> d(N);
            for (int i = 0; i < N; ++i) {
                d[i] = p[i];
            }
            return Node(d);
        }

        static bool decode(const Node &node, cv::Vec<double, N> &p)
        {
            if (!node.IsSequence() || node.size() != p.channels) {
                return false;
            }
            for (int i = 0; i < node.size(); ++i) {
                if (!node[i].IsScalar()) {
                    return false;
                }
            }
            for (int i = 0; i < node.size(); ++i) {
                p[i] = node[i].as<double>();
            }
            return true;
        }
    };

    template <>
    struct convert<cv::Mat> {
        static Node encode(const cv::Mat &img)
        {
            if (img.type() != CV_8UC3) {
                throw std ::runtime_error("Only `8UC3` image is valid.");
            }
            std::vector<uint8_t> buf;
            cv::imencode(".jpg", img, buf);
            auto *enc_msg = reinterpret_cast<uint8_t *>(buf.data());
            return Node(encode64(enc_msg, buf.size()));
        }

        static bool decode(const Node &node, cv::Mat &img)
        {
            auto code = node.as<std::string>();
            auto jpg = decode64(code);
            std::vector<uint8_t> data(jpg.begin(), jpg.end());
            img = cv::imdecode(cv::Mat(data), 1);
            return true;
        }
    };

    template <>
    struct convert<action_types> {
        static Node encode(const action_types &a) { return Node(action_mapper.left.at(a)); }

        static bool decode(const Node &node, action_types &a)
        {
            a = action_mapper.right.at(node.as<std::string>());
            return true;
        }
    };

    template <>
    struct convert<data_types> {
        static Node encode(const data_types &a) { return Node(data_mapper.left.at(a)); }

        static bool decode(const Node &node, data_types &a)
        {
            a = data_mapper.right.at(node.as<std::string>());
            return true;
        }
    };

} // namespace YAML
