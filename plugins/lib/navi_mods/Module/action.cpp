#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/algorithm/string.hpp>
#include <fmt/format.h>
#include "action.hpp"

namespace action {

    std::map<data_types, type_list> type_mapping = {
        { data_types::point2d, { typeid(point2) } },
        { data_types::point3d, { typeid(point3) } },
        { data_types::curve2d, { typeid(points2) } },
        { data_types::curve3d, { typeid(points3) } },
        { data_types::polygen2d, { typeid(points2) } },
        { data_types::polygen3d, { typeid(points3) } },
        { data_types::image, { typeid(cv::Mat) } },
    };

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

    // clang-format off
    static const acttypes action_mapper = boost::assign::list_of<acttypes::relation>
        (action_types::select_point,    "select_point")
        (action_types::select_curve,    "select_curve")
        (action_types::select_polygen,  "select_polygen")
        (action_types::select_mesh,     "select_mesh")
        (action_types::choose_item,     "choose_item");

    static const dattypes data_mapper = boost::assign::list_of<dattypes::relation>
        (data_types::point2d,   "point2d")
        (data_types::point3d,   "point3d")
        (data_types::curve2d,   "curve2d")
        (data_types::curve3d,   "curve3d")
        (data_types::polygen2d, "polygen2d")
        (data_types::polygen3d, "polygen3d")
        (data_types::image,     "image");
    // clang-format on

    //-----------------------------------------------------------------------------

    YAML::Node request::generate(std::vector<data_ptr> data)
    {
        if (source_types.size() != data.size()) {
            throw std::runtime_error("Given data doesn't has coorect size.");
        }
        YAML::Node node;
        for (int i = 0; i < source_types.size(); ++i) {
            auto &d = data[i];
            auto &dt = source_types[i];

            const auto &types = type_mapping.at(dt);
            if (d->type != dt
                || std::find(types.begin(), types.end(), d->get_type()) == types.end()) {
                throw std::runtime_error(
                    fmt::format("Request member {} is supposed to be {}, but {} is given.", d->name,
                        format_types(types), d->get_type().name()));
            }
            YAML::Node item;
            item["type"] = d->type;
            item["data"] = d->get();
            node[d->name] = item;
        }
        return node;
    }

    std::vector<data_ptr> request::parse(const YAML::Node &node)
    {
        std::vector<data_ptr> data = {};
        auto nodes = node.as<std::vector<YAML::Node>>();
        for (auto &n : nodes) {
            try {
                if (!n["type"] || !n["data"]) {
                    continue;
                }
                data_types type = n["type"].as<data_types>();
                auto ptr = create(type);
                ptr->set(n["data"]);
                data.emplace_back(std::move(ptr));
            } catch (...) {
            }
        }
        return data;
    }

    namespace detail {

        template <data_types T>
        data_ptr create(const std::string &name);

        template <>
        data_ptr create<data_types::point2d>(const std::string &name)
        {
            return std::make_shared<act_data<action::point2>>(name, data_types::point2d);
        }

        template <>
        data_ptr create<data_types::point3d>(const std::string &name)
        {
            return std::make_shared<act_data<point3>>(name, data_types::point3d);
        }

        template <>
        data_ptr create<data_types::curve2d>(const std::string &name)
        {
            return std::make_shared<act_data<points2>>(name, data_types::curve2d);
        }

        template <>
        data_ptr create<data_types::polygen2d>(const std::string &name)
        {
            return std::make_shared<act_data<points2>>(name, data_types::polygen2d);
        }

        template <>
        data_ptr create<data_types::curve3d>(const std::string &name)
        {
            return std::make_shared<act_data<points3>>(name, data_types::curve3d);
        }

        template <>
        data_ptr create<data_types::polygen3d>(const std::string &name)
        {
            return std::make_shared<act_data<points3>>(name, data_types::polygen3d);
        }

        template <>
        data_ptr create<data_types::image>(const std::string &name)
        {
            return std::make_shared<act_data<cv::Mat>>(name, data_types::image);
        }

    } // namespace detail

    data_ptr create(data_types dt, const std::string &name)
    {
        if (dt == data_types::point2d) {
            return detail::create<data_types::point2d>(name);
        } else if (dt == data_types::point3d) {
            return detail::create<data_types::point3d>(name);
        } else if (dt == data_types::curve2d) {
            return detail::create<data_types::curve2d>(name);
        } else if (dt == data_types::curve3d) {
            return detail::create<data_types::curve3d>(name);
        } else if (dt == data_types::polygen2d) {
            return detail::create<data_types::polygen2d>(name);
        } else if (dt == data_types::polygen3d) {
            return detail::create<data_types::polygen3d>(name);
        } else if (dt == data_types::image) {
            return detail::create<data_types::image>(name);
        }
        return nullptr;
    }

} // namespace action

std::string YAML::decode64(const std::string &val)
{
    using namespace boost::archive::iterators;
    using It = transform_width<binary_from_base64<std::string::const_iterator>, 8, 6>;
    return boost::algorithm::trim_right_copy_if(
        std::string(It(std::begin(val)), It(std::end(val))), [](char c) { return c == '\0'; });
}

std::string YAML::encode64(uint8_t *const ptr, size_t size)
{
    using namespace boost::archive::iterators;
    using It = base64_from_binary<transform_width<std::string::const_iterator, 6, 8>>;
    std::string buff(ptr, ptr + size);
    auto tmp = std::string(It(buff.begin()), It(buff.end()));
    return tmp.append((3 - size % 3) % 3, '=');
}

// #include <fstream>
// int main()
//{
//    using namespace action;
//    request req("roi", action_types::select_polygen,
//        { data_types::point2d, data_types::curve2d, data_types::image }, {
//        data_types::polygen2d
//        });
//
//    cv::Mat src(320, 320, CV_8UC3);
//    cv::randn(src, 128, 128);
//
//    // clang-format off
//    auto p = std::make_shared<act_data<point2>>("point", data_types::point2d, point2(0, 1));
//    auto c = std::make_shared<act_data<points2>>("curve", data_types::curve2d, points2{
//    point2(1, 1), point2(-1, 1) }); auto i = std::make_shared<act_data<cv::Mat>>("image",
//    data_types::image, src);
//    // clang-format on
//
//    std::ofstream of("test.yaml");
//    of << req.generate({ p, c, i });
//}
