#include "utils.h"
#include "prop.hpp"

namespace prop {

    // clang-format off
    const bitypes types_mapper = boost::assign::list_of<bitypes::relation>
        (types::_bool,   "bool"     )
        (types::_int,    "int"      )
        (types::_float,  "float"    )
        (types::_string, "string"   )
        (types::_vector, "vector"   )
        (types::_enum,   "enum"     );


    const std::map<types, type_list> tag_list{
        { types::_bool, { 
            typeid(bool), 
            typeid(int), 
            typeid(int64_t) } },
        { types::_int, { 
            typeid(int), 
            typeid(int64_t) } },
        { types::_float, { 
            typeid(float), 
            typeid(double) } },
        { types::_string, {
            typeid(std::string) } },
        { types::_vector, {
            typeid(std::vector<int>), 
            typeid(std::vector<int64_t>), 
            typeid(std::vector<float>), 
            typeid(std::vector<double>) } },
        { types::_enum, { 
            typeid(int), 
            typeid(int64_t), 
            typeid(std::string) } },
    };
    // clang-format on

    //-----------------------------------------------------------------------------

    prop_item::prop_item(const this_is_private, prop_namespace &parent, const std::string &name,
        types type, prop_ptr def)
        : parent(&parent)
        , name(name)
        , type(type)
        , defval(def)
    {
    }

    YAML::Node prop_item::get()
    {
        YAML::Node item;
        item["field"] = parent->get_field();
        item["name"] = name;
        item["type"] = types_mapper.left.at(type);
        item["defval"] = defval->get();
        item["value"] = value->get();
        item["comment"] = comment;
        if (tag)
            item["tag"] = tag->get();
        if (!notes.empty())
            item["note"] = notes;
        return item;
    }

    //-----------------------------------------------------------------------------

    prop_namespace::prop_namespace()
        : parent(nullptr)
        , name("root")
        , sub_ns()
        , leafs()
    {
    }

    prop_namespace::prop_namespace(prop_namespace &p, const std::string &n)
        : parent(&p)
        , name(n)
        , sub_ns()
        , leafs()
    {
    }

    prop_namespace &prop_namespace::emplace_ns(std::string field)
    {
        auto pns = std::make_shared<prop_namespace>(*this, field);
        auto p = sub_ns.insert(pns);
        if (!p.second) {
            throw std::runtime_error("This filed `" + field + "` already exists.");
        }
        return **(p.first);
    }

    void prop_namespace::emplace_ns(prop_namespace &ns)
    {
        auto p = sub_ns.insert(ns.shared_from_this());
        if (!p.second) {
            throw std::runtime_error("This filed `" + ns.name + "` already exists.");
        }
    }

    std::string prop_namespace::get_field() const
    {
        if (!is_root()) {
            if (parent->is_root()) {
                return this->name;
            } else {
                return parent->get_field() + "/" + this->name;
            }
        }
        return "";
    }

    bool prop_namespace::is_root() const { return parent == nullptr; }

    bool prop_namespace::is_leaf() const { return sub_ns.empty() && !leafs.empty(); }

    YAML::Node prop_namespace::generate(const prop_namespace &pns)
    {
        if (!pns.is_root()) {
            throw std::runtime_error("`generate` only accept a root namespace.");
        }
        std::vector<YAML::Node> chs;
        for (auto &ns : pns.sub_ns) {
            auto pending = ns->get();
            chs.insert(chs.end(), pending.begin(), pending.end());
        }
        return (chs.size() <= 0) ? YAML::Node() : YAML::Node(chs);
    }

    std::vector<YAML::Node> prop_namespace::get()
    {
        std::vector<YAML::Node> chs;

        for (auto &lf : leafs) {
            chs.push_back(lf->get());
        }
        for (auto &ns : sub_ns) {
            auto pending = ns->get();
            chs.insert(chs.end(), pending.begin(), pending.end());
        }
        return chs;
    }

    prop_ptr prop_namespace::find(const std::string &field)
    {
        auto fs = ilsr::split(field, "/");
        if (fs.size() < 1)
            return nullptr;
        // Get sub namespace
        prop_namespace *pns = this;
        for (int i = 0; i < fs.size() - 1; ++i) {
            for (auto &ns : pns->sub_ns) {
                if (ns->name == fs[i]) {
                    pns = ns.get();
                    break;
                }
            }
            if (!pns) {
                return nullptr;
            }
        }
        // Get prop item
        for (auto &p : pns->leafs) {
            if (p->name == fs.back()) {
                return p->value;
            }
        }
        return nullptr;
    }

    //-----------------------------------------------------------------------------

    YAML::Node prop_tag<types::_bool>::get() const { return YAML::Node(); }

    YAML::Node prop_tag<types::_int>::get() const
    {
        YAML::Node tag;
        tag["unit"] = unit;
        tag["range"] = range;
        return tag;
    }

    YAML::Node prop_tag<types::_float>::get() const
    {
        YAML::Node tag;
        tag["unit"] = unit;
        tag["range"] = range;
        return tag;
    }

    YAML::Node prop_tag<types::_string>::get() const
    {
        YAML::Node tag;
        tag["type"] = type;
        return tag;
    }

    YAML::Node prop_tag<types::_enum>::get() const
    {
        YAML::Node tag;
        tag["type"] = type;
        tag["options"] = options;
        return tag;
    }

    YAML::Node prop_tag<types::_vector>::get() const
    {
        YAML::Node tag;
        tag["unit"] = unit;
        tag["size"] = size;
        tag["type"] = type;
        return tag;
    }

} // namespace prop

//#include <iostream>
//
// int main() {
//
//    std::setlocale(LC_ALL, "zh_CN.UTF-8");
//    make_prop_info();
//
//    std::cout << prop_namespace::generate();
//
//    return 0;
//}
