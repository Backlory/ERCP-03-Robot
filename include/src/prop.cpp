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

prop_item::prop_item(const this_is_private,
                     prop_namespace &parent,
                     const std::string &name,
                     types type,
                     prop_ptr def)
    : parent(&parent)
    , name(name)
    , type(type)
    , defval(def)
{
}

/**
 * @brief 将单个属性及其元数据导出为 YAML 节点。
 * @details 输出字段路径、类型、默认值、当前值以及可选标签和备注，供配置查询接口使用。
 */
YAML::Node prop_item::get()
{
    // 将属性路径、类型、默认值、当前值及标签/备注集中导出为 YAML 节点。
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

/**
 * @brief 计算当前命名空间相对于根节点的层级路径。
 * @details 沿父节点递归拼接名称，根节点返回空字符串，子节点之间使用 '/' 分隔。
 */
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

bool prop_namespace::is_root() const
{
    return parent == nullptr;
}

bool prop_namespace::is_leaf() const
{
    return sub_ns.empty() && !leafs.empty();
}

/**
 * @brief 从根属性命名空间生成可序列化的 YAML 属性列表。
 * @details 校验调用对象必须是根节点，再按命名空间递归收集叶属性并保持遍历顺序。
 */
YAML::Node prop_namespace::generate(const prop_namespace &pns)
{
    // 只接受根命名空间，递归收集所有子命名空间的属性节点并生成 YAML 序列。
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

/**
 * @brief 收集当前命名空间及其所有后代属性。
 * @details 先输出当前层叶子，再递归追加子命名空间，形成稳定且易读的配置顺序。
 */
std::vector<YAML::Node> prop_namespace::get()
{
    // 先收集当前命名空间叶子，再递归追加子命名空间，保持配置树的自然阅读顺序。
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

/**
 * @brief 按 '/' 分隔的属性路径查找对应属性值。
 * @details 先逐级定位命名空间，再在目标层查找叶属性；任一层不存在时返回空指针。
 */
prop_ptr prop_namespace::find(const std::string &field)
{
    // 阶段一：按 '/' 分割属性路径；阶段二：逐级定位命名空间；阶段三：返回目标叶子的当前值。
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

YAML::Node prop_tag<types::_bool>::get() const
{
    return YAML::Node();
}

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
