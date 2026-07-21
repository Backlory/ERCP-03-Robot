#pragma once
#include <stdint.h>
#include <typeindex>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <yaml-cpp/yaml.h>

namespace prop {

    enum class types {
        _bool,
        _int,
        _float,
        _string,
        _vector,
        _enum,
    };

    struct _prop_ {
        virtual YAML::Node get() const = 0;
        virtual void set(const YAML::Node &) = 0;
        virtual const std::type_info &get_type() const = 0;
    };

    struct _tag_ {
        virtual YAML::Node get() const = 0;
    };

    template <types T>
    struct prop_tag : public _tag_ {
        YAML::Node get() const override;
    };

    struct prop_item;

    struct prop_namespace;

    using prop_ptr = std::shared_ptr<_prop_>;
    using tag_ptr = std::shared_ptr<_tag_>;
    using ns_ptr = std::shared_ptr<prop_namespace>;
    using item_ptr = std::shared_ptr<prop_item>;

    //-----------------------------------------------------------------------------

    using bitypes = boost::bimaps::bimap<types, std::string>;
    extern const bitypes types_mapper;

    using type_list = std::set<std::type_index>;
    extern const std::map<types, type_list> tag_list;

    //-----------------------------------------------------------------------------

    template <typename T>
    class prop_value : public _prop_ {
    public:
        using value_type = T;

    public:
        prop_value(const T &value)
            : value(value)
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

    struct prop_item {

        friend struct prop_namespace;

    protected:
        struct this_is_private {
        };

    public:
        prop_item(const this_is_private, prop_namespace &parent, const std::string &name,
            types type, prop_ptr def);

        template <typename... Args>
        static std::shared_ptr<prop_item> create(Args &&...args)
        {
            return std::make_shared<prop_item>(this_is_private(), ::std::forward<Args>(args)...);
        }

        template <types T, typename... Args>
        prop_item &set_tag(Args &&...args)
        {
            if (T != this->type) {
                std::stringstream ss;
                ss << "Tag type is different with the defination. [ ";
                ss << types_mapper.left.at(T);
                ss << " | ";
                ss << types_mapper.left.at(this->type);
                ss << " ]";
                throw std::runtime_error(ss.str().c_str());
            }

            tag_ptr ptr = std::make_shared<prop_tag<T>>(::std::forward<Args>(args)...);
            this->tag.swap(ptr);
            return *this;
        }

        prop_item &set_comment(const std::string &comm)
        {
            comment = comm;
            return *this;
        }

        prop_item &add_note(const std::string &note)
        {
            notes.push_back(note);
            return *this;
        }

    protected:
        prop_item(const prop_item &) = delete;

        YAML::Node get();

    private:
        const std::string name;
        const types type;
        const prop_ptr defval;
        prop_ptr value;
        tag_ptr tag;
        std::string comment;
        std::vector<std::string> notes;

    protected:
        prop_namespace *parent;
    };

    //-----------------------------------------------------------------------------

    struct prop_namespace : std::enable_shared_from_this<prop_namespace> {
    public:
        // Make a root
        prop_namespace();
        // Make a sub namespace
        prop_namespace(prop_namespace &p, const std::string &n);
        prop_namespace(const prop_namespace &) = delete;

        prop_namespace &add(const std::string &name);

        template <typename T>
        prop_item &emplace(const std::string &name, types type, const T &value)
        {
            prop_ptr def = std::make_shared<prop_value<T>>(value);

            auto ptr = prop_item::create(*this, name, type, def);
            ptr->value = std::make_shared<prop_value<T>>(value);

            auto &list = tag_list.at(type);
            if (list.find(def->get_type()) == list.end()) {
                std::stringstream ss;
                ss << "This type of tag `" << def->get_type().name()
                   << "` is not allowed, valid types are [ ";
                for (auto ptr = list.begin(); ptr != list.end();) {
                    ss << ptr->name() << (++ptr == list.end() ? " ]." : ", ");
                }
                throw std::runtime_error(ss.str().c_str());
            }

            auto p = leafs.insert(ptr);
            if (!p.second) {
                throw std::runtime_error("This item `" + name + "` already exists.");
            }
            return **(p.first);
        }

        prop_namespace &emplace_ns(std::string field);

        std::string get_field() const;

        template <typename T>
        T get(const std::string &field)
        {
            auto p = find(field);
            if (p) {
                return p->get().as<T>();
            }
        }

        template <typename T>
        void set(const std::string &field, const T &val)
        {
            auto p = find(field);
            if (p) {
                if (std::is_same<T, YAML::Node>::value) {
                    p->set(val);
                } else {
                    p->set(YAML::Node(val));
                }
            }
        }

        bool is_root() const;
        bool is_leaf() const;

        const std::string name;

    public:
        static YAML::Node generate(const prop_namespace &ns);

    private:
        std::vector<YAML::Node> get();
        prop_ptr find(const std::string &field);
        void emplace_ns(prop_namespace &ns);

    private:
        std::set<ns_ptr> sub_ns;
        std::set<item_ptr> leafs;
        prop_namespace *parent = nullptr;
    };

    //-----------------------------------------------------------------------------

    template <>
    struct prop_tag<types::_bool> : public _tag_ {
        YAML::Node get() const override;
    };

    template <>
    struct prop_tag<types::_int> : public _tag_ {
        std::string unit;
        std::array<int, 2> range;
        YAML::Node get() const override;
        prop_tag(const std::string &u, const std::array<int, 2> &r)
            : unit(u)
            , range(r)
        {
        }
    };

    template <>
    struct prop_tag<types::_float> : public _tag_ {
        std::string unit;
        std::array<double, 2> range;
        YAML::Node get() const override;
        prop_tag(const std::string &u, const std::array<double, 2> &r)
            : unit(u)
            , range(r)
        {
        }
    };

    template <>
    struct prop_tag<types::_string> : public _tag_ {
        std::string type; // e.g.  ipaddr, ipaddr_port
        YAML::Node get() const override;
        prop_tag(const std::string &t)
            : type(t)
        {
        }
    };

    template <>
    struct prop_tag<types::_enum> : public _tag_ {
        std::string type; // e.g. serial
        std::vector<std::string> options;
        YAML::Node get() const override;
        prop_tag(const std::string &t, const std::vector<std::string> &o)
            : type(t)
            , options(o)
        {
        }
    };

    template <>
    struct prop_tag<types::_vector> : public _tag_ {
        std::string unit;
        size_t size;
        std::string type; // e.g. int, float
        YAML::Node get() const override;
        prop_tag(const std::string &u, size_t s, const std::string &t)
            : unit(u)
            , size(s)
            , type(t)
        {
        }
    };

} // namespace prop
