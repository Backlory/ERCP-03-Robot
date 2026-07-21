// A learnable case: https://github.com/tkem/fsmlite
#pragma once
#include <atomic>
#include <type_traits>

namespace fsmlib {

    namespace detail {
        ////////////////////////////////////////////////////////////////
        // An implement of std::invocable for lower c++17
        ////////////////////////////////////////////////////////////////
#if __cplusplus < 201703L || _MSVC_LANG < 201703L

        template <typename _Callable, typename... _Args>
        using invoke_result_t = typename std::_Select_invoke_traits<_Callable, _Args...>::type;

        template <typename _Callable, typename... _Args>
        struct is_invocable : std::_Select_invoke_traits<_Callable, _Args...>::_Is_invocable {
            // determines whether _Callable is callable with _Args
        };

#elif __cplusplus < 201103L || _MSVC_LANG < 201103L
#error "Requiring C++11 support."
#endif

        ////////////////////////////////////////////////////////////////
        // These helpers make caller for action depending on the parameter list.
        ////////////////////////////////////////////////////////////////
        template <typename action, typename obj, typename event,
            bool f1 = is_invocable<action>::value, bool f2 = is_invocable<action, obj>::value,
            bool f3 = is_invocable<action, event>::value,
            bool f4 = is_invocable<action, obj, event>::value>
        struct _invoke_helper;

        template <typename action, typename obj, typename event>
        struct _invoke_helper<action, obj, event, true, false, false, false> {
            using result_t = invoke_result_t<action>;
            static result_t invoke(action &&f, obj &&, event &&) { return std::invoke(f); }
        };

        template <typename action, typename obj, typename event>
        struct _invoke_helper<action, obj, event, false, true, false, false> {
            using result_t = invoke_result_t<action, obj>;
            static result_t invoke(action &&f, obj &&a, event &&) { return std::invoke(f, a); }
        };

        template <typename action, typename obj, typename event>
        struct _invoke_helper<action, obj, event, false, false, true, false> {
            using result_t = invoke_result_t<action, event>;
            static result_t invoke(action &&f, obj &&, event &&b) { return std::invoke(f, b); }
        };

        template <typename action, typename obj, typename event>
        struct _invoke_helper<action, obj, event, false, false, false, true> {
            using result_t = invoke_result_t<action, obj, event>;
            static result_t invoke(action &&f, obj &&a, event &&b) { return std::invoke(f, a, b); }
        };

        template <typename action, typename obj, typename event>
        using _action_result_t = typename _invoke_helper<action, obj, event>::result_t;

        template <typename action, typename obj, typename event>
        _action_result_t<action, obj, event> do_action(action &&f, obj &&a, event &&b)
        {
            return _invoke_helper<action, obj, event>::invoke(
                std::forward<action>(f), std::forward<obj>(a), std::forward<event>(b));
        }

        ////////////////////////////////////////////////////////////////
        // Following are basic components of transition.
        ////////////////////////////////////////////////////////////////
        // We use nothing of `_transition_table` but its template
        template <typename...>
        struct _transition_table {
        };

        ////////////////////////////////////////////////////////////////
        template <class...>
        struct _transition_table_concat;

        // For end of the concating.
        template <>
        struct _transition_table_concat<> {
            using type = _transition_table<>;
        };

        template <class T, class... Types>
        struct _transition_table_concat<T, _transition_table<Types...>> {
            using type = _transition_table<T, Types...>;
        };

        ////////////////////////////////////////////////////////////////
        template <template <typename> typename _Condition, typename...>
        struct _transition_filter;

        ///< For those no valid filtered items.
        template <template <typename> typename _Condition>
        struct _transition_filter<_Condition> {
            using type = _transition_table<>;
        };

        template <template <typename> typename _Condition, typename Item0, typename... Items>
        struct _transition_filter<_Condition, Item0, Items...> {
#define _Complement typename _transition_filter<_Condition, Items...>::type
            /* Decide if `Item0` should be included. */
            using type = typename std::conditional< //
                _Condition<Item0>::value,
                typename _transition_table_concat<Item0, _Complement>::type, /* include `Item0` */
                _Complement /* exclude `Item0` */
                >::type;
#undef _Complement
        };

    } // namespace detail

    /**
     * @brief Module with Finite State Mechian (fsm) model.
     */
    template <typename State>
    class fsm {
    public:
        ///< Return the state machine's current state.
        State get_current_state() const { return m_state; }

    protected:
        fsm(State init_state)
            : m_state(init_state)
        {
            static_assert(
                std::is_trivially_copyable<State>::value, "`State` should be trivially copyable.");
        }

        fsm(const fsm &) = delete;

        ~fsm() {}

    public:
        bool is_transition() const { return m_nested > 0; }

    protected:
        /**
         * @brief Costom-end post event to FSM, which will cause the transition.
         * @tparam Event
         * @param event
         * @return State New state of the transistion.
         */
        template <typename Object, typename Event>
        State post_event(const Event &event)
        {
            processing_lock lock(*this);
            // Assert this class is derived from fsm.
            static_assert(std::is_base_of<fsm, Object>::value, "Must derive from fsm.");
            // Filter `transition_table` by current event type.
            using items = typename do_transition_filter< //
                Event, typename Object::transition_table>::type;
            // Do action on the event.
            Object &self = static_cast<Object &>(*this);
            return do_transition<Object, Event, items>::execute(self, event, m_state);
        }

        /**
         * @brief Costom-end post event to FSM, which will cause the transition.
         * @tparam Event
         * @param event
         * @return State New state of the transistion.
         */
        template <typename Object>
        bool test_connection(State to)
        {
            // Assert this class is derived from fsm.
            static_assert(std::is_base_of<fsm, Object>::value, "Must derive from fsm.");
            // Filter `transition_table` by current event type.
            using items = typename Object::transition_table;
            return test_transition<Object, items>::execute(m_state, to);
        }

    public:
        ////////////////////////////////////////////////////////////////
        // A base class of transistion from `from` state to `to` state with `event`.
        ////////////////////////////////////////////////////////////////
        template <typename Object, typename State from, typename Event, typename State to>
        class _transition {
        public:
            using state_t = State;
            using event_t = Event;

            static constexpr state_t from_state() { return from; }

            static constexpr state_t to_state() { return to; }

        protected:
            // template <typename Action>
            // static void do_event(Action &&action, Derived &self, const Event &event) {
            //    detail::do_action(action, self, event);
            //}

            // template <typename Guard>
            // static bool do_guard(Guard &&guard, const Derived &self, const Event &event) {
            //    return detail::do_action(guard, self, event);
            //}

            // For no object != derived (secondary derived)
            template <typename Object, typename Action>
            static void do_event(Action &&action, Object &self, const Event &event)
            {
                detail::do_action(action, self, event);
            }

            // For no object != derived (secondary derived)
            template <typename Object, typename Guard>
            static bool do_guard(Guard &&guard, const Object &self, const Event &event)
            {
                return detail::do_action(guard, self, event);
            }

            // For no object
            static void do_event(std::nullptr_t, Object &, const Event &) {}

            // For no object
            static constexpr bool do_guard(std::nullptr_t, const Object &, const Event &)
            {
                return true;
            }

            //// For no object
            // static void do_event(std::nullptr_t, Derived &, const Event &) {}

            //// For no object
            // static constexpr bool do_guard(std::nullptr_t, const Derived &, const Event &) {
            //    return true;
            //}
        };

        // public:
        //    template <typename State from,
        //              typename Event,
        //              typename State to,
        //              void (Derived::*action)() = nullptr,
        //              bool (Derived::*guard)() const = nullptr>
        //    struct transition : public _transition<from, Event, to> {

        //        static void do_event(Derived &self, const Event &event) {
        //            if (action != nullptr) {
        //                _transition<from, Event, to>::do_event(action, self, event);
        //            }
        //        }

        //        static bool do_guard(const Derived &self, const Event &event) {
        //            if (guard != nullptr) {
        //                return _transition<from, Event, to>::do_guard(guard, self, event);
        //            } else {
        //                return true;
        //            }
        //        }
        //    };

        // public:
        //    template <typename State from,
        //              typename Event,
        //              typename State to,
        //              void (Derived::*action)(const Event &) = nullptr,
        //              bool (Derived::*guard)(const Event &) const = nullptr>
        //    struct transition_e : public _transition<from, Event, to> {

        //        static void do_event(Derived &self, const Event &event) {
        //            if (action != nullptr) {
        //                _transition<from, Event, to>::do_event(action, self, event);
        //            }
        //        }

        //        static bool do_guard(const Derived &self, const Event &event) {
        //            if (guard != nullptr) {
        //                return _transition<from, Event, to>::do_guard(guard, self, event);
        //            } else {
        //                return true;
        //            }
        //        }
        //    };

    public:
        template <typename Object, typename State from, typename Event, typename State to,
            void (Object::*action)() = nullptr, bool (Object::*guard)() const = nullptr>
        struct transition_x : public _transition<Object, from, Event, to> {

            static void do_event(Object &self, const Event &event)
            {
                if (action != nullptr) {
                    _transition<Object, from, Event, to>::do_event(action, self, event);
                }
            }

            static bool do_guard(const Object &self, const Event &event)
            {
                if (guard != nullptr) {
                    return _transition<Object, from, Event, to>::do_guard(guard, self, event);
                } else {
                    return true;
                }
            }
        };

    public:
        template <typename Object, typename State from, typename Event, typename State to,
            void (Object::*action)(const Event &) = nullptr,
            bool (Object::*guard)(const Event &) const = nullptr>
        struct transition_xe : public _transition<Object, from, Event, to> {

            static void do_event(Object &self, const Event &event)
            {
                if (action != nullptr) {
                    _transition<Object, from, Event, to>::do_event(action, self, event);
                }
            }

            static bool do_guard(const Object &self, const Event &event)
            {
                if (guard != nullptr) {
                    return _transition<Object, from, Event, to>::do_guard(guard, self, event);
                } else {
                    return true;
                }
            }
        };

    protected:
        template <class Event>
        State no_transition(const Event &event)
        {
            //// NOTE: this case means some transitions are not defined.
            // std::string info
            //    = std::string("No transition for event [") + typeid(event).name() + "].";
            // throw std::exception(info.c_str());
            return get_current_state();
        }

    protected:
        template <class Event, typename...>
        struct do_transition_filter;

        ///< For those no valid filtered items.
        template <typename Event>
        struct do_transition_filter<Event, detail::_transition_table<>> {
            using type = detail::_transition_table<>;
        };

        ///< Instance filter that depending on event type.
        template <typename Event, typename... Items>
        struct do_transition_filter<Event, detail::_transition_table<Items...>> {
            template <typename Item>
            using predicate = std::is_same<typename Item::event_t, Event>;
            using type = typename detail::_transition_filter<predicate, Items...>::type;
        };

    protected:
        template <typename Object, typename Event, typename...>
        struct do_transition;

        ///< For those unimplemented transition.
        template <typename Object, typename Event>
        struct do_transition<Object, Event, detail::_transition_table<>> {
            static State execute(Object &self, const Event &event, State)
            {
                return self.no_transition(event);
            }
        };

        template <typename Object, typename Event, typename Item0, typename... Items>
        struct do_transition<Object, Event, detail::_transition_table<Item0, Items...>> {
            static State execute(Object &self, const Event &event, State state)
            {
                using _Complement = typename detail::_transition_table<Items...>;
                if (state == Item0::from_state() && Item0::do_guard(self, event)) {
                    /* Do current event if start state is same and guard succeed. */
                    Item0::do_event(self, event);
                    return Item0::to_state();
                } else {
                    /* Other wise do the rest events. */
                    return do_transition<Object, Event, _Complement>::execute(self, event, state);
                }
                /* So, generally there shound not be two trasitions that both have same `from` and
                 * `action`. */
            }
        };

    protected:
        template <typename Object, typename...>
        struct test_transition;

        ///< For those unimplemented transition.
        template <typename Object>
        struct test_transition<Object, detail::_transition_table<>> {
            static bool execute(State, State) { return false; }
        };

        template <typename Object, typename Item0, typename... Items>
        struct test_transition<Object, detail::_transition_table<Item0, Items...>> {
            static bool execute(State from, State to)
            {
                using _Complement = typename detail::_transition_table<Items...>;
                if (from == Item0::from_state() && to == Item0::to_state()) {
                    return true;
                } else {
                    /* Other wise do the rest events. */
                    return test_transition<Object, _Complement>::execute(from, to);
                }
                return false;
            }
        };

    protected:
        class processing_lock {
        public:
            processing_lock(fsm &m)
                : nested(m.m_nested)
            {
                if (nested > 0) {
                    throw std::logic_error("Do not post another event in trasistion process.");
                }
                nested++;
            }

            ~processing_lock() { nested--; }

        private:
            std::atomic<size_t> &nested;
        };

    protected:
        std::atomic<State> m_state;
        std::atomic<size_t> m_nested = 0;
    };

} // namespace fsmlib
