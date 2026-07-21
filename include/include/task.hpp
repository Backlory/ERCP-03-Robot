#pragma once
#if defined(_WINDOWS) || defined(WINDOWS) || defined(_WIN32) || defined(WIN32)
#ifdef min
#pragma push_macro("min")
#undef min
#define _NO_MIN
#endif

#ifdef max
#pragma push_macro("max")
#undef max
#define _NO_MAX
#endif
#endif

#include <map>
#include <vector>
#include <thread>
#include <future>
#include <memory>
#include <iostream>
#include <functional>
#include <type_traits>
#if !defined(USING_TBB) || !USING_TBB
#error "TBB library is need!"
#endif
#include <tbb/concurrent_map.h>

using namespace std::placeholders;

namespace task {

    enum task_status : int {
        errored = -1,
        ideal = 0,
        running = 1,
        passed = 2,
        failed = 3,
        skipped = 4,
    };

    using report_t = tbb::concurrent_map<std::string, int>;

    template <typename... Args>
    class _TaskBase : public std::enable_shared_from_this<_TaskBase<Args...>> {
    public:
        using fn_t = std::function<bool(Args...)>;
        using this_type = _TaskBase<Args...>;

    public:
        _TaskBase(std::string name, const fn_t &&fn, bool skipable = true, int verbose = 0)
            : m_name(name)
            , m_task(fn)
            , m_skipable(skipable)
            , m_verbose(verbose)
        {
        }

        _TaskBase(std::string name)
            : m_name(name)
        {
        }

        _TaskBase(const _TaskBase &task) = delete;

        std::string get_name() const { return m_name; }

        fn_t get_task() const { return m_task; };

        virtual void report(report_t &ret) { ret.emplace(m_name, ideal); }

        virtual bool is_busy() const { return m_running; }

        virtual bool run(report_t &ret, Args... args)
        {
            m_running = true;
            // Skip finished task.
            if (m_skipable) {
                if (ret.find(m_name) != ret.end()
                    && (ret.at(m_name) == passed || ret.at(m_name) == skipped)) {
                    ret.at(m_name) = skipped;
                    return true;
                }
            }
            if (ret.find(m_name) == ret.end()) {
                ret.emplace(m_name, ideal);
            }
            // Run the task.
            bool res = false;
            ret.at(m_name) = running;
            try {
                m_counter++;
                res = m_task ? m_task(args...) : false;
                ret.at(m_name) = res ? passed : failed;
            } catch (...) {
                ret.at(m_name) = errored;
                m_error = std::current_exception();
            }
            m_running = false;
            return res;
        }

        virtual std::string dump(int level, bool sequential)
        {
            return std::string(level * 4, ' ') + (sequential ? "->" : "==") + m_name + "\n";
        }

        virtual void update(std::string name, bool prefix = true)
        {
            if (prefix)
                m_name = name + "/" + m_name;
            else
                m_name = name;
        }

        virtual const std::exception_ptr get_error() const { return m_error; }

        virtual size_t get_count() const { return m_counter; }

        virtual const std::shared_ptr<this_type> get_task(std::string name)
        {
            if (name == m_name) {
                return this->shared_from_this();
            }
            return nullptr;
        }

    protected:
        std::atomic<size_t> m_counter = { 0 };
        std::atomic_bool m_running = { false };
        std::string m_name;
        std::exception_ptr m_error;
        const fn_t m_task = nullptr;
        const bool m_skipable = true;
        const int m_verbose = 0;
    };

    ///////////////////////////////////////////////////////////////////////////

    template <typename... Args>
    class _TasksBase : public _TaskBase<Args...> {
    public:
        using base_type = _TaskBase<Args...>;
        using task_t = _TaskBase<Args...>;

        _TasksBase(std::string name)
            : _TaskBase<Args...>(name)
        {
        }

        void emplace(std::string name, task_t::fn_t &&fn, bool skipable = true, int verbose = 0)
        {
            m_tasks.emplace_back(std::make_shared<task_t>(
                task_t::m_name + "/" + name, std::move(fn), skipable, verbose));
        }

        void emplace(std::shared_ptr<_TasksBase<Args...>> t)
        {
            t->update(this->m_name, true);
            m_tasks.push_back(t);
        }

        const std::shared_ptr<base_type> get_task(std::string name) override
        {
            for (const auto &task : m_tasks) {
                auto ptr = task->get_task(name);
                if (ptr) {
                    return ptr;
                }
            }
            return nullptr;
        }

        size_t get_count() const override
        {
            size_t count = 0;
            for (const auto &task : m_tasks) {
                auto cnt = task->get_count();
                if (cnt > count) {
                    count = cnt;
                }
            }
            return count;
        }

        virtual void report(report_t &ret) override
        {
            for (auto &task : m_tasks) {
                task->report(ret);
            }
        }

        virtual bool is_busy() const
        {
            for (auto &task : m_tasks) {
                if (task->is_busy()) {
                    return true;
                }
            }
            return false;
        }

        virtual bool run(report_t &ret, Args... args) override { return task_t::run(ret, args...); }

        virtual std::string dump(int level, bool sequential) override
        {
            return task_t::dump(level, sequential);
        }

        virtual void update(std::string name, bool prefix = true)
        {
            _TaskBase<Args...>::update(name, prefix);
            for (auto &q : m_tasks) {
                q->update(name, true);
            }
        }

        const std::exception_ptr get_error() const override
        {
            auto err = _TaskBase<Args...>::get_error();
            if (!err) {
                for (auto &q : m_tasks) {
                    err = q->get_error();
                    if (err) {
                        break;
                    }
                }
            }
            return err;
        }

    protected:
        std::vector<std::shared_ptr<task_t>> m_tasks;
    };

    template <typename... Args>
    class SequentialTasks : public _TasksBase<Args...> {
    public:
        using base_t = _TasksBase<Args...>;

        SequentialTasks(std::string name)
            : _TasksBase<Args...>(name)
        {
        }

        bool run(report_t &ret, Args... args) override
        {
            for (auto &task : base_t::m_tasks) {
                {
                    std::lock_guard<decltype(m_mutex)> lock(m_mutex);
                    m_current = task;
                }
                if (!task->run(ret, args...)) {
                    return false;
                }
            }
            return true;
        }

        std::string dump(int level = 0, bool sequential = true) override
        {
            std::string info = _TasksBase<Args...>::dump(level, sequential);
            for (auto &task : base_t::m_tasks) {
                info += task->dump(level + 1, true);
            }
            return std::move(info);
        }

        const std::shared_ptr<typename base_t::task_t> current()
        {
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            return m_current;
        }

    private:
        std::mutex m_mutex;
        std::shared_ptr<typename base_t::task_t> m_current;
    };

    template <typename... Args>
    class ParallelTasks : public _TasksBase<Args...> {
    public:
        using base_t = _TasksBase<Args...>;

        ParallelTasks(std::string name)
            : _TasksBase<Args...>(name)
        {
        }

        bool run(report_t &ret, Args... args) override
        {
            bool global = true;
            std::vector<std::future<bool>> promices;
            // Creat async tasks
            for (auto &task : base_t::m_tasks) {
                promices.push_back(std::async(std::launch::async,
                    std::bind(&base_t::task_t::run, task.get(), std::ref(ret), args...)));
            }
            // Wait for all async tasks return
            for (auto &prom : promices) {
                global = global && prom.get();
            }
            return global;
        }

        std::string dump(int level = 0, bool sequential = false) override
        {
            std::string info = _TasksBase<Args...>::dump(level, sequential);
            for (auto &task : base_t::m_tasks) {
                info += task->dump(level + 1, false);
            }
            return std::move(info);
        }
    };

    template <typename... Args>
    using tasks_ptr = std::shared_ptr<_TaskBase<Args...>>;
    template <typename... Args>
    using const_tasks_ptr = std::shared_ptr<const _TaskBase<Args...>>;

    template <typename... Args>
    using seque_ptr = std::shared_ptr<SequentialTasks<Args...>>;

    template <typename... Args>
    using paral_ptr = std::shared_ptr<ParallelTasks<Args...>>;

} // namespace task

#ifdef _NO_MIN
#pragma pop_macro("min")
#undef _NO_MIN
#endif

#ifdef _NO_MAX
#pragma pop_macro("max")
#undef _NO_MAX
#endif
