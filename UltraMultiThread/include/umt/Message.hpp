#ifndef _UMT_MESSAGE_HPP_
#define _UMT_MESSAGE_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace umt {

    /**
     * @brief 消息异常类型
     */
    class MessageError : public std::runtime_error {
    protected:
        using std::runtime_error::runtime_error;
    };

    /**
     * @brief 消息异常类型，当前消息上无publisher
     */
    class MessageError_Stopped : public MessageError {
    public:
        MessageError_Stopped() : MessageError("no publisher on this message!") {}
    };

    /**
     * @brief 消息异常类型，消息读取超时
     */
    class MessageError_Timeout : public MessageError {
    public:
        MessageError_Timeout() : MessageError("message read timeout!") {}
    };

    /**
     * @brief 消息异常类型，空消息（未初始化或使用过std::move）
     */
    class MessageError_Empty : public MessageError {
    public:
        MessageError_Empty() : MessageError("empty message. maybe uninitailized or moved!") {}
    };

    template<class T>
    class Publisher;

    template<class T>
    class Subscriber;

    namespace utils {

        template<class T>
        class MessagePipe {
            friend class Publisher<T>;

            friend class Subscriber<T>;

        public:
            using MsgType = T;
        private:
            std::mutex pubs_mtx;
            std::list<Publisher<T> *> pubs;
            std::mutex subs_mtx;
            std::list<Subscriber<T> *> subs;
        };
    }

    /**
    * @brief 消息订阅器类型
    * @details 使用队列存储收到的消息，可以设置队列最大长度，当超出最大队列长度时，新消息会覆盖最老的消息
    * @tparam T 消息对象类型
    */
    template<class T>
    class Subscriber {
        friend Publisher<T>;
    private:
        using MsgManager = ObjManager<utils::MessagePipe<T>>;
    public:
        using MsgType = T;

        Subscriber() = default;

        /**
         * @details 构造函数
         * @param msg_name 消息名称
         * @param max_fifo_size 最大消息长度
         */
        explicit Subscriber(const std::string &msg_name, size_t size = 1) : fifo_size(size) {
            bind(msg_name);
        }

        /// 拷贝构造函数
        Subscriber(const Subscriber &other) : fifo_size(other.fifo_size), fifo(other.fifo), p_msg(other.p_msg) {
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.emplace_front(this);
        }

        /// 移动构造函数
        Subscriber(Subscriber &&other) noexcept: fifo_size(other.fifo_size), fifo(std::move(other.fifo)),
                                                 p_msg(other.p_msg) {
            other.reset();
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.emplace_front(this);
        }

        /// 析构函数
        ~Subscriber() { reset(); }

        /// 判断当前订阅器是否绑定到某个消息
        explicit operator bool() {
            return p_msg != nullptr;
        }

        /// 重置订阅器
        void reset() {
            if (!fifo.empty()) fifo = std::queue<T>();
            if (!p_msg) return;
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.remove(this);
            p_msg.reset();
        }

        /**
         * @brief 绑定当前订阅器到某个名称的消息
         * @param msg_name 消息名称
         */
        void bind(const std::string &msg_name) {
            reset();
            p_msg = MsgManager::find_or_create(msg_name);
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.emplace_front(this);
        }

        /**
         * @brief 清空接收缓冲区
         */
        void clear() {
            std::unique_lock lock(mtx);
            fifo = std::queue<T>();
        }

        /**
         * @brief 设置队列长度，size==0则不限制最大长度
         * @param size 最大队列长度
         */
        void set_fifo_size(size_t size) {
            fifo_size = size;
        }

        /**
         * @brief 读取当前最大队列长度
         * @return 当前最大队列长度
         */
        size_t get_fifo_size() {
            return fifo_size;
        }

        /**
         * @brief 尝试获取一条消息
         * @details 如果当前消息上没有发布器，则会抛出一条异常
         * @return 读取到的消息
         */
        T pop() {
            if (!p_msg) throw MessageError_Empty();
            std::unique_lock lock(mtx);
            cv.wait(lock, [this]() { return p_msg->pubs.empty() || !fifo.empty(); });
            if (p_msg->pubs.empty()) throw MessageError_Stopped();
            T tmp = std::move(fifo.front());
            fifo.pop();
            return tmp;
        }

        /**
         * @brief 尝试获取一条消息，有超时时间
         * @details 如果当前消息上没有发布器，则会抛出一条异常；如果超时，也会抛出一条异常
         * @param ms 超时时间，单位毫秒
         * @return 读取到的消息
         */
        T pop_for(size_t ms) {
            if (!p_msg) throw MessageError_Empty();
            using namespace std::chrono;
            std::unique_lock lock(mtx);
            if (!cv.wait_for(lock, milliseconds(ms), [this]() { return p_msg->pubs.empty() || !fifo.empty(); })) {
                throw MessageError_Timeout();
            }
            if (p_msg->pubs.empty()) throw MessageError_Stopped();
            T tmp = std::move(fifo.front());
            fifo.pop();
            return tmp;
        }

        /**
         * @brief 尝试获取一条消息，直到某个时间点超时
         * @details 如果当前消息上没有发布器，则会抛出一条异常；如果超时，也会抛出一条异常
         * @param pt 超时时间点，为std::chrono::time_point类型
         * @return 读取到的消息
         */
        template<class P>
        T pop_until(P pt) {
            if (!p_msg) throw MessageError_Empty();
            std::unique_lock lock(mtx);
            if (!cv.wait_until(lock, pt, [this]() { return p_msg->pubs.empty() || !fifo.empty(); })) {
                throw MessageError_Timeout();
            }
            if (p_msg->pubs.empty()) throw MessageError_Stopped();
            T tmp = std::move(fifo.front());
            fifo.pop();
            return tmp;
        }

    private:
        void write_obj(const T &obj) {
            std::unique_lock lock(mtx);
            if (fifo_size > 0 && fifo.size() >= fifo_size) {
                fifo.pop();
            }
            fifo.push(obj);
        }

        void notify() const {
            cv.notify_one();
        }

    private:
        mutable std::mutex mtx;
        mutable std::condition_variable cv;
        size_t fifo_size{};
        std::queue<T> fifo;
        typename MsgManager::sptr p_msg;
    };

    template<class T>
    class Publisher {
    private:
        using MsgManager = ObjManager<utils::MessagePipe<T>>;
    public:
        using MsgType = T;

        Publisher() = default;

        /**
         * @brief 发布器的构造函数
         * @param msg_name 消息名称
         */
        explicit Publisher(const std::string &msg_name) {
            bind(msg_name);
        }

        /// 拷贝构造函数
        Publisher(const Publisher &other) : p_msg(other.p_msg) {
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.emplace_front(this);
        }

        /// 移动构造函数
        Publisher(Publisher &&other) noexcept: p_msg(other.p_msg) {
            other.reset();
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.emplace_front(this);
        }

        /// 析构函数
        ~Publisher() { reset(); }

        /// 判断当前发布器是否绑定到某个消息
        explicit operator bool() {
            return p_msg != nullptr;
        }

        /// 重置发布器
        void reset() {
            if (!p_msg) return;
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.remove(this);
            if (p_msg->pubs.empty()) {
                std::unique_lock subs_lock(p_msg->subs_mtx);
                for (const auto &sub : p_msg->subs) {
                    sub->notify();
                }
            }
            p_msg.reset();
        }

        /**
         * @brief 绑定当前发布器到某个名称的消息
         * @param msg_name 消息名称
         */
        void bind(const std::string &msg_name) {
            reset();
            p_msg = MsgManager::find_or_create(msg_name);
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.emplace_front(this);
        }

        /**
         * @brief 发布一条消息
         * @param obj 待发布的消息消息
         */
        void push(const T &obj) {
            if (!p_msg) throw MessageError_Empty();
            std::unique_lock subs_lock(p_msg->subs_mtx);
            for (auto &sub : p_msg->subs) {
                sub->write_obj(obj);
                sub->notify();
            }
        }

    private:
        typename MsgManager::sptr p_msg;
    };

}

#define UMT_EXPORT_MESSAGE_ALIAS_WITHOUT_TYPE_EXPORT(name, type, var) \
    PYBIND11_EMBEDDED_MODULE(Message_##name, m) {               \
        using namespace umt;                                    \
        using namespace umt::utils;                             \
        namespace py = pybind11;                                \
        m.def("names", &ObjManager<MessagePipe<type>>::names);  \
        py::class_<Publisher<type>>(m, "Publisher")             \
            .def(py::init<>())                                  \
            .def(py::init<std::string>(), py::arg("msg_name"))  \
            .def("reset", &Publisher<type>::reset)              \
            .def("bind", &Publisher<type>::bind)                \
            .def("push", &Publisher<type>::push,                \
                 py::call_guard<py::gil_scoped_release>())      \
            ;                                                   \
        py::class_<Subscriber<type>>(m, "Subscriber")           \
            .def(py::init<>())                                  \
            .def(py::init<std::string, size_t>(),               \
                 py::arg("msg_name"),                           \
                 py::arg("fifo_size") = 0)                      \
            .def("reset", &Subscriber<type>::reset)             \
            .def("bind", &Subscriber<type>::bind)               \
            .def("clear", &Subscriber<type>::clear)             \
            .def("pop", &Subscriber<type>::pop,                 \
                 py::call_guard<py::gil_scoped_release>())      \
            .def("pop_for", &Subscriber<type>::pop_for,         \
                 py::call_guard<py::gil_scoped_release>())      \
            ;                                                   \
    }


#define UMT_EXPORT_MESSAGE_ALIAS(name, type, var)               \
void __umt_init_message_##name(pybind11::class_<type> && var);  \
    PYBIND11_EMBEDDED_MODULE(Message_##name, m) {               \
        using namespace umt;                                    \
        using namespace umt::utils;                             \
        namespace py = pybind11;                                \
        m.def("names", &ObjManager<MessagePipe<type>>::names);  \
        py::class_<Publisher<type>>(m, "Publisher")             \
            .def(py::init<>())                                  \
            .def(py::init<std::string>(), py::arg("msg_name"))  \
            .def("reset", &Publisher<type>::reset)              \
            .def("bind", &Publisher<type>::bind)                \
            .def("push", &Publisher<type>::push,                \
                 py::call_guard<py::gil_scoped_release>())      \
            ;                                                   \
        py::class_<Subscriber<type>>(m, "Subscriber")           \
            .def(py::init<>())                                  \
            .def(py::init<std::string, size_t>(),               \
                 py::arg("msg_name"),                           \
                 py::arg("fifo_size") = 0)                      \
            .def("reset", &Subscriber<type>::reset)             \
            .def("bind", &Subscriber<type>::bind)               \
            .def("clear", &Subscriber<type>::clear)             \
            .def("pop", &Subscriber<type>::pop,                 \
                 py::call_guard<py::gil_scoped_release>())      \
            .def("pop_for", &Subscriber<type>::pop_for,         \
                 py::call_guard<py::gil_scoped_release>())      \
            ;                                                   \
        try {                                                   \
            __umt_init_message_##name(                          \
            py::class_<type, std::shared_ptr<type>>(m, #name)); \
        } catch (...) {}                                        \
    }                                                           \
    void __umt_init_message_##name(pybind11::class_<type>&& var)

#endif /* _UMT_MESSAGE_HPP_ */
