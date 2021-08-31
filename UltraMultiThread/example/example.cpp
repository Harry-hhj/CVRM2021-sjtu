#include <umt/umt.hpp>

struct foo {
    std::string name;
    double value;
};

UMT_EXPORT_OBJMANAGER_ALIAS(foo, foo, c) {
    c.def(pybind11::init<>());
    c.def(pybind11::init<std::string, double>());
    c.def_readwrite("name", &foo::name);
    c.def_readwrite("value", &foo::value);
}

UMT_EXPORT_MESSAGE_ALIAS(foo, foo, c) {
    c.def(pybind11::init<>());
    c.def(pybind11::init<std::string, double>());
    c.def_readwrite("name", &foo::name);
    c.def_readwrite("value", &foo::value);
}

int main(int argc, char *argv[]) {
    return Py_BytesMain(argc, argv);
}
