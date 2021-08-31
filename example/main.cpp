//
// Created by hhj on 2021/1/20.
// Author's Homepage: https://github.com/Harry-hhj
// Last Modified: 2021/1/20
//

#include <thread>
#include <Python.h>
#include "main.h"

void A::print() {
    std::cout << str << std::endl;
}

/*
 * Caution: Be sure to define a Publisher before you define its subscriber,
 * especially when you communicate betweeen c++ and python
 * use sleep_for or try-catch
 */

// inside is a publisher
void func1(){
    // create one publisher
    // format: umt::Publisher<<class type>> variable(<name>)
    umt::Publisher<A> pub1("pub-A");
    /*
     * To define a finite length, declare this param when you create a publisher
     * format: umt::Publisher<<type>> pub1(<name>, length);
     * e.g. When length equals to 1, it means the data is up-to-date
     */

    while (true){
        pub1.push(A("hello, world."));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}


// inside is a subscriber
void func2(){
    // create one subscriber
    // format: umt::Subscriber<<class type>> variable(<name>)
    umt::Subscriber<A> sub1("pub-A");

    while(true) {
        try {
            A a = sub1.pop();
            a.print();
        } catch (...) {
            std::cout << "no msg" << std::endl;
        }
    }
}

// inside is a publisher whose subscriber is in python
void func3(){
    // create one publisher
    // format: umt::Publisher<<class type>> variable(<name>)
    umt::Publisher<A> pub2("pub-A1");

    while (true){
        pub2.push(A("pubA1: hello, world."));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// inside is a subscriber whose publisher is in python
void func4(){
    umt::Subscriber<A> sub2("pub-B1");
    A a;

    while(true){
        try {
            a = sub2.pop();
            a.print();
        } catch(...) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}


int main(int argc, char *argv[]) {
    // before jump into python console, run your function using thread
    std::thread(func1).detach();
    std::thread(func2).detach();
    std::thread(func3).detach();
    std::thread(func4).detach();
    return Py_BytesMain(argc, argv);
}

// register your customized class in python library
UMT_EXPORT_MESSAGE_ALIAS(A, A, c){
    /*
     * for class member variable, use c.def_readwrite() or c.def_readonly()
     * for class member function, use c.def()
     * just take their address ussing '&'
     */

    c.def(pybind11::init<>());
    c.def(pybind11::init<std::string>());
    c.def_readwrite("str", &A::str);
    c.def("print", &A::print);
}