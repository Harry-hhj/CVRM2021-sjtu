//
// Created by hhj on 2021/1/20.
// Author's Homepage: https://github.com/Harry-hhj
// Last Modified: 2021/1/20
//

#ifndef CVRM2021_MAIN_H
#define CVRM2021_MAIN_H


#include <string>
#include "umt/umt.hpp"

struct A{
    std::string str;

    A() = default;  // default constructer
    A(std::string _str): str(std::move(_str)){}  // customized constructer

    // member function
    void print();
};

/*
 * func1() and func2() communicate in c++
 * func3() is a publisher sending msg into python
 * func4() is a subscriber reading msg from python
 */
void func1();
void func2();
void func3();
void func4();


#endif //CVRM2021_MAIN_H
