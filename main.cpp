#include <Python.h>

/*
 * 主函数入口，不需要改动
 */
int main(int argc, char *argv[]) {
    wchar_t *w_argv[argc];
    for (int i = 0; i < argc; i++) {
        size_t len = std::mbstowcs(nullptr, argv[i], 0) + 1;
        w_argv[i] = new wchar_t[len];
        std::mbstowcs(w_argv[i], argv[i], len);
    }
    return Py_Main(argc, w_argv);
}

