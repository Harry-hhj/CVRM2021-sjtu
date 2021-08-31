#ifndef _CVRM2021_WEBKITS_HPP_
#define _CVRM2021_WEBKITS_HPP_

struct RangeParam {
    double current_value = 0;
    double min_value = 0;
    double max_value = 255;
    double step_value = 1;
};

class Button {
public:
    bool is_press_once();

    void set_press_once();

private:
    bool is_press = false;
};

struct CheckBox {
    bool checked;
};

#endif /* _CVRM2021_WEBKITS_HPP_ */
