import ObjManager_RangeParam
import ObjManager_Button
import ObjManager_CheckBox
import Message_cvMat
import cv2


def get_range_params_info():
    mutparams = []
    for name in ObjManager_RangeParam.names():
        mp = ObjManager_RangeParam.find(name)
        if mp is not None:
            mutparams.append((name, mp))
    return mutparams


def get_range_param(name):
    return ObjManager_RangeParam.find(name)


def get_buttons_info():
    buttons = []
    for name in ObjManager_Button.names():
        b = ObjManager_Button.find(name)
        if b is not None:
            buttons.append((name, b))
    return buttons


def get_button(name):
    return ObjManager_Button.find(name)


def get_checkboxes_info():
    checkbox = []
    for name in ObjManager_CheckBox.names():
        b = ObjManager_CheckBox.find(name)
        if b is not None:
            checkbox.append((name, b))
    return checkbox


def get_checkbox(name):
    return ObjManager_CheckBox.find(name)


def get_cvmat_names():
    return Message_cvMat.names()


def get_cvmat_jpegcode(name):
    wi = Message_cvMat.Subscriber(name, 1)
    if wi is not None:
        while True:
            jpeg_code = cv2.imencode(".jpeg", wi.pop().get_nparray())[1].tobytes()
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg_code + b'\r\n\r\n'
