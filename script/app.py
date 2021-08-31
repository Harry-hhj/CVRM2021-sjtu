"""
网页脚本
"""
from flask import Flask, Response, render_template, request
import argparse
import bridge

app = Flask(__name__)


@app.route('/')
def index():
    return render_template("index.html",
                           video_names=bridge.get_cvmat_names(),
                           params_info=bridge.get_range_params_info(),
                           buttons_info=bridge.get_buttons_info(),
                           checkboxes_info=bridge.get_checkboxes_info())


@app.route('/video/<name>')
def video(name):
    return render_template("video.html", name=name)


@app.route('/setting/<param_name>', methods=["GET"])
def setting(param_name):
    current_value = float(request.args["current_value"])
    range_param = bridge.get_range_param(param_name)
    if range_param is not None:
        range_param.current_value = current_value
        return str(range_param.current_value)
    else:
        return "empty param"


@app.route('/button/<button_name>', methods=["GET"])
def button_press(button_name):
    button = bridge.get_button(button_name)
    if button is not None:
        button.set_press_once()
    return ""


@app.route('/checkbox/<checkbox_name>', methods=["GET"])
def checkbox_change(checkbox_name):
    print(checkbox_name, request.args["checked"])
    checked = request.args["checked"] == "true"
    checkbox = bridge.get_checkbox(checkbox_name)
    if checkbox is not None:
        checkbox.checked = checked
        print("set checkbox")
    return ""


@app.route('/video_feed/<name>')
def video_feed(name):
    return Response(bridge.get_cvmat_jpegcode(name), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--script", "-s", nargs='+', default=[], help="startup script(s) before start the web app")
    opt = parser.parse_args()
    for script in opt.script:
        print(f"running startup script: '{script}'")
        exec(open(script).read())
    app.run(host="0.0.0.0", port=3000, threaded=True)
