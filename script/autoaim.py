import AutoAim

AutoAim.background_detection_run(
    onnx_file="../asset/model-opt-4.onnx"
)
AutoAim.background_predict_EKF_run()
