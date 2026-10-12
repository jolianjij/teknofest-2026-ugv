from glob import glob
from setuptools import find_packages, setup


package_name = "ugv_stage_sign_detector"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/models", glob("models/*.onnx")),
    ],
    install_requires=["setuptools", "numpy", "opencv-python", "Pillow", "PyYAML"],
    zip_safe=True,
    maintainer="UGV Team",
    maintainer_email="team@example.com",
    description="Hybrid CV and ONNX detector for numbered UGV competition stage signs.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ugv-sign-detect = ugv_stage_sign_detector.cli:main",
            "ugv-sign-calibrate = ugv_stage_sign_detector.calibration:main",
            "ugv-generate-signs = ugv_stage_sign_detector.generate_signs:main",
            "ugv-ml-generate-dataset = ugv_stage_sign_detector.ml_dataset:generate_main",
            "ugv-ml-capture = ugv_stage_sign_detector.capture_dataset:capture_main",
            "ugv-ml-train = ugv_stage_sign_detector.ml_train:train_main",
            "ugv-ml-evaluate = ugv_stage_sign_detector.ml_train:evaluate_main",
            "stage_sign_detector_node = ugv_stage_sign_detector.ros_node:main",
            "stage_manager_node = ugv_stage_sign_detector.stage_manager_node:main",
            "stage_action_dispatcher_node = ugv_stage_sign_detector.action_dispatcher_node:main",
            "mock_stage_action_server = ugv_stage_sign_detector.mock_action_server:main",
        ]
    },
)
