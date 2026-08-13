import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lane_detection_cnn"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # El checkpoint del modelo NO se incluye por defecto (puede pesar
        # varias decenas de MB); ver README.md para donde colocarlo antes
        # de "colcon build".
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jesus Emmanuel Vidal-Cuevas",
    maintainer_email="jvidal@upp.edu.mx",
    description=(
        "Cuarto detector de carril del benchmark: segmentacion U-Net sobre "
        "la imagen IPM, mismo contrato de topics que fcm/sliding_windows/hough."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lane_detection_cnn_node = lane_detection_cnn.lane_detection_cnn_node:main",
        ],
    },
)
