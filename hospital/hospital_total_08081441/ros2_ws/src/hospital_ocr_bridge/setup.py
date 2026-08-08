from glob import glob
from setuptools import find_packages, setup

package_name = "hospital_ocr_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", ["config/ocr_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="rokey@example.com",
    description="AMR1 KimSeoul external PaddleOCR verifier and nameplate tracker.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hospital_ocr_node = hospital_ocr_bridge.ocr_node:main",
        ],
    },
)
