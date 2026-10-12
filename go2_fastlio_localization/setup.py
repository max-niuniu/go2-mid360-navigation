from glob import glob
from setuptools import setup
package_name = "go2_fastlio_localization"
setup(
    name=package_name, version="0.2.0", packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Tianwei Niu", maintainer_email="tianwei.niu@cau.edu.cn",
    description="FAST-LIO odometry bridge, PCD ICP localization and map conversion for Go2.",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "fast_lio_odom_bridge = go2_fastlio_localization.fast_lio_odom_bridge:main",
        "pcd_to_nav2_map = go2_fastlio_localization.pcd_to_nav2_map:main",
        "pcd_icp_localizer = go2_fastlio_localization.pcd_icp_localizer:main",
    ]},
)
