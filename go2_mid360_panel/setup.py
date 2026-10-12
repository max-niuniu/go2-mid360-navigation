from glob import glob
from setuptools import setup

package_name = 'go2_mid360_panel'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/assets', glob('assets/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Tianwei Niu', maintainer_email='tianwei.niu@cau.edu.cn',
    description='Desktop supervisor for Go2 simulation, mapping and navigation.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['control_panel = go2_mid360_panel.app:main']},
)
