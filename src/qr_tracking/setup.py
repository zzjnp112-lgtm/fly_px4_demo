from setuptools import setup
from setuptools import find_packages

package_name = 'qr_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzj',
    maintainer_email='zzjnp112@gmail.com',
    description='视觉追踪节点：检测目标并发布像素坐标到 zzjnp',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yuan = qr_tracking.yuan:main',
        ],
    },
)
