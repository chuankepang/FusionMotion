from setuptools import find_packages, setup

package_name = 'inspire_hand'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 可选：添加启动文件目录
        # ('share/' + package_name + '/launch', ['launch/inspire_hand.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meng',
    maintainer_email='hanmg@buaa.edu.cn',
    description='Inspire Hand ROS2 Driver for Teleoperation and Autonomous Control',
    license='Apache-2.0',  # 修改为合适的license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 原有的遥操作节点（IMU手套控制）
            "inspire_teleop = inspire_hand.inspire_sub:main",
            
            # 新增的独立控制节点（捏取/释放等动作控制）
            "inspire_controller = inspire_hand.inspire_hand_controller:main",
            
            # 可选：添加测试节点
            # "inspire_tester = inspire_hand.inspire_tester:main",
        ],
    },
)