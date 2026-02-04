from setuptools import setup
import os
from glob import glob

package_name = 'final_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.scripts'], # ▼▼▼ 注意：这里要加上子模块 .scripts
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'sdf'), glob('sdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    # ▼▼▼ 修改部分：删掉 scripts=['...'] ▼▼▼
    scripts=[], 
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
    
    # ▼▼▼ 修改部分：启用 entry_points ▼▼▼
    entry_points={
        'console_scripts': [
            # 格式: '运行命令名 = 包名.子文件夹.文件名:函数名'
            'dual_arm_test = final_moveit.scripts.dual_arm_test:main',
            'perception_test = final_moveit.scripts.perception_test:main',
            'dual_arm_tasks = final_moveit.scripts.dual_arm_tasks:main',
            'tic_tac_toe = final_moveit.scripts.dual_arm_tic_tac_toe:main',
            'dual_arm_replay = final_moveit.scripts.dual_arm_replay:main',
            # 如果你有其他脚本，继续往下加
            # 'pick_and_place = final_moveit.scripts.pick_and_place:main',
        ],
    },
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
)