from setuptools import find_packages, setup

package_name = 'dqn_robot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel',
    maintainer_email='manuel.mamani@ucb.edu.bo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dqn_agent = dqn_robot_nav.dqn_agent:main',
            'environment = dqn_robot_nav.environment:main',
            'state_processor = dqn_robot_nav.state_processor:main',
            'test_node = dqn_robot_nav.test_node:main',
            'train_node = dqn_robot_nav.train_node:main',
        ],
    },
)
