from setuptools import find_packages, setup

package_name = 'methane_ai_perception'

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
    maintainer='anjana',
    maintainer_email='anjana@todo.todo',
    description='AI perception and mission decision nodes for methane leak risk publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leak_risk_node = methane_ai_perception.leak_risk_node:main',
            'decision_node = methane_ai_perception.decision_node:main',
            'mission_command_node = methane_ai_perception.mission_command_node:main',
        ],
    },
)
