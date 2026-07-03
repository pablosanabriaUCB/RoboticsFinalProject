from setuptools import find_packages, setup

package_name = 'eye_tracker_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pablo@todo.todo',
    description='Nodo de visión que usa MediaPipe para rastrear la posición del rostro/ojos humanos',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = eye_tracker_vision.vision_node:main',
        ],
    },
)
