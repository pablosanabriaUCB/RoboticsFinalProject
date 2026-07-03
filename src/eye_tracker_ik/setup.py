from setuptools import find_packages, setup

package_name = 'eye_tracker_ik'

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
    description='Nodo de cinemática inversa numérica para brazo de 6-DOF',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ik_node = eye_tracker_ik.ik_node:main',
        ],
    },
)
