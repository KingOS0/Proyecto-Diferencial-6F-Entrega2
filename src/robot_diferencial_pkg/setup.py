from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_diferencial_pkg'


def carpeta_recursiva(origen):
    """Instala una carpeta con subcarpetas (models/...) conservando la estructura."""
    salida = []
    for raiz, _, archivos in os.walk(origen):
        if archivos:
            salida.append((os.path.join('share', package_name, raiz),
                           [os.path.join(raiz, a) for a in archivos]))
    return salida


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ] + carpeta_recursiva('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo 6-F',
    maintainer_email='idekel.plazs@gmail.com',
    description='Robot diferencial skid-steer en Gazebo: AEB, seguimiento de pared, competencia',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_cinematica = robot_diferencial_pkg.nodo_cinematica:main',
            'aeb_node = robot_diferencial_pkg.aeb_node:main',
            'wall_follow = robot_diferencial_pkg.wall_follow:main',
            'mux_cmd = robot_diferencial_pkg.mux_cmd:main',
            'cronometro_vueltas = robot_diferencial_pkg.cronometro_vueltas:main',
        ],
    },
)
