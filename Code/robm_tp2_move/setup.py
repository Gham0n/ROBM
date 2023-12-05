from setuptools import find_packages, setup

package_name = 'robm_tp2_move'

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
    maintainer='gui',
    maintainer_email='gui@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avance = robm_tp2_move.avance:main',
            'droite = robm_tp2_move.droite:main',
            'tourne_gauche = robm_tp2_move.tourne_gauche:main',
            'vel_to_motor = robm_tp2_move.vel_to_motor:main',
            "telemetre = robm_tp2_move.telemetre:main",
            "color = robm_tp2_move.color:main",
            "all = robm_tp2_move.sensor_based_control:main"
        ],
    },
)
