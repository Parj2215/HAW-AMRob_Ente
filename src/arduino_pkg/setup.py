from setuptools import setup

package_name = 'arduino_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Arduino package for manual motor and servo control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_control_manual = arduino_pkg.arduino_control_manual:main',
        ],
    },
)
