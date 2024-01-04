from setuptools import setup

setup(
    name='imu_processing_i2c',
    package_dir={'': 'src'},
    packages=['imu_processing_i2c'],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'imu_reader_i2c = imu_processing_i2c.imu_reader_i2c:main',
            'imu_processor_i2c = imu_processing_i2c.imu_processor_i2c:main',
        ],
    },
)
