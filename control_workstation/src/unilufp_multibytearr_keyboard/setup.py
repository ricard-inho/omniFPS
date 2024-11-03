from setuptools import find_packages, setup

package_name = 'unilufp_multibytearr_keyboard'

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
    maintainer='admin51',
    maintainer_email='ricardmarsalicastan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unilufp_multibytearr_keyboard = unilufp_multibytearr_keyboard.main:main'
        ],
    },
)
