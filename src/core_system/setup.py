from setuptools import setup

package_name = 'core_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=['core_system'],
    py_modules=['core_system.core_system_node'],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='Core system to manage AMR robot and manipulator using service calls',
    entry_points={
        'console_scripts': [
            'core_system_node = core_system.core_system_node:main',
            'amr_client = core_system.amr_client:main',  # AMR client
        ],
    },
)
