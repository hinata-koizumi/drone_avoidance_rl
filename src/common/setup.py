from setuptools import setup
import os
from glob import glob

package_name = 'common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools==59.6.0'],
    zip_safe=True,
    maintainer='Hinata Koizumi',
    maintainer_email='example@example.com',
    description='共通ユーティリティ・ベースクラス',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}'])
    ],
) 