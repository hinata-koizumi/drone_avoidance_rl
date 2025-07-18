from setuptools import setup, find_packages

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(
    name='drone_rl',
    version='0.1.0',
    packages=find_packages(),
    install_requires=requirements,
    author='Koizumi Hinata',
    author_email='hkoizumi1123@gmail.com',
    description='A reinforcement learning environment for drones.',
    # test_suite='tests',
) 