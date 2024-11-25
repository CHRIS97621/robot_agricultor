from setuptools import find_packages
from setuptools import setup

setup(
    name='cap_topicos',
    version='0.0.0',
    packages=find_packages(
        include=('cap_topicos', 'cap_topicos.*')),
)
