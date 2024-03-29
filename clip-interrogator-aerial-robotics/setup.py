import os

import pkg_resources
from setuptools import setup, find_packages

setup(
    name = "clip-interrogator-aerial-robotics",
    version = "0.6.0",
    license = 'MIT',
    author = 'HaechanMarkBong',
    author_email = 'haechan.bong@gmail.com',
    url = 'https://github.com/HaeChanBong/clip-interrogator-aerial-robotics/tree/HaechanMarkBong-patch-1',
    description = "Generate a prompt from an aerial image",
    long_description = open('README.md', encoding = 'utf-8').read(),
    long_description_content_type = "text/markdown",
    packages = find_packages(),
    install_requires = [
        str(r)
        for r in pkg_resources.parse_requirements(
            open(os.path.join(os.path.dirname(__file__), "requirements.txt"))
        )
    ],
    include_package_data = True,
    extras_require = {'dev': ['pytest']},
    classifiers = [
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Topic :: Education',
        'Topic :: Scientific/Engineering',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
    keywords = ['blip','clip','prompt-engineering','CLIPSeg','text-to-image'],
)
