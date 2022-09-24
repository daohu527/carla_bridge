import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="carla_bridge",
    version="0.0.1",
    author="daohu527",
    author_email="daohu527@gmail.com",
    description="Carla to cyber bridge",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/daohu527/carla_bridge",
    project_urls={
        "Bug Tracker": "https://github.com/daohu527/carla_bridge/issues",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "."},
    packages=setuptools.find_packages(where="."),
    install_requires=[
        'protobuf>=3.17.0',
    ],
    entry_points={
        'console_scripts': [
            'carla_bridge = carla_bridge.main:main',
        ],
    },
    python_requires=">=3.6",
)
