from setuptools import setup, find_packages

setup(
    name="leap_v2_utils",  # Name of your package
    version="0.1.0",  # Package version
    packages=find_packages(),  # Automatically find and include all packages
    include_package_data=True,  # Include additional files specified in MANIFEST.in
    description="A utility package for common functions",  # Short description
    url="https://github.com/yourusername/utils",  # URL to your package repository
    author="Alfredo",  # Your name
    author_email="your.email@example.com",  # Your email
    license="MIT",  # License for your package (can be MIT, Apache, etc.)
    install_requires=[],  # Add any external dependencies here (e.g., numpy, etc.)
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',  # Minimum Python version required
)
