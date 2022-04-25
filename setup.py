from skbuild import setup

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="f16dynamics",
    version="0.0.1",
    author="Ethan Lew",
    author_email="ethanlew16@gmail.com",
    description="F16 Flight Dynamics",
    long_description=long_description,
    long_description_content_type="text/markdown",
    package_dir={"": "python-bindings"},
    cmake_install_dir='python-bindings/f16dynamics',
    packages=['f16dynamics'],
    python_requires=">=3.6",
)
