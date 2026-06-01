import os
from glob import glob
from setuptools import setup

package_name = "semantic_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
        (os.path.join("share", package_name, "web"),
         glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Agentic semantic reasoning + web dashboard for TurtleBot3.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "semantic_tagger = semantic_nav.semantic_tagger:main",
            "semantic_map = semantic_nav.semantic_map:main",
            "semantic_query = semantic_nav.semantic_query:main",
            "query_and_go = semantic_nav.query_and_go:main",
            "web_backend = semantic_nav.web_backend:main",
        ],
    },
)
