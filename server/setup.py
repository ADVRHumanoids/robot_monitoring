import site
import sys

from setuptools import setup

if __name__ == "__main__":
    site.ENABLE_USER_SITE = "--user" in sys.argv[1:]
    setup(data_files=[
      # ('share/bash-completion/completions/', ['src/forest/forest.bash']),
    ])