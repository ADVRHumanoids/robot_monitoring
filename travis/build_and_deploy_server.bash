set -e

cd server && sudo python3 -m build 

if [ -z $TRAVIS_TAG ]
    then echo "Not a tag build, will not upload to pypi"
else
    twine upload -u __token__ -p $PYPI_TOKEN dist/*
fi
