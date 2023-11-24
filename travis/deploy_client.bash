
if [ -z $TRAVIS_TAG ]
    then echo "Not a tag build, will not upload to gh releases"
    exit 0
fi

set -e

# auth
echo $GH_TOKEN | gh auth login -p https --with-token 

# create release
gh release create $TRAVIS_TAG \
 'build_output/xbot2_gui_client_android_arm64_v8a_signed.apk#Android APK' \
 'build_output/xbot2_gui_client_x86_64.zip#Linux App' \
 -t 'XBot2 GUI Client' \
 -n '' \
 --verify-tag


