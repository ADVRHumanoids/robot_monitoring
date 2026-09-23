FROM hhcmhub/qt6dev:611@sha256:6555b9795852161c7ba43783a6065834c05a32141eda5adfa73646b358478005

# Linux release artifacts target Ubuntu 24.04 and newer. Fail immediately if
# the external Qt build image changes its distribution baseline.
RUN . /etc/os-release \
    && test "$ID" = ubuntu \
    && test "$VERSION_ID" = 24.04

COPY --chown=1000:1000 context/robot_monitoring /home/user/robot_monitoring
COPY --chown=1000:1000 build.bash /home/user/build.bash
