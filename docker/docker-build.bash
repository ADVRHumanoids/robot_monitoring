cp ~/.netrc .
DOCKER_BUILDKIT=1 docker build --secret id=mynetrc,src=.netrc . -t next_ui