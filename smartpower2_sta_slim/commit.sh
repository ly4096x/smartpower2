git add -u
git commit "$@"
echo "#define __GIT_SHA__ \"$(git rev-parse --short HEAD)\"" > git_version.h
