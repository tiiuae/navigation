#!/bin/bash

set -euxo pipefail

output_dir=$1

build_number=${GITHUB_RUN_NUMBER:=0}

iname=navigation
docker build \
  --build-arg COMMIT_ID="$(git rev-parse HEAD)" \
  --build-arg GIT_VER="$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)" \
  --build-arg BUILD_NUMBER="${build_number}" -t "${iname}" .

container_id=$(docker create "${iname}" "")
docker cp "${container_id}":/packages .
docker rm "${container_id}"
mkdir -p "$output_dir"
cp packages/*.deb "$output_dir"
rm -Rf packages

exit 0
