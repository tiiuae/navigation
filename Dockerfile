# fog-sw BUILDER
FROM ros:foxy-ros-base as fog-sw-builder

ARG BUILD_NUMBER
ARG COMMIT_ID
ARG GIT_VER

ENV ROS_DISTRO_="foxy"

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    python3-bloom \
    fakeroot \
    dh-make \
    ros-${ROS_DISTRO_}-octomap \
    ros-${ROS_DISTRO_}-octomap-msgs \
    ros-${ROS_DISTRO_}-dynamic-edt-3d \
    ros-${ROS_DISTRO_}-laser-geometry \
    ros-${ROS_DISTRO_}-pcl-conversions \
    ros-${ROS_DISTRO_}-pcl-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

COPY . .

RUN params="-m $(realpath .) " \
    && [ ! "${BUILD_NUMBER}" = "" ] && params="$params -b ${BUILD_NUMBER}" || : \
    && [ ! "${COMMIT_ID}" = "" ] && params="$params -c ${COMMIT_ID}" || : \
    && [ ! "${GIT_VER}" = "" ] && params="$params -g ${GIT_VER}" || : \
    && ./packaging/common/package.sh $params

FROM scratch
COPY --from=fog-sw-builder /ros-*-navigation_*.deb /packages/
