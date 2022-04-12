FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-latest AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb & writes it to /main_ws/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-1cabd43

ENTRYPOINT /entrypoint.sh

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-navigation_*_amd64.deb /navigation.deb

RUN apt update && apt install -y --no-install-recommends ./navigation.deb \
	&& rm /navigation.deb
