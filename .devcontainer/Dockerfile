ARG DOCA_VERSION=3.3.0
ARG BASE_IMAGE=nvcr.io/nvidia/doca/doca

FROM ${BASE_IMAGE}:${DOCA_VERSION}-devel

RUN apt-get update && apt-get install -y --no-install-recommends \
        ninja-build \
        pkg-config \
        python3-pip \
        python3-setuptools \
        python3-wheel \
    && pip3 install --no-cache-dir meson \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
