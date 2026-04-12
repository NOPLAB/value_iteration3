# AMD EDF (Embedded Development Framework) 2025.2 build environment
# Yocto-based successor to PetaLinux — no monolithic installer needed.
#
# Usage:
#   make -C petalinux docker-build
#   make -C petalinux edf-setup   # repo init + sync (first time)
#   make -C petalinux edf-build   # bitbake
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# Yocto Project host dependencies (Yocto 5.x / Scarthgap)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates chrpath cpio curl debianutils diffstat \
    file gawk gcc git gnupg iputils-ping less libacl1 liblz4-tool locales lz4 make \
    python3 python3-git python3-jinja2 python3-pexpect python3-pip \
    python3-subunit socat sudo texinfo unzip wget xz-utils zstd \
    && rm -rf /var/lib/apt/lists/*

# Locale (required by bitbake)
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8

# Non-root user (bitbake refuses to run as root)
ARG PUSER=yocto
ARG PUID=1000
RUN useradd -m -u ${PUID} -s /bin/bash ${PUSER} \
    && echo "${PUSER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && ln -sf /bin/bash /bin/sh

# Install Google repo tool
RUN curl -fsSL https://storage.googleapis.com/git-repo-downloads/repo \
    > /usr/local/bin/repo && chmod +x /usr/local/bin/repo

# Workspace mount point
RUN mkdir -p /work/edf/build && chown -R ${PUSER}:${PUSER} /work
WORKDIR /work

USER ${PUSER}

# Git config (required by repo tool)
RUN git config --global user.email "builder@vi-sweep" \
    && git config --global user.name "VI Builder" \
    && git config --global color.ui false

COPY --chown=${PUSER}:${PUSER} entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
