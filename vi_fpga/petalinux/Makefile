REPO_ROOT   := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)
IMAGE_NAME  := vi-edf:2025.2

# Path to .xsa (for machine config generation)
XSA ?=

# Machine name (default: custom Ultra96-V2 config)
MACHINE ?= ultra96v2-vi

# Yocto image recipe
IMAGE ?= edf-linux-disk-image

.PHONY: docker-build docker-shell edf-setup edf-build clean

# ---------- Docker image ----------

docker-build:
	docker build -t $(IMAGE_NAME) docker/

# ---------- Run helpers ----------

DOCKER_FLAGS ?= -it
# Named volume for bitbake build dir (Unix sockets don't work on Windows FS)
BUILDVOL := vi-edf-build
define DOCKER_RUN
	docker run --rm $(DOCKER_FLAGS) \
		-v "$(REPO_ROOT):/work" \
		-v "$(BUILDVOL):/work/edf/build" \
		-w /work \
		$(IMAGE_NAME)
endef

docker-shell:
	$(DOCKER_RUN) bash

# ---------- EDF workflow ----------

edf-setup:
	$(DOCKER_RUN) bash /work/petalinux/scripts/setup.sh \
		$(if $(XSA),--xsa "$(XSA)")

edf-build:
	$(DOCKER_RUN) bash /work/petalinux/scripts/build.sh "$(MACHINE)" "$(IMAGE)"

# ---------- Clean ----------

clean:
	rm -rf edf/ sstate-cache/ downloads/ output/
