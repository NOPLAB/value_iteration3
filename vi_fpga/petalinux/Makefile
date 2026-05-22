REPO_ROOT   := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)
IMAGE_NAME  := vi-edf:2025.2

# Path to .xsa (for machine config generation)
XSA ?=

# Machine name (default: custom Ultra96-V2 config)
MACHINE ?= ultra96v2-vi

# Yocto image recipe
IMAGE ?= edf-linux-disk-image

.PHONY: docker-build docker-shell edf-setup edf-build clean help

help:
	@echo "AMD EDF (Yocto) build targets for Ultra96-V2:"
	@echo ""
	@echo "  make docker-build                    Build Docker image (no installer needed)"
	@echo "  make docker-shell                    Interactive shell in container"
	@echo "  make edf-setup                       repo init + sync EDF layers (first time)"
	@echo "  make edf-setup XSA=<path>            Setup + generate machine config from XSA"
	@echo "  make edf-build                       Build Linux image (WIC + BOOT.BIN)"
	@echo "  make edf-build MACHINE=<name>        Build for specific machine"
	@echo "  make clean                           Remove project and output dirs"
	@echo ""
	@echo "First-time workflow:"
	@echo "  1. make docker-build"
	@echo "  2. make edf-setup XSA=path/to/design.xsa"
	@echo "  3. make edf-build"

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
	$(DOCKER_RUN) bash /work/petalinux/scripts/build.sh \
		--machine "$(MACHINE)" --image "$(IMAGE)"

# ---------- Clean ----------

clean:
	rm -rf edf/ sstate-cache/ downloads/ output/
