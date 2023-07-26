###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

.PHONY: build
build: src example

.PHONY: all
all: build

.PHONY: src
src:
	$(MAKE) -C src build_client

.PHONY: example
example: | src
	$(MAKE) -C apps/example

.PHONY: pcs
pcs: | src
	$(MAKE) -C apps/pcs dynamic

.PHONY: ros
ros: | src
	$(MAKE) -C apps/ros

.PHONY: clean
clean:
	$(MAKE) -C src/utils clean
	$(MAKE) -C src/sdk_common clean
	$(MAKE) -C src/sdk_client clean
	$(MAKE) -C apps/example clean
	@! test -f apps/pcs/Makefile || $(MAKE) -C apps/pcs clean_dynamic
	@! test -f apps/ros/Makefile || $(MAKE) -C apps/ros clean
