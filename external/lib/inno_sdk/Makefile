###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##
.PHONY: all
all: build

.PHONY: build
ifndef MINGW64
build: utils ws_utils sdk_common sdk_client sdk
else
build: utils sdk_common sdk_client sdk
endif

.PHONY: build_client
ifndef MINGW64
build_client: utils sdk_common sdk_client
else
build_client: utils sdk_common sdk_client
endif

.PHONY: utils
utils:
	@$(MAKE) -C $@

.PHONY: ws_utils
ws_utils:
	@$(MAKE) -C $@

.PHONY: sdk_common
sdk_common: | utils
	@$(MAKE) -C $@

.PHONY: sdk_client
sdk_client: | sdk_common
	@$(MAKE) -C $@

.PHONY: sdk
sdk: | sdk_client
	@$(MAKE) -C $@

.PHONY: clean
clean:
	@$(MAKE) -C utils clean
	@$(MAKE) -C ws_utils clean
	@$(MAKE) -C sdk_common clean
	@$(MAKE) -C sdk_client clean
	@$(MAKE) -C sdk clean

.PHONY: test
test:
	@$(MAKE) -C utils test

