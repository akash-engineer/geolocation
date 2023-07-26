###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

STATIC_LIB = libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a
DYNA_LIB = libinnolidarsdkclient.so libinnolidarsdkcommon.so libinnolidarutils.so
SRC_DIR = ../../apps/parse
OBJ_DIR = ../../obj/parse
DEP_DIR = ../../dep/parse
LIB_DIR = ../../lib
TARGETS = parse_pcap
CPPLINT = ../../build/cpplint.py
SRC_PYTHON = parse_pcap_python.cpp
SRC_C = parse_pcap.cpp
CC ?= gcc
CXX ?= g++
STRIP ?= strip
CFLAGS = -fPIC -O2 -g2 -Wall -Werror -fpermissive -std=c++11 $(INC_DIR)
LINKFLAGS = -Wl,--whole-archive -lpthread -Wl,--no-whole-archive -Wl,-Bstatic -static
DYNA_LINKFLAGS = -pthread
INC_DIR = -I../ -I../../ -I../../src/
INNO_CLIENT_LIBS = -linnolidarsdkclient -linnolidarsdkcommon -linnolidarutils

OTHER_LIBS = -ldl -lstdc++ -lm -lpcap  -pthread
STATIC_LIB_FILES := $(patsubst %, $(LIB_DIR)/%, $(STATIC_LIB))
.PHONY: all
all: build

ifeq ($(ARCH_TAG), -arm)
build: lint
else ifeq ($(ROSRELEASE), kinetic)
build: lint parse_pcap
else
build: inno_falcon_b_parse.so lint parse_pcap
endif
parse_pcap :$(SRC_C) $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) $^ -o $@ -I ./ $(OTHER_LIBS)


inno_falcon_b_parse.so : $(SRC_PYTHON) $(STATIC_LIB_FILES)
	g++ -O3 -Wall -shared -std=c++11 -I ../../src -I 3rdparty/eigen-git-mirror -I 3rdparty/pybind11/include -fPIC -Wl,-undefined,dynamic_lookup `python3-config --includes` $^ -o $@

.PHONY : clean
clean:
	rm -f *.so* *.o lint_checked
	
	
.PHONY: lint
lint: lint_checked

lint_checked: $(SRC_PYTHON) $(SRC_C)
	$(CPPLINT) --root= $?
	touch lint_checked