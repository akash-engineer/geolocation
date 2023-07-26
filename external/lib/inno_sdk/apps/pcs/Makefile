###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

STATIC_LIB = libinnolidarsdk.a libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a libinnolidarwsutils.a
DYNA_LIB = libinnolidarsdk.so libinnolidarsdkclient.so libinnolidarsdkcommon.so libinnolidarutils.so libinnolidarwsutils.so
SRC_DIR = ../../apps/pcs
OBJ_DIR = ../../obj/pcs
DEP_DIR = ../../dep/pcs
LIB_DIR = ../../lib
TARGETS = inno_pc_server_dynamic inno_pc_client_dynamic
TARGETS_STATIC = inno_pc_server inno_pc_client

CPPLINT = ../../build/cpplint.py
CC ?= gcc
CXX ?= g++
STRIP ?= strip
CFLAGS_ALL = -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -Wall -fpermissive -std=gnu++11 $(INC_DIR)
LINKFLAGS = -Wl,--whole-archive -lpthread -Wl,--no-whole-archive -Wl,-Bstatic -static
DYNA_LINKFLAGS = -pthread
INC_DIR = -I../ -I../../ -I../../src/ $(BOOST_INC)
INNO_LIBS = -linnolidarsdk -linnolidarsdkclient -linnolidarsdkcommon -linnolidarutils -linnolidarwsutils
INNO_CLIENT_LIBS = -linnolidarsdkclient -linnolidarsdkcommon -linnolidarutils -linnolidarwsutils
OTHER_LIBS = $(BOOST_LIB) -lssl -lcrypto -ldl -lstdc++ -lm
ifeq ($(ROSRELEASE), kinetic)
	OTHER_LIBS += -lboost_system
endif
UBUNTU_VER := $(shell lsb_release -rs)
ifeq ($(UBUNTU_VER), 20.04)
    CFLAGS = $(CFLAGS_ALL)
else
    CFLAGS = $(CFLAGS_ALL) -Werror
endif
$(info INC_DIR: $(INC_DIR))
$(info OTHER_LIBS: $(OTHER_LIBS))
$(info CFLAGS: $(CFLAGS))


SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS := $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))
STATIC_LIB_FILES := $(patsubst %, $(LIB_DIR)/%, $(STATIC_LIB))
DYNA_LIB_FILES := $(patsubst %, $(LIB_DIR)/%, $(DYNA_LIB))

INC ?= -I./
OTHER_CFLAGS ?= -fPIC
CFLAGS += $(INC) $(OTHER_CFLAGS)

ifeq ($(ARCH_TAG), -arm)
	CFLAGS += -march=armv8-a+crc -mtune=cortex-a53 -DARCH_ARM64
endif

.PHONY: build
build: lint $(TARGETS_STATIC)

.PHONY: dynamic
dynamic: lint $(TARGETS)

.PHONY: all
all: build dynamic

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)

-include $(DEPS)

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

inno_pc_server_dynamic: $(OBJS) $(DYNA_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS)  -L $(LIB_DIR) $(INNO_LIBS) $(DYNA_LINKFLAGS) $(OTHER_LIBS)
	$(STRIP) $@

inno_pc_server: $(OBJS) $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_LIBS) $(LINKFLAGS) $(OTHER_LIBS)
	cp -f $@ $@_debug
	$(STRIP) $@

inno_pc_client_dynamic: $(OBJS) $(DYNA_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_CLIENT_LIBS) $(DYNA_LINKFLAGS) $(OTHER_LIBS)
	$(STRIP) $@

inno_pc_client: $(OBJS) $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_CLIENT_LIBS) $(LINKFLAGS) $(OTHER_LIBS)
	cp -f $@ $@_debug
	$(STRIP) $@

.SILENT: $(LIB_DIR)/libinnolidarsdk.a
$(LIB_DIR)/libinnolidarsdk.a:
	echo "libinnolidarsdk.a does not exist."
	echo "If you are running 'make' in an SDK distrubution, please run 'make dynamic'."
	echo "Exiting..."
	false

.PHONY: clean
clean:
	rm -rf $(OBJ_DIR) $(DEP_DIR) $(TARGETS) $(TARGETS_STATIC) lint_checked

.PHONY: clean_dynamic
clean_dynamic:
	rm -rf $(OBJ_DIR) $(DEP_DIR) $(TARGETS) lint_checked

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp)
	$(CPPLINT) --root=.. $?
	touch lint_checked
