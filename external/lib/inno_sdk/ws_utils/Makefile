include version_gen.mk

STATIC_LIB = libinnolidarwsutils.a
DYNA_LIB = libinnolidarwsutils.so

SRC_DIR = ../../src/ws_utils
OBJ_DIR = ../../obj/ws_utils
DEP_DIR = ../../dep/ws_utils
OUT_DIR = ../../lib

CPPLINT = ../../build/cpplint.py
CC ?= gcc
CXX ?= g++
CFLAGS ?= -I .. -I ../.. -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -std=gnu++11 -Wall -Werror -fpermissive \
                        $(CUSTOMER_RELEASE_DEF) $(CUSTOMER_SOURCE_DEF)
LINKFLAGS ?= -pthread -Wl,-Bstatic -static
LINKFLAGS_DYNA ?= -pthread
AR ?= ar
OBJCOPY ?= objcopy
INC ?= -I./ $(BOOST_INC)
OTHER_CFLAGS ?= -fPIC
CFLAGS += $(INC) $(OTHER_CFLAGS)

ifeq ($(ARCH_TAG), -arm)
	CFLAGS += -march=armv8-a+crc -mtune=cortex-a53 -DARCH_ARM64
endif

SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS = $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))

.PHONY: build
build: lint $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)

# pull in dependency info for *existing* .o files
-include $(DEPS)

$(OUT_DIR)/$(STATIC_LIB):: $(OUT_DIR)
$(OUT_DIR)/$(STATIC_LIB):: $(OBJS)
	$(AR) -r -o $@- $^
	$(OBJCOPY) $(OBJCOPY_OPT) $@- $@
	rm -f $@-

$(OUT_DIR)/$(DYNA_LIB)::  $(OUT_DIR)
$(OUT_DIR)/$(DYNA_LIB):: $(OBJS)
	$(CC) -shared -Wl,-soname,$(DYNA_LIB).$(DYNA_LIB_MAJ) -o $@- $^
	$(OBJCOPY) $(OBJCOPY_OPT) $(OBJCOPY_OPT_DYNA) $@- $@.$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD)
	rm -f $@-
	rm -f $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD) $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	cd -

$(OBJ_DIR)/%.o:: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp) $(wildcard server_ws/server_ws.hpp)
	$(CPPLINT) --linelength=100 --root=.. $?
	touch lint_checked

clean:
	rm -fr lint_checked $(OBJ_DIR) $(DEP_DIR) $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)*
