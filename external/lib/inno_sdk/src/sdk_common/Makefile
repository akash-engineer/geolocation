include version_gen.mk

STATIC_LIB = libinnolidarsdkcommon.a
DYNA_LIB = libinnolidarsdkcommon.so

SRC_DIR = ../../src/sdk_common
OBJ_DIR = ../../obj/sdk_common
DEP_DIR = ../../dep/sdk_common
OUT_DIR = ../../lib

CPPLINT = ../../build/cpplint.py
CC ?= gcc
CXX ?= g++
CFLAGS ?= -D_GLIBCXX_USE_CXX11_ABI=0 -g2 -std=gnu++11 -Wall -Werror -fpermissive \
                        $(CUSTOMER_RELEASE_DEF) $(CUSTOMER_SOURCE_DEF)
LINKFLAGS ?= -pthread -Wl,-Bstatic -static
LINKFLAGS_DYNA ?= -pthread
AR ?= ar
OBJCOPY ?= objcopy
INC += -I./ -I ..
OTHER_CFLAGS ?= -fPIC
CFLAGS += $(INC) $(OTHER_CFLAGS)
OTHER_LIB_BUILD_CFLAG ?=

ifeq ($(ARCH_TAG), -mingw64)
INNO_LIB_BUILD_CFLAG ?= -L ../../lib -linnolidarutils
endif

ifeq ($(ARCH_TAG), -arm)
	MARCH_VALUE ?= armv8-a+crc
	MTUNE_VALUE ?= cortex-a53
	CFLAGS += -march=$(MARCH_VALUE) -mtune=$(MTUNE_VALUE) -DARCH_ARM64
else
ifneq ($(ARCH_TAG), -qnx)
	CFLAGS += -msse4.2
endif
endif

SRCS := $(wildcard $(SRC_DIR)/*.cpp $(SRC_DIR)/*/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS = $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))

.PHONY: build
build: lint $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/converter

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)
	mkdir -p $(DEP_DIR)/converter

# pull in dependency info for *existing* .o files
-include $(DEPS)

$(OUT_DIR)/$(STATIC_LIB):: $(OUT_DIR)
$(OUT_DIR)/$(STATIC_LIB):: $(OBJS)
	$(AR) -r -o $@- $^
	$(OBJCOPY) $(OBJCOPY_OPT) $@- $@
	rm -f $@-

$(OUT_DIR)/$(DYNA_LIB):: $(OUT_DIR)
$(OUT_DIR)/$(DYNA_LIB):: $(OBJS)
	$(CC) -shared -Wl,-soname,$(DYNA_LIB).$(DYNA_LIB_MAJ) -o $@- $^ $(INNO_LIB_BUILD_CFLAG) $(OTHER_LIB_BUILD_CFLAG)
	$(OBJCOPY) $(OBJCOPY_OPT) $(OBJCOPY_OPT_DYNA) $@- $@.$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD)
	rm -f $@-
	rm -f $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD) $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	cd -

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp) $(wildcard */*.cpp)
	$(CPPLINT) --root=.. $?
	touch lint_checked

clean:
	rm -fr lint_checked $(OBJ_DIR) $(DEP_DIR) $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)*
