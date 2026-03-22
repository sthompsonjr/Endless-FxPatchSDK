# Makefile — Polyend Endless FxPatch
# Usage: make TOOLCHAIN=/usr/bin/arm-none-eabi-
#        make native         (build active effect for native testing)
#        make tests          (build and run all test harnesses)
#        make effect E=DOD250 (copy effects/PatchImpl_DOD250.cpp to source/PatchImpl.cpp and build)

TOOLCHAIN  ?= arm-none-eabi-
CUSTOM_COMPILER_OPTIONS ?=
PATCH_LOAD_ADDR ?= 0x80000000

CC          = $(TOOLCHAIN)gcc
CXX         = $(TOOLCHAIN)g++
AS          = $(TOOLCHAIN)gcc
LD          = $(CXX)
OBJCOPY     = $(TOOLCHAIN)objcopy

# ── Paths ──────────────────────────────────────────────────────────────────
SDK_DIR     = sdk
DSP_DIR     = dsp
WDF_DIR     = wdf
SOURCE_DIR  = source
BUILD_DIR   = build
EFFECTS_DIR = effects
TESTS_DIR   = tests

# ── Include paths ──────────────────────────────────────────────────────────
# -I. allows #include "dsp/..." and #include "wdf/..." and #include "sdk/..."
# -I./sdk allows bare includes within sdk/ files (PatchABI.h, PatchCppWrapper.h, Patch.h)
# -I./dsp allows #include "CircularBuffer.h" within dsp/ files
# -I./wdf allows #include "WdfPort.h" within wdf/ files
INCLUDES    = -I. -I./$(SDK_DIR) -I./$(DSP_DIR) -I./$(WDF_DIR)

# ── Common flags (preserved from original SDK Makefile) ────────────────────
COMMON_FLAGS := -Wall -Wextra -Werror -fno-builtin -fno-common -ffunction-sections -fdata-sections \
                -fsingle-precision-constant -g -mfloat-abi=hard -mthumb \
                -fstack-usage -specs=nano.specs -Wdouble-promotion -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 \
                -mfp16-format=ieee -O3 -DNDEBUG

CFLAGS      := $(COMMON_FLAGS) -std=c11 $(CUSTOM_COMPILER_OPTIONS)
CXXFLAGS    := $(COMMON_FLAGS) -std=c++20 -includecstdint -felide-constructors -Wno-psabi \
               -fno-exceptions -fno-rtti $(CUSTOM_COMPILER_OPTIONS)

# ── Sources ────────────────────────────────────────────────────────────────
SRC_C       := $(wildcard $(SDK_DIR)/*.c)
SRC_CPP     := $(wildcard $(SOURCE_DIR)/*.cpp)
SRC_CPP     += $(wildcard $(SDK_DIR)/*.cpp)
OBJ         := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRC_C)) \
               $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRC_CPP))

# ── Linker flags ───────────────────────────────────────────────────────────
LDFLAGS     = -nostartfiles --specs=nosys.specs -Wl,-gc-sections -T linker.ld \
              -Wl,--defsym=PATCH_LOAD_ADDR=$(PATCH_LOAD_ADDR) -Wl,--defsym=end=__patch_bss_end

LIBS        := -Wl,--start-group -lstdc++ -lc -lm -lgcc -Wl,--end-group

# ── Platform detection ─────────────────────────────────────────────────────
ifeq ($(OS),Windows_NT)
    RM = rmdir /s /q
    MKDIR = mkdir
    PATCH_TIMESTAMP := $(shell powershell -Command "Get-Date -Format 'yyyyMMdd_HHmmss'")
    FIX_PATH = $(subst /,\,$1)
    NULL_DEVICE = NUL
else
    RM = rm -rf
    MKDIR = mkdir -p
    PATCH_TIMESTAMP := $(shell date +"%Y%m%d_%H%M%S")
    FIX_PATH = $1
    NULL_DEVICE = /dev/null
endif

# ── Output ─────────────────────────────────────────────────────────────────
PATCH_BIN   := $(BUILD_DIR)/patch_$(PATCH_TIMESTAMP).bin

# ── Default target: ARM binary ─────────────────────────────────────────────
.PHONY: all
all: $(PATCH_BIN)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@$(MKDIR) $(call FIX_PATH,$(dir $@)) 2>$(NULL_DEVICE) || exit 0
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	@$(MKDIR) $(call FIX_PATH,$(dir $@)) 2>$(NULL_DEVICE) || exit 0
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/patch.elf: $(OBJ) linker.ld | $(BUILD_DIR)
	$(LD) $(CXXFLAGS) $(OBJ) -o $@ $(LDFLAGS) $(LIBS)

$(PATCH_BIN): $(BUILD_DIR)/patch.elf | $(BUILD_DIR)
	$(OBJCOPY) -O binary $< $@
	@echo "Built: $@"

$(BUILD_DIR):
	@$(MKDIR) $(call FIX_PATH,$(BUILD_DIR)) 2>$(NULL_DEVICE) || exit 0

# ── Native build (for testing on host) ─────────────────────────────────────
.PHONY: native
native:
	g++ -std=c++20 -O2 -Wall -Wextra -Wdouble-promotion \
	    $(INCLUDES) \
	    $(SOURCE_DIR)/PatchImpl.cpp -o $(BUILD_DIR)/patch_native -lm
	@echo "Native build: $(BUILD_DIR)/patch_native"

# ── Effect switcher ────────────────────────────────────────────────────────
# Usage: make effect E=Bitcrush
.PHONY: effect
effect:
	@if [ -z "$(E)" ]; then echo "Usage: make effect E=<EffectName>"; exit 1; fi
	@if [ ! -f "$(EFFECTS_DIR)/PatchImpl_$(E).cpp" ]; then \
	    echo "Effect not found: $(EFFECTS_DIR)/PatchImpl_$(E).cpp"; \
	    echo "Available effects:"; \
	    ls $(EFFECTS_DIR)/PatchImpl_*.cpp 2>/dev/null | sed 's|$(EFFECTS_DIR)/PatchImpl_||;s|\.cpp||'; \
	    exit 1; \
	fi
	cp $(EFFECTS_DIR)/PatchImpl_$(E).cpp $(SOURCE_DIR)/PatchImpl.cpp
	@echo "Active effect: $(E)"
	$(MAKE) all

# ── Tests ──────────────────────────────────────────────────────────────────
.PHONY: tests
tests:
	@bash $(TESTS_DIR)/run_all_tests.sh

# ── Utilities ──────────────────────────────────────────────────────────────
.PHONY: clean
clean:
ifeq ($(OS),Windows_NT)
	@if exist $(BUILD_DIR) $(RM) $(BUILD_DIR)
else
	$(RM) $(BUILD_DIR)
endif

.PHONY: list-effects
list-effects:
	@echo "Available effects:"
	@ls $(EFFECTS_DIR)/PatchImpl_*.cpp 2>/dev/null | sed 's|$(EFFECTS_DIR)/PatchImpl_||;s|\.cpp||' || echo "  (none)"

.PHONY: active-effect
active-effect:
	@echo "Active effect in source/PatchImpl.cpp:"
	@head -5 $(SOURCE_DIR)/PatchImpl.cpp | grep -E '(Effect|Circuit|class|Patch)' || true

.PHONY: headers-check
headers-check:
	@echo "Checking all headers compile clean..."
	@fail=0; \
	for f in $(SDK_DIR)/*.h $(DSP_DIR)/*.h $(WDF_DIR)/*.h; do \
	    [ -f "$$f" ] || continue; \
	    errors=$$(g++ -std=c++20 -I. -I./$(SDK_DIR) -I./$(DSP_DIR) -I./$(WDF_DIR) \
	        -Wall -Wdouble-promotion -fsyntax-only "$$f" 2>&1 \
	        | grep -E 'error:'); \
	    if [ -z "$$errors" ]; then \
	        echo "OK: $$f"; \
	    else \
	        echo "FAIL: $$f"; echo "$$errors"; fail=1; \
	    fi; \
	done; \
	if [ $$fail -eq 1 ]; then exit 1; fi
