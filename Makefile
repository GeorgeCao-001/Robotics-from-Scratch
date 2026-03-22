CXX := clang++
CXXFLAGS := -g -Wall -O2 -std=c++17 -stdlib=libc++ -MMD -MP

SRC_DIR := cpp
BUILD_DIR := build

CXX_SRCS := $(shell find $(SRC_DIR) -name "*.cpp")

PROGS := $(CXX_SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%)

DEPS := $(PROGS:%=%.d)

all: $(PROGS)

$(BUILD_DIR)/%: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $< -o $@

-include $(DEPS)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
