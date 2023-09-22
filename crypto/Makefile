CXXFLAGS ?= -std=c++20 -Werror -Wall -Wextra -g
LDFLAGS ?= -g

CXXFLAGS += -MMD -MP

SRCDIR := .
TARGET := crypto

CSRCS   := $(wildcard $(SRCDIR)/*.c)
CXXSRCS := $(wildcard $(SRCDIR)/*.cpp)
OBJS    := $(CSRCS:.c=.o) $(CXXSRCS:.cpp=.o)
DEPS    := $(OBJS:.o=.d)

ifeq ($(V), 1)
	QUIET :=
else
	QUIET := @
endif

all: $(TARGET)

$(TARGET): $(OBJS)
	@echo link $@ ← $^
	$(QUIET)$(CXX) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	@echo build $@ ← $<
	$(QUIET)$(CXX) -c -o $@ $<

clean:
	$(QUIET)rm -f *.o *.d $(TARGET)

.PHONY: clean all

-include $(DEPS)