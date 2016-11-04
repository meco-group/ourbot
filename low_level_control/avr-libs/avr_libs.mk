# Makefile to be included to build the avr libraries

#AVR_LIBRARY_PATH = $(CURDIR) #/home/maarten/projects/avr_libs

AVR_LIBRARY_CFILES := $(wildcard $(AVR_LIBRARY_PATH)/*/lib/*.c) $(wildcard $(AVR_LIBRARY_PATH)/*/lib/*/*.c)
AVR_LIBRARY_CPPFILES := $(wildcard $(AVR_LIBRARY_PATH)/*/lib/*.cpp) $(wildcard $(AVR_LIBRARY_PATH)/*/lib/*/*.cpp)

AVR_LIBRARY_INCLUDES := $(foreach lib,$(filter %/, $(wildcard $(AVR_LIBRARY_PATH)/*/lib/)), -I$(lib)) $(foreach lib,$(filter %/, $(wildcard $(AVR_LIBRARY_PATH)/*/lib/*/)), -I$(lib))

AVR_LIBRARY_SOURCES := $(AVR_LIBRARY_CFILES:.c=.o) $(AVR_LIBRARY_CPPFILES:.cpp=.o)
#AVR_LIBRARY_OBJS := $(foreach src,$(AVR_LIBRARY_SOURCES), $(BUILDDIR)/$(src))

$(info $$AVR_LIBRARY_PATH is [${AVR_LIBRARY_PATH}])
$(info $$AVR_LIBRARY_CFILES is [${AVR_LIBRARY_CFILES}])
$(info $$AVR_LIBRARY_CPPFILES is [${AVR_LIBRARY_CPPFILES}])
$(info $$AVR_LIBRARY_INCLUDES is [${AVR_LIBRARY_INCLUDES}])
