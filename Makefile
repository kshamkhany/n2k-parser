
BUILD_DIR = build

PREFIX = arm-linux-gnueabihf-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.

CC = $(PREFIX)gcc
CXX = $(PREFIX)g++ 
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

SRC = \
$(wildcard src/*.cpp) \
$(wildcard src/n2k/*.cpp)
 
INCLUDES = \
-Isrc/n2k \
-Isrc

CXXFLAGS =	$(INCLUDES) -O2 -g -Wall -fmessage-length=0 -std=gnu++11

OBJS =  $(addprefix $(BUILD_DIR)/,$(notdir $(SRC:.cpp=.o)))
vpath %.cpp $(sort $(dir $(SRC)))

LIBS = -pthread

TARGET =	n2k

$(BUILD_DIR)/%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@
	
$(TARGET):	 $(BUILD_DIR) $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LIBS)
	
$(BUILD_DIR):
	mkdir $@

all: $(TARGET)

clean:
	rm -rf $(BUILD_DIR) $(TARGET)
