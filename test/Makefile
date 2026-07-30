# Host test harness for MMDVM-TNC.
#
#   make            build and run everything
#   make build      build only
#   make run        run everything
#   make run ARGS=--list                list test names
#   make run ARGS=--filter=correlator   run a subset
#   make SAN=1      build with the address and undefined behaviour sanitisers
#   make valgrind   run under valgrind, if it is installed
#   make clean
#
# The firmware sources are compiled unmodified. Only the three hardware
# specific files -- IOSTM.cpp, SerialSTM.cpp, STMUART.cpp -- are left out, and
# shim/Stubs.cpp supplies host versions of the methods they define.

FW  := ..
OBJ := build

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -g -O1
WARNINGS := -Wall -Wextra -Wno-unused-parameter -Wno-reorder -Wno-unused-variable \
            -Wno-unused-but-set-variable -Wno-sign-compare
INCLUDES := -I. -Ishim -I$(FW)
DEFINES  := -DSTM32F4XX

ifeq ($(SAN),1)
  CXXFLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer
  LDFLAGS  += -fsanitize=address,undefined
endif

# Portable firmware sources.
FW_SRCS := Mode2RX.cpp Mode2TX.cpp \
           IL2PRX.cpp IL2PTX.cpp IL2PRS.cpp \
           AX25RX.cpp AX25TX.cpp AX25Frame.cpp AX25CRC.cpp \
           AX25Demodulator.cpp AX25Twist.cpp \
           Hamming.cpp Utils.cpp TokenStore.cpp \
           IO.cpp SerialPort.cpp

# MMDVM.cpp is compiled too, so the harness picks up any change to loop() and
# to the definitions of the firmware globals. Its main() is renamed out of the
# way; the harness supplies its own in framework.cpp.
FW_MAIN := MMDVM.cpp

TEST_SRCS := framework.cpp Radio.cpp shim/arm_math.cpp shim/Stubs.cpp \
             $(sort $(wildcard tests_*.cpp))

FW_OBJS   := $(addprefix $(OBJ)/fw_,$(FW_SRCS:.cpp=.o))
MAIN_OBJ  := $(OBJ)/fw_MMDVM.o
TEST_OBJS := $(addprefix $(OBJ)/,$(TEST_SRCS:.cpp=.o))

BIN := $(OBJ)/tnctest

.PHONY: all build run clean valgrind

all: run

build: $(BIN)

$(BIN): $(FW_OBJS) $(MAIN_OBJ) $(TEST_OBJS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^

$(OBJ)/fw_MMDVM.o: $(FW)/MMDVM.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(WARNINGS) $(INCLUDES) $(DEFINES) -Dmain=mmdvm_main_unused -c -o $@ $<

$(OBJ)/fw_%.o: $(FW)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(WARNINGS) $(INCLUDES) $(DEFINES) -c -o $@ $<

$(OBJ)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(WARNINGS) $(INCLUDES) $(DEFINES) -c -o $@ $<

run: $(BIN)
	@./$(BIN) $(ARGS)

valgrind: $(BIN)
	@command -v valgrind >/dev/null 2>&1 || { echo "valgrind is not installed"; exit 1; }
	valgrind --error-exitcode=9 --errors-for-leak-kinds=none --leak-check=no \
	         --track-origins=yes ./$(BIN) $(ARGS)

clean:
	@rm -rf $(OBJ)
