CC=arm-linux-gnueabihf-gcc
TARGET=ap_alink
TEST_TARGET=performance_test/test_wrapper
SRC=ap_alink.c
TEST_SRC=performance_test/test_wrapper.c performance_test/performance_test.c

# Production build (optimized, no debug output)
all:
	$(CC) -O2 -Wall -Wno-unused-result $(SRC) -o $(TARGET) -lpthread -lrt -lm $(LDFLAGS)

# Test build (standalone test wrapper)
test:
	$(CC) -O2 -Wall -Wno-unused-result $(TEST_SRC) -o $(TEST_TARGET) -lpthread -lrt -lm $(LDFLAGS)

# Debug build (with debug output)
debug:
	$(CC) -O2 -Wall -Wno-unused-result -DDEBUG $(SRC) -o $(TARGET)_debug -lpthread -lrt -lm $(LDFLAGS)

# SITL build (with SITL support)
sitl:
	$(CC) -O2 -Wall -Wno-unused-result $(SRC) sitl/sitl_telemetry.c -o $(TARGET)_sitl -lpthread -lrt -lm $(LDFLAGS)

# Build both main app and test wrapper
both: all test

# Build all (main app, test wrapper, and SITL)
all_targets: all test sitl

clean:
	rm -f $(TARGET) $(TARGET)_debug $(TARGET)_sitl $(TEST_TARGET)
