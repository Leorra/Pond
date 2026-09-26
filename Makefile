NVCC        := g++
NVCCFLAGS   := -O3 -std=c++23
INCLUDES    := -I.

TARGET      := main
SRCS        := main.cpp

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(NVCC) $(NVCCFLAGS) $(INCLUDES) $< -o $@ $(LIBS)

clean:
	rm -f $(TARGET)