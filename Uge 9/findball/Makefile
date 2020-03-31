CC = g++
CFLAGS = -g -Wall
SRCS = ./src/*.cpp
PROG = OpencvTest
BIN = ./bin/

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o ${BIN}$(PROG) $(SRCS) $(LIBS)

clean:
	-rm $(BIN)/*
