CC = g++

all: main 
main:
	$(CC) -O3 -I . -lglut -lGLU -o as4 as4.cpp
clean: 
	rm as4