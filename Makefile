CC = g++

all: main 
main:
	$(CC) -O3 -I . -o as4 as4.cpp -lGL -lglut
clean: 
	rm as4