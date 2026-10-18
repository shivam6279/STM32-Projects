CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../../drivers -I../../app
SRC = test_stepgen_fill.c ../../drivers/stepgen_fill.c ../../app/scurve.c ../../app/motion.c

test_stepgen_fill: $(SRC) ../../drivers/stepgen_fill.h ../../app/scurve.h ../../app/motion.h
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

run: test_stepgen_fill
	./test_stepgen_fill

clean:
	rm -f test_stepgen_fill test_stepgen_fill.exe

.PHONY: run clean
