CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../../drivers -I../../app
SRC = test_stepgen_fill.c ../../drivers/stepgen_fill.c ../../app/scurve.c ../../app/motion.c

test_stepgen_fill: $(SRC) ../../drivers/stepgen_fill.h ../../app/scurve.h ../../app/motion.h
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

JOG_SRC = test_jog.c ../../app/jog.c ../../drivers/stepgen_fill.c ../../app/scurve.c ../../app/motion.c

test_jog: $(JOG_SRC) ../../app/jog.h ../../drivers/stepgen_fill.h ../../app/motion.h
	$(CC) $(CFLAGS) -I../../platform -o $@ $(JOG_SRC) -lm

run: test_stepgen_fill test_jog
	./test_stepgen_fill
	./test_jog

jog: test_jog
	./test_jog

clean:
	rm -f test_stepgen_fill test_stepgen_fill.exe test_jog test_jog.exe

.PHONY: run jog clean
