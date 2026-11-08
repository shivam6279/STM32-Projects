CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../../drivers -I../../app
SRC = test_stepgen_fill.c ../../drivers/stepgen_fill.c ../../app/scurve.c ../../app/motion.c

test_stepgen_fill: $(SRC) ../../drivers/stepgen_fill.h ../../app/scurve.h ../../app/motion.h
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

JOG_SRC = test_jog.c ../../app/jog.c ../../drivers/stepgen_fill.c ../../app/scurve.c ../../app/motion.c

test_jog: $(JOG_SRC) ../../app/jog.h ../../drivers/stepgen_fill.h ../../app/motion.h
	$(CC) $(CFLAGS) -I../../platform -o $@ $(JOG_SRC) -lm

JUNC_SRC = test_junction.c ../../drivers/stepgen_fill.c ../../app/scurve.c ../../app/motion.c

test_junction: $(JUNC_SRC) ../../drivers/stepgen_fill.h ../../app/scurve.h ../../app/motion.h
	$(CC) $(CFLAGS) -o $@ $(JUNC_SRC) -lm

PROBE_SRC = test_probe.c ../../app/probe_detect.c

test_probe: $(PROBE_SRC) ../../app/probe_detect.h
	$(CC) $(CFLAGS) -o $@ $(PROBE_SRC) -lm

PWM_SRC = test_pwm.c ../../drivers/pwm_profile.c

test_pwm: $(PWM_SRC) ../../drivers/pwm.h ../../drivers/pwm_hw.h
	$(CC) $(CFLAGS) -I../../platform -o $@ $(PWM_SRC) -lm

run: test_stepgen_fill test_junction test_probe test_jog test_pwm
	./test_stepgen_fill
	./test_junction
	./test_probe
	./test_jog
	./test_pwm

probe: test_probe
	./test_probe

junction: test_junction
	./test_junction

jog: test_jog
	./test_jog

pwm: test_pwm
	./test_pwm

clean:
	rm -f test_stepgen_fill test_stepgen_fill.exe test_junction test_junction.exe test_probe test_probe.exe test_jog test_jog.exe test_pwm test_pwm.exe

.PHONY: run probe junction jog pwm clean
