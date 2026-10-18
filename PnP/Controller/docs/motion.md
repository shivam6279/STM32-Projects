# Motion: how a move becomes step pulses

This walks through the motion code from the top (a move in mm) to the bottom (pins toggling),
with the math, and then maps each idea onto the files and functions.

```
 G1 X10 Y5 F6000           G-code (app/gcode.c), or diags 'move X10 Y5 F100'
        |
 MotionMove()              app/motion.c     mm -> steps, limits, S-curve plan, timeline
        |   7 segments per moving axis, all with the same start ticks
 StepQueue()               drivers/stepgen.c
        |   per axis ring buffer (32 segments)
 StepFillWords()           drivers/stepgen_fill.c   segment -> exact step times -> BSRR words
        |   512 words per half buffer, in the DMA interrupt
 TIM6 + DMA1 Stream0       hardware: one 32 bit word into GPIOE->BSRR every 1us
        |
 STEP1-8 / DIR1-8 on PE0-15
```

## 1. The bottom: one timer for 8 axes

All 8 STEP pins and all 8 DIR pins are on port E. GPIOE->BSRR sets and clears any mix of
port E pins in a single 32 bit write: bits 0-15 set a pin, bits 16-31 clear it. So one word
describes everything that happens to all 16 pins at one instant.

TIM6 overflows every 1us (260MHz / 260). Each overflow asks DMA1 Stream0 to copy the next word
of a buffer into GPIOE->BSRR. The buffer is 1024 words in a circle:

```
 step_buf:  [ half A: 512 words ][ half B: 512 words ]
              DMA plays A ............ then B ............ then A again
```

When DMA finishes half A it raises the half-transfer interrupt and moves on to half B. The
interrupt then has 512us (the time B takes to play) to compute the next 512us of words and put
them in A. Same thing the other way round at the end of B. That's the whole real time part:
the timing of every edge comes from the timer, not from code, so there is no jitter beyond the
1us grid.

Rules the fill keeps:
- STEP high for exactly 1 tick: the set bit goes in word i, the clear bit in word i+1.
- Steps on one axis at least 2 ticks apart, so the fastest an axis can go is 500kHz.
- DIR changes at least 2 ticks after a step (STEP has been low for 1 tick), and the next step is
  at least `STEP_DIR_SETUP` ticks after DIR changed.

If the interrupt is ever late (the DMA got all the way round to the half that wasn't refilled),
it stops everything: timer off, DMA off, STEP pins low, drivers disabled, fault = underrun.
Playing old words again would mean steps nobody asked for.

## 2. The middle: segments

The fill works from **segments**. A segment is a stretch of time where an axis moves with
constant jerk, so its position (in steps) is a cubic:

```
 p(t) = p0 + v0*t + a0*t^2/2 + j*t^3/6        t = ticks since the segment's start
```

```c
typedef struct {
	uint64_t start;		// absolute tick it begins
	uint32_t duration;	// ticks
	bool dir;			// DIR level for the whole segment
	double p0;			// steps already done in this move at its start
	double v0, a0, j;	// steps/tick, steps/tick^2, steps/tick^3
} StepSegment;
```

There is no "number of steps" in a segment. The fill puts down a step every time p(t) crosses
a whole number: step 1 at p = 1, step 2 at p = 2, and so on. For each one it solves
p(t) = k for the exact t, then rounds to the nearest tick. That's why:

- **Fractions carry over.** A segment can end at p = 166.67. The next one starts at p0 = 166.67
  and its first step is at p = 167. Nothing is lost or doubled at the joins.
- **Speed is exact on average.** Every step time comes from the formula, not from adding up
  rounded intervals, so there is no drift. Each edge is off by at most half a tick.
- **Axes stay together.** `start` is absolute time. If X and Y get segments with the same start
  and duration, they are locked together for the whole move.

### Solving p(t) = k

p(t) never goes down inside a move (velocity >= 0), so there is exactly one answer between the
previous step's time and the end of the segment. `Solve()` uses Newton's method: from a guess x,
the next guess is `x - (p(x) - k) / v(x)`. It starts from the previous step's time, which is
already very close, so it usually takes 1-2 iterations.

Two cases need care:
- At the very start of a move v = 0, so Newton can't divide by it. Then it halves the range
  instead (bisection) until the speed is useful.
- Near the end of a move the speed goes to 0 again and p(t) gets extremely flat: 6 ticks before
  the end the axis might be 3e-10 steps from its final position. So the solver stops on *time*
  (the time error |p - k| / v is under 0.001 tick), not on position.

Constant rate segments (a0 = j = 0) skip all that: t = (k - p0) / v0.

## 3. The top: planning an S-curve move

### Why S-curves
A trapezoid profile (accelerate at constant rate, cruise, decelerate) has instant jumps in
acceleration at the corners, which are jolts to the machine. An S-curve limits the rate of change
of acceleration (jerk), so acceleration ramps up and down smoothly.

### The 7 phases
For a move from rest to rest, with limits on velocity V, acceleration A and jerk J:

| phase | jerk | what happens | duration |
|---|---|---|---|
| 0 | +J | acceleration ramps 0 -> A | tj |
| 1 | 0 | constant acceleration A | ta |
| 2 | -J | acceleration ramps A -> 0, reach cruise speed | tj |
| 3 | 0 | cruise | tv |
| 4 | -J | deceleration ramps in | tj |
| 5 | 0 | constant deceleration | ta |
| 6 | +J | deceleration ramps out, stopped | tj |

With the phase times picked, everything else follows:

```
 peak accel   A = J*tj
 cruise speed V = A*(tj + ta)
 distance     D = V*(2*tj + ta + tv)
```

(The accel half covers V*(2tj+ta)/2 because its speed curve is symmetric; same for decel.)

### Picking the times (`SCurvePlan`)
1. If V is reachable with full acceleration: tj = A/J, ta = V/A - tj. Otherwise acceleration
   never gets to A: tj = sqrt(V/J), ta = 0.
2. Cruise time tv = D/V - (2tj + ta). If that's negative the move is too short to reach V:
   - Lower the peak speed so the accel + decel use exactly the distance, no cruise.
     With full accel that's V^2/A + V*tj = D.
   - If even that doesn't reach full accel: tj = (D/(2J))^(1/3), ta = 0.
3. Round tj, ta, tv **up** to whole ticks, then recompute J so the distance comes out exactly D:
   J = D / (tj*(tj+ta)*(2tj+ta+tv)). With the times fixed, D is just J times a number, so this is
   exact. Rounding times up only ever lowers J, A and V, so every limit still holds. And now the
   whole profile sits exactly on the 1us tick grid.

### One curve for all axes
The planner doesn't plan each axis separately. It plans one curve u(t) along the path, where
u goes from 0 to 1 (D = 1 in the formulas above). An axis that moves 4000 steps then has
p(t) = 4000*u(t); one that moves 7 steps has p(t) = 7*u(t). Same phase times, same start ticks,
so they start together, stop together and stay on the straight line between the two points.

`SCurveSegments()` walks the 7 phases on u, and for each phase writes one segment with the
current u, u', u'' and the phase's jerk, all multiplied by that axis's step count.

### The limits (`MotionMove`)
Every limit is turned into a limit on u, and the smallest wins:

- Feed F (mm/s along the path): u' <= F / L, L = path length in mm.
- Each axis i moving |d_i| steps: its speed is |d_i|*u', so u' <= v_max_i / |d_i| (in steps).
  Same for a_max and j_max on u'' and u'''.
- Each axis: u' <= 0.2 / |d_i| (200kHz).
- All axes together: u' <= 0.5 / sum|d_i| (500k steps/s total, see below).

### The timeline
Moves run one after another, like G-code lines. The planner keeps `timeline_end`, the tick the
last move finishes. A new move starts at that tick, or at `StepNow() + 1024` if the machine is
idle (StepNow is the next tick the interrupt will fill; 1024 ticks = two half buffers of margin
so the segment is never already in the past).

A move needs 7 free queue slots on every axis it uses, or `MotionMove` returns MOTION_BUSY and
queues nothing. The queues are 32 deep, so about 4 moves can be waiting.

## 4. Limits, and where they come from

`step bench` runs the real fill with the DMA pointed at a dummy word (no pins move) and times
the interrupt with the cycle counter. Measured on this board:

| load | worst refill (budget 512us) |
|---|---|
| 8 axes constant 333kHz (2.7M steps/s) | 334us |
| 8 axes S-curve, 300kHz peak | 512-520us: underrun |
| 8 axes S-curve, 87.5kHz peak (700k steps/s) | 407us |
| 8 axes S-curve, 62.5kHz peak (500k steps/s) | 297us |

An S-curve step costs about 1.1us of CPU (Newton + double math + bookkeeping), a constant rate
step about 0.25us. The pins could do 500kHz per axis, but the interrupt is the real limit, so the
planner keeps a move's combined peak rate at 500k steps/s (worst case ~60% of the CPU in the
interrupt, leaving room for USB, ethernet and CAN). One axis can still go 200kHz.

For scale: 160 steps/mm at 500mm/s is 80kHz.

## 5. Files and functions

| file | what's in it |
|---|---|
| `app/motion.c` | `MotionMove` (plan + queue a move), `MotionIdle`, `MotionPosition` (planned), `MotionActual` (from steps put out), `MotionSetPosition` (G92), axis config table `motion_axes[]` |
| `app/scurve.c` | `SCurvePlan` (phase times + exact jerk), `SCurveSegments` (phases -> per axis segments). Pure math |
| `drivers/stepgen_fill.c` | `StepFillQueue` (add a segment), `StepFillWords` (fill N ticks), `LoadNext` (start a segment, DIR), `NextEdge` / `Solve` (next step time). Pure, runs on the PC too |
| `drivers/stepgen.c` | TIM6 + DMA1 setup, refill interrupt, underrun check, `StepNow`, `StepPosition`, `StepAbort`, `StepDryRun`, ISR timing |
| `app/gcode.c` | G0/G1, M400, M114, M92/M201/M203/M204/M205, M112/M999 on top of `MotionMove` |
| `app/diags.c` | `move`, `pos`, `step` (status, `bench`, `dry on/off`) |
| `test/host/test_stepgen_fill.c` | PC tests of the whole stack |

## 6. How it's tested

`make -C test/host run` compiles motion.c, scurve.c and stepgen_fill.c for the PC, replays every
BSRR word onto a simulated port and checks:

- 1e6 steps at constant rate across 100 segments: every interval exact, no gaps at the joins.
- Single axis S-curves from 1 to 123457 steps: exact counts, every step within 1 tick of an
  independent S-curve evaluation (bisection on a separately written u(t), no shared code).
- A 3 axis diagonal: every axis matches the reference and they all finish on the same tick.
- Back and forth moves, and full speed instant reversals: DIR hold and setup times.
- 300 random moves on 8 axes: every axis ends exactly where it should, nothing late.

The tests were checked by breaking the code on purpose (wrong jerk sign, missing slack at
segment ends, no DIR setup, no DIR hold): each one fails.

On the board: `step bench` (above) and `step dry on` + `move` + `pos` to run real moves with the
pins off.

## 7. Trying it

```
step dry on                 pins off, everything else real
move X10 Y5 F100            absolute mm, F in mm/s
pos                         planned vs actual per axis
step                        per axis position, steps, queue, late segments, fault
step dry off                pins live again
```

## 8. Not done yet

- Real axis numbers: steps/mm, max speed, accel and jerk are placeholders (`motion_axes[]` in
  motion.c).
- STEPPER_EN polarity is still a guess (board.h), so nothing has been run with drivers enabled.
- No lookahead: every move stops at its end. Fine for pick and place; streaming lots of tiny
  moves would start/stop each one.
- Homing, endstops and E-stop.
- The G-code layer (Marlin dialect) on top of MotionMove.
