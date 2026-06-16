# A1 — Electricity, GPIO & Embedded Basics

**Session time:** ~2 hours
**Assumed knowledge:** none. Every term is defined when first used.
**Goal:** by the end, you can explain — from first principles — what voltage,
current, ground, GPIO, and logic levels actually *are* physically, why "common
ground" is a real rule with real consequences, and why this robot needs *two*
separate computers (ESP32-P4 and Raspberry Pi 5) instead of one.

---

## 1. What Is Electricity, Actually? (Voltage, Current, Resistance, Power)

Before anything about microcontrollers makes sense, you need a physical
intuition for four words that get thrown around constantly: **voltage**,
**current**, **resistance**, **power**.

### 1.1 The water analogy (use this forever)

Imagine a water tank on a tower, connected by a pipe to a water wheel at the
bottom, then back to a reservoir.

| Electrical term | Symbol | Unit | Water analogy |
|---|---|---|---|
| Voltage | V | Volts (V) | The **height** of the water tank — how much "pressure" is pushing water down the pipe |
| Current | I | Amps (A) | The **rate of water flow** through the pipe (liters/second) |
| Resistance | R | Ohms (Ω) | How **narrow/clogged** the pipe is — narrower pipe = more resistance = less flow for the same pressure |
| Power | P | Watts (W) | How much work the water wheel can do per second — depends on *both* how hard the water pushes (pressure/voltage) and how much of it flows (current) |

**Key insight:** voltage by itself does nothing. A 12V battery sitting on a
table, not connected to anything, does nothing — just like a water tank with no
pipe attached does nothing. Things only *happen* (motors spin, LEDs light up,
chips compute) when there's a **complete loop** (a "circuit") for current to
flow around, driven by a voltage difference.

### 1.2 Voltage is always a *difference between two points*

This is the single most important and most-skipped fact in beginner electronics.

You never measure "the voltage of a wire." You measure the voltage **between
two points** — e.g., "this pin is at 3.3V *relative to* this other pin." When
people say "this wire is at 3.3V" they're implicitly comparing it to some agreed
reference point called **ground (GND)**, which is *defined* to be 0V.

Think of it like altitude: you don't say "that mountain is 3000 tall" — you say
"3000m *above sea level*." Sea level is the reference (= ground). Voltage works
the same way. We'll come back to this in Section 3 — it's the entire reason
"common ground" is a rule.

### 1.3 Ohm's Law — the one equation that explains almost everything

```
V = I × R
```

Voltage = Current × Resistance.

Rearranged, it tells you:
- `I = V / R` — for a fixed resistance, more voltage pushes more current through.
- `R = V / I` — if you know the voltage across something and the current through
  it, you can compute its resistance.

**Worked example:** A 12V battery connected across a 12Ω motor winding.
Current = `V / R = 12 / 12 = 1A`. If the motor's resistance were instead 6Ω
(thicker windings, less resistance), the *same* 12V would push `12 / 6 = 2A`
through it — double the current, for the same voltage. More current generally
means more torque/force but also more heat (`P = I²R`).

### 1.4 Power — what actually does work

```
P = V × I
```

Power = Voltage × Current, measured in **Watts**.

**Worked example, tied to this robot:** the main battery is a "3S3P LiPo,
11.1V nominal... ~7800mAh." If the four drive motors together draw 2A at 11.1V,
that's `P = 11.1 × 2 = 22.2W` of power being delivered to the motors. This power
comes *from* the battery's stored chemical energy and is converted *into*
mechanical motion (plus some waste heat).

### 1.5 DC vs AC — and why it doesn't matter much here

- **AC (Alternating Current)** — voltage oscillates back and forth (this is
  what comes out of a wall socket). Used for long-distance power transmission
  and most home appliances.
- **DC (Direct Current)** — voltage stays at a constant polarity (battery
  positive terminal is always positive). Every single thing in this robot —
  the LiPo batteries, the ESP32, the RPi5, the motors, the sensors — runs on
  **DC**. You will not need to think about AC again for this entire project.

---

## 2. Circuits, Ground, and Why "Common Ground" Is a Real Rule

### 2.1 What is a "circuit"?

A circuit is a **closed loop** that current can flow around. Electrons leave the
battery's negative terminal, flow through wires and components, and return to
the battery's positive terminal (conventionally we say "current flows from + to
−" — this is a historical convention, the actual electrons move the other way,
but it doesn't matter for anything you'll do here).

**If the loop is broken anywhere — a disconnected wire, a switch that's open, a
blown fuse — current is zero everywhere in that loop.** This is why a single
loose wire can make an entire subsystem "just not work" with zero error message.

### 2.2 What is "Ground" (GND)?

"Ground" is just **the name we give to the 0V reference point** that a circuit
is measured against. It is *not* magically connected to the actual earth/soil
(that's a different, historical meaning of "ground" from AC mains wiring — not
relevant here). In this robot, "ground" means: the negative terminal of the
battery, and every wire/pin that's directly connected to it.

When a datasheet says "this pin outputs 3.3V when HIGH," it actually means:
**"this pin will be 3.3V higher than this chip's GND pin."** That's it. That's
the whole definition.

### 2.3 The Common Ground Problem — why it's a *real* rule, not superstition

Here's the scenario that makes this concrete, and it's **literally a rule in
this project's wiring spec**:

> *"ESP32-P4 GND and both MDD10A GNDs must share the same negative terminal as
> the main battery. Isolated grounds = undefined logic levels on PWM/DIR pins."*

Let's unpack *why*.

Suppose the ESP32's GND pin and the MDD10A motor driver's GND pin are **not**
physically wired together — maybe they're powered from two completely separate
batteries with no shared wire at all.

- ESP32 outputs "3.3V" on its DIR pin. But that 3.3V is *relative to the ESP32's
  own GND*.
- The MDD10A reads that DIR pin's voltage *relative to the MDD10A's own GND*.
- If the ESP32's GND and the MDD10A's GND are sitting at *different actual
  potentials* (which happens automatically if they're not wired together — two
  isolated batteries have no reason to agree on what "zero" means), then the
  3.3V signal the ESP32 *thinks* it's sending might arrive at the MDD10A looking
  like 1V, or 5V, or something that drifts around unpredictably.

The receiving chip has voltage thresholds for "this counts as a digital HIGH"
and "this counts as a digital LOW" (we'll get to this in Section 3). If the
*reference points disagree*, a signal that was a clean "3.3V = HIGH" at the
source can arrive at the destination as an ambiguous voltage that's neither
reliably HIGH nor LOW — this is exactly what "**undefined logic levels**" means
in the wiring rule above. The result: motors twitching randomly, direction pins
flipping on their own, or the driver chip just ignoring the signal entirely.

**The fix is trivial once you understand the problem:** run one wire connecting
all the GND points together. Now they all agree on what "0V" means, and a 3.3V
signal from the ESP32 is unambiguously 3.3V as measured by the MDD10A too.

### 2.4 This robot's literal ground rail

From the wiring spec:

```
Main Battery (3S3P LiPo)
├── GND → MDD10A #1 GND ┐
│                        ├── COMMON GROUND RAIL ← must share with ESP32 GND
└──────→ MDD10A #2 GND ┘
```

One rail. Every GND pin on every board touches it. This single wire is what
makes every other "3.3V means HIGH" statement in this entire project true.

---

## 3. Digital Logic & Logic Levels

### 3.1 What "digital" means

A **digital** signal only cares about two states: **HIGH** and **LOW**,
representing the binary digits **1** and **0**. Physically, "HIGH" and "LOW"
are just voltage *ranges* — not exact values.

For a 3.3V-logic chip like the ESP32-P4:
- Roughly **0V to ~0.99V** → read as **LOW** (0)
- Roughly **2.31V to 3.3V** → read as **HIGH** (1)
- The gap in between (~1V–2.3V) is an **undefined/forbidden zone** — a real
  signal should never sit there for long, and if it does, the chip's behavior
  is unpredictable (this is literally what "undefined logic level" from Section
  2.3 means at the voltage level).

These exact thresholds vary by chip family, but the *shape* — a LOW band, a
HIGH band, and a forbidden gap between them — is universal.

### 3.2 Logic "families" — 3.3V vs 5V vs 12V, and why they don't mix freely

A **logic family** is just "what voltage this chip's HIGH signals use."

- **ESP32-P4 GPIO pins** → 3.3V logic. A GPIO set HIGH outputs ~3.3V; a GPIO
  reads HIGH if the incoming signal is in the ~2.3–3.3V band.
- **Many older microcontrollers/Arduinos** → 5V logic.
- **The motor power rail in this robot** → ~11–12.8V (the main LiPo's voltage
  range) — but this is **not a logic signal**, it's raw power for spinning
  motors. It's a completely different "domain."

**Why you can't just wire things across domains carelessly:**

1. **3.3V signal → 5V-only input:** A 5V chip might have a HIGH threshold of,
   say, 3.5V minimum. A 3.3V "HIGH" from the ESP32 might fall *short* of that
   threshold and get read as ambiguous/LOW. Signal doesn't register reliably.
2. **5V signal → 3.3V-only input:** Many 3.3V chips are only rated to tolerate
   up to ~3.3V–3.6V on their input pins. Feeding them 5V can **permanently
   damage** the pin (overvoltage). This direction is the dangerous one.
3. **12V motor power → any logic pin:** Obviously destructive. This is why the
   motor driver (MDD10A) exists at all — see 3.3 below.

### 3.3 Why the Cytron MDD10A exists — the "translator" between logic and power

The ESP32-P4's GPIO pins can only ever output ~3.3V at very low current (a few
tens of milliamps) — nowhere near enough to spin a motor, and at the wrong
voltage anyway (motors here run off the ~11–12.8V main battery).

The **MDD10A motor driver** is the device that sits in between:

- **Logic side (VCC, PWM, DIR pins):** powered by 3.3V from the ESP32, reads
  small 3.3V *logic* signals — "what speed and direction should the motor go?"
- **Power side (VM+, motor output terminals):** connected to the ~12V main
  battery and directly to the motor windings — this side carries real current
  capable of spinning a motor.

Internally, the MDD10A uses the small 3.3V logic signal to control switches
(transistors) that connect the motor to the big 12V power rail in the
commanded direction. **The ESP32 never directly touches the 12V/motor-current
power — it only ever sends small, safe, 3.3V control signals.** This
separation — small-signal logic on one side, big power on the other, with a
driver chip translating between them — is a pattern you will see *everywhere*
in embedded systems (motor drivers, relay modules, MOSFET drivers, etc.).

---

## 4. GPIO — General Purpose Input/Output

### 4.1 What "GPIO" means

**GPIO = General Purpose Input/Output.** It's a physical pin on a chip whose
*function* is configurable by software — at any moment, firmware can decide
"this pin is currently an input" or "this pin is currently an output," and (for
many pins) can additionally route it to a special hardware peripheral instead
of plain digital I/O (we'll meet these peripherals in later sessions — PWM
generation, pulse counting, SPI, etc., are all "alternate functions" of GPIO
pins).

"General purpose" is the opposite of a pin with one fixed job (like a power
pin, which is *always* power, never configurable).

### 4.2 Digital Output mode

When a GPIO is configured as a **digital output**, software can set it to
either:
- **HIGH** → the pin's voltage goes to (approximately) the chip's logic supply
  voltage (3.3V for ESP32-P4)
- **LOW** → the pin's voltage goes to (approximately) 0V / GND

This is how the ESP32 controls the MDD10A's `DIR` (direction) pins from the
GPIO assignment table — `GPIO 5 = FL_DIR`, etc. Setting that pin HIGH means
"forward," LOW means "reverse" (or vice versa, depending on wiring — this is
exactly what the "Motor Direction Matrix" in the project log documents and
calibrates).

### 4.3 Digital Input mode

When configured as a **digital input**, the chip continuously reads the voltage
on that pin and reports back HIGH or LOW based on which band (Section 3.1) it
falls into. This is how you'd read a button, a switch, or — at the raw level —
an encoder signal (though this project uses a dedicated hardware peripheral,
PCNT, to read encoders far faster and more reliably than software polling a
GPIO input — that's Session A3).

### 4.4 Analog vs Digital I/O — a quick distinction

Everything in Section 4.2–4.3 is **digital** — only two possible states.

Some pins can *also* do **analog** I/O:
- **ADC (Analog-to-Digital Converter) input** — measures a *continuous* voltage
  (e.g., 0 to 3.3V) and converts it to a number (e.g., 0–4095 for a 12-bit ADC).
  Used for things like reading a battery voltage divider or a potentiometer.
- **DAC (Digital-to-Analog Converter) output** — the reverse: outputs a
  continuous voltage corresponding to a number. Rarely used in this kind of
  robot.

You'll meet a *third* category next session — **PWM** — which is technically a
*digital* output (just HIGH/LOW) but switched on and off so fast that it
*behaves* like an analog voltage to whatever it's driving (a motor, an LED).
That's the whole subject of Session A2 — don't worry about it yet, just know
the category exists.

### 4.5 Pull-up / Pull-down Resistors & Why Floating Inputs Are Dangerous

This is one of those things that seems like arcane trivia until you've been
burned by it once — then you never forget it.

**The problem — a "floating" input:**

Suppose a GPIO is configured as a digital input, but the pin isn't connected to
*anything* — no wire to a sensor, nothing driving it HIGH or LOW. What voltage
does the chip read?

**The answer is: undefined, and it changes randomly.** A disconnected pin acts
like a tiny antenna — it picks up stray electrical noise from nearby wires,
power supplies, even your hand moving near the board. The chip might read HIGH
for a moment, then LOW, then HIGH again, with no pattern. If your code does
`if (gpio_read(PIN) == HIGH) { do_something(); }`, `do_something()` might fire
at random, for no reason you can see in your code.

**The fix — pull-up and pull-down resistors:**

- A **pull-up resistor** connects the pin to the logic supply (3.3V) through a
  large resistance (e.g., 10kΩ). With nothing else connected, the pin reads a
  stable **HIGH** (current trickles through the resistor to hold it at 3.3V,
  but only a tiny amount — that's what the large resistance is for). If
  something *else* (like a button connecting the pin to GND) actively pulls the
  pin LOW, that overpowers the weak pull-up, and the pin reads LOW.
- A **pull-down resistor** is the mirror image: connects the pin to GND through
  a large resistor, so it reads a stable **LOW** by default, until something
  actively pulls it HIGH.

Either way, the point is: **a pin with a pull resistor has a well-defined
default state when nothing else is driving it**, instead of floating randomly.

**Internal vs external pull resistors:** Most modern microcontrollers (ESP32
included) have these resistors *built into the chip*, enabled/disabled by a
single line of configuration code (`gpio_set_pull_mode(...)`, or similar) — no
extra physical resistor needed on the board. You only need an *external*
resistor if you need a specific resistance value the internal one doesn't
provide, or if the internal pull is too weak for your situation.

**Why this matters for this project:** any digital input pin that isn't
*continuously and actively driven* by something else (a sensor output, another
chip's output pin) needs a defined pull state — otherwise you get exactly the
kind of intermittent, hard-to-reproduce glitches that fill debugging logs.

---

## 5. Microcontroller vs Single-Board Computer — ESP32-P4 vs Raspberry Pi 5

This is the conceptual heart of A1, and it directly explains the single biggest
architectural decision in this entire project.

### 5.1 What is a Microcontroller (MCU)?

A **microcontroller** is a single chip that contains:
- A CPU core (or several)
- A small amount of RAM (kilobytes to a few megabytes)
- Flash memory to store the program (your firmware *is* the only program — there's
  no "operating system" underneath it in the traditional sense)
- A bunch of built-in peripherals: GPIO, timers, PWM generators, ADCs, UART/SPI/I2C
  controllers, pulse counters, etc.

When you power on a microcontroller, it starts executing *your* firmware
(almost) immediately — there's no boot menu, no login screen, no background
services competing for CPU time. **Whatever your firmware tells it to do, it
does, in the order and timing your code dictates** (modified only by whatever
RTOS scheduling you've set up — Session A5).

The **ESP32-P4** in this robot is a microcontroller: it runs **FreeRTOS** (a
*real-time operating system* — much more on this in A5, but for now: a tiny
scheduler that lets you run a handful of cooperating tasks with precise timing
guarantees, *not* a general OS like Linux).

### 5.2 What is a Single-Board Computer (SBC)?

A **single-board computer**, like the **Raspberry Pi 5**, is — despite the name
similarity — architecturally much closer to a laptop than to a microcontroller:
- A relatively powerful multi-core CPU (RPi5: quad-core ARM)
- Gigabytes of RAM (this robot's Pi has 8GB)
- Storage (SD card / SSD) holding a full **operating system** — here, **Ubuntu
  24.04**, a full Linux distribution
- Runs **many programs at once** — a process scheduler, networking stack,
  filesystem, dozens of background services, and on top of all that, **ROS2
  (Jazzy)** with potentially dozens of nodes running concurrently

### 5.3 "Real-time" vs "General Purpose" — the idea that explains everything

This is the part people most often get wrong, so let's be precise:

> **"Real-time" does NOT mean "fast."** It means **"the time it takes to
> respond is *guaranteed* to be within a known bound, every single time, no
> exceptions."**

A general-purpose OS like Linux is usually *very fast on average* — but it
makes **no hard guarantees** about any individual task's timing. Linux's
scheduler might, at any moment, decide to spend an extra few milliseconds on a
different process — handling a network packet, running a filesystem
operation, garbage-collecting something, whatever. Most of the time this delay
is imperceptible. But "most of the time" is not good enough for a control loop.

**Why this matters for a wheel's PID loop specifically (preview of A4):**

This robot runs a velocity PID loop for each wheel at **1kHz** — meaning the
loop *must* execute every **1 millisecond**, reliably, forever. If that loop
runs late by even a few milliseconds occasionally:
- The PID controller's math assumes a fixed time step (`dt`) between updates.
  If the *actual* time between updates varies unpredictably, the controller's
  calculations (especially the integral and derivative terms — A4) become
  inaccurate.
- A motor that's supposed to get smooth, continuous correction every 1ms
  instead gets corrections at irregular intervals — 1ms, 1ms, 4ms, 1ms,
  9ms... — which shows up physically as **jerky, unstable, or oscillating
  motion**.

If this PID loop ran *inside ROS2 on the Raspberry Pi*, it would be at the mercy
of: the Linux scheduler, whatever else ROS2 is doing that millisecond (SLAM
processing a LiDAR scan, Nav2 replanning, the EKF running, USB driver
interrupts...), and general OS jitter. **None of that is bounded.** Linux *can*
be configured for "soft real-time" with tuning, but it's never as
deterministic as dedicated hardware running nothing else.

**The ESP32-P4, by contrast, runs almost nothing else.** Its FreeRTOS tasks for
encoder reading and PID control (Session A5 covers this in detail — they're
pinned to a specific CPU core, at high priority, with nothing competing for
that core) execute on a predictable schedule, every time, because there's
nothing else happening on that core that could delay them.

### 5.4 Why this robot has *two* computers — the division of labor

Putting it together:

| | ESP32-P4 (Microcontroller) | Raspberry Pi 5 (SBC) |
|---|---|---|
| **Runs** | FreeRTOS (real-time) | Ubuntu 24.04 + ROS2 Jazzy (general-purpose) |
| **Good at** | Guaranteed, precise timing for a *small, fixed* set of tasks | Flexible, complex, many concurrent programs; rich software ecosystem (ROS2, Python, networking) |
| **Bad at** | Running large/complex software, multitasking many unrelated things | Hard real-time guarantees |
| **This robot's job for it** | Encoder reading (1kHz), wheel PID control (1kHz), serial comms — the things that **must never be late** | SLAM, Nav2, sensor fusion, exploration logic, visualization — things where an occasional late update is *fine* |

**The rule of thumb this project follows:** *"If being a few milliseconds late
makes the robot physically misbehave (jerky motion, unstable control), it
belongs on the MCU. If being briefly late just means a slightly stale map or a
delayed visualization update, it belongs on the SBC."*

This single principle is *why* the firmware's scope is deliberately narrow
("encoder capture, wheel PID, serial command handling, watchdog-backed E-stop" —
nothing else), and *why* literally everything else — sensors, SLAM, navigation,
exploration — lives on the Pi where ROS2's rich ecosystem outweighs the lack of
hard real-time guarantees.

---

## 6. Power Systems — Voltage, Current & This Robot's Actual Power Tree

### 6.1 Battery basics — what does "3S3P" mean?

LiPo (Lithium Polymer) battery packs are built from individual **cells**, each
roughly:
- **Nominal voltage:** 3.7V per cell
- **Max (fully charged) voltage:** 4.2V per cell
- **Min (safe discharge limit):** ~3.0V per cell

Cells are combined two ways:
- **Series (S)** — connecting cells + to −, end to end, **adds their voltages**.
  3 cells in series ("3S") = 3 × 3.7V = **11.1V nominal** (3 × 4.2V = 12.6V max
  — close to this project's stated 12.8V max, small differences are normal
  cell-chemistry variation).
- **Parallel (P)** — connecting same-polarity terminals together **adds their
  capacities** (and current capability) while keeping voltage the same. 3 cells
  in parallel ("3P"), each say 2600mAh, gives 3 × 2600mAh = **7800mAh** total —
  exactly matching this project's spec ("2600mAh cells → 7800mAh").

So **"3S3P"** = 3 cells in series (for voltage) × 3 of those series-groups in
parallel (for capacity) = **9 cells total**, delivering **11.1V nominal /
12.8V max** at **~7800mAh**.

**mAh (milliamp-hours)** is a measure of *capacity* — how much current the
battery can supply for how long. A 7800mAh battery can (roughly) supply 7800mA
(7.8A) for 1 hour, or 1A for ~7.8 hours, before being empty. (Real batteries
aren't perfectly linear like this, but it's a fine mental model.)

### 6.2 Voltage regulation — why a "buck converter" exists

The Raspberry Pi 5 needs a clean, stable **5V** supply (and can draw up to 5A
under load — that's 25W, a lot for an SBC!). But this robot's RPi5 battery is a
**"4S 1200mAh"** pack — 4 cells in series = nominal ~14.8V, way higher than the
5V the Pi needs.

A **buck converter** is a circuit that takes a higher input DC voltage and
converts it down to a lower, *regulated* (stable, constant) output voltage —
in this case, the spec says "→ XL buck converter → 5.12V / 5.1A." "Buck" =
step-down (the opposite, "boost," steps voltage up).

Why not just use a battery that's *already* ~5V? Because:
1. Higher-voltage packs can deliver the same *power* (`P = V×I`) at *lower*
   current — thinner wires, less resistive loss, less heat.
2. A regulator keeps the output voltage **stable** even as the battery's actual
   voltage sags from ~16.8V (full) down toward ~12V (nearly empty) over a
   discharge cycle — the Pi always sees a clean 5V regardless of battery state.
   Without regulation, the Pi would get a slowly-drooping voltage as the
   battery drains, which could cause brownouts/crashes well before the battery
   is actually "empty."

(This is also foreshadowing: your project memory notes a real Pi 5
brownout/reboot incident that turned out to be caused by a USB *cable* not
supporting 5V/5A — i.e., the cable itself acted like an unwanted extra
resistance, causing voltage to sag under load even though the regulator
upstream was fine. You'll cover that story properly in Session B11 — but now
you have the voltage-sag vocabulary to understand *why* a cable rated for less
current causes a voltage drop under load: `V_drop = I × R_cable`, and a thinner/
worse cable has higher `R_cable`.)

### 6.3 This robot's complete power tree, end to end

Putting Sections 6.1–6.2 and the common-ground rule (Section 2.4) together,
here is the **entire** power distribution of this robot:

```
┌─────────────────────────────────────────────────────────────────┐
│  MAIN BATTERY — 3S3P LiPo, 11.1V nominal / 12.8V max, ~7800mAh   │
│  (9 cells: 3 in series × 3 in parallel)                          │
└─────────────────────────────────────────────────────────────────┘
        │ VM+ (~11-12.8V, high current capable)
        ├──────────────► Cytron MDD10A #1 — motor power input
        ├──────────────► Cytron MDD10A #2 — motor power input
        │
        └─ GND ─────────► COMMON GROUND RAIL (Section 2.4)
                              ▲              ▲
                              │              │
                      MDD10A #1 GND   MDD10A #2 GND   ESP32-P4 GND
                                                       (also tied in)

┌─────────────────────────────────────────────────────────────────┐
│  RPi5 BATTERY — 4S, 1200mAh (nominal ~14.8V)                     │
└─────────────────────────────────────────────────────────────────┘
        │
        ▼
   XL Buck Converter (steps ~14.8V down to a regulated 5.12V / 5.1A)
        │
        ▼
   Raspberry Pi 5 (USB-C power input)
        │
        ▼ RPi5 USB-A port → ESP32-P4 USB-C port
   ESP32-P4 receives: 5V POWER (runs the chip) + SERIAL DATA
   (single USB cable carries both — this is the @921600 baud link
    you'll study in Session A6 / B6)

   ESP32-P4 also has a 3V3 (3.3V) output pin, which powers the
   LOGIC side (VCC) of both MDD10A drivers — connecting Section 3.3's
   "translator" concept back to a real pin on a real chip.
```

**Notice how many of this session's concepts are load-bearing in this one
diagram:**
- Series/parallel battery config → the actual voltage/capacity numbers (6.1)
- Buck converter → regulated 5V for the Pi despite a higher-voltage battery (6.2)
- USB carrying both power *and* data → the ESP32 is powered *by* the Pi (5.4's
  "two computers" now have a literal physical/electrical relationship, not just
  a logical one)
- 3.3V logic from the ESP32 → MDD10A logic-side VCC (3.3)
- Common ground rail → the only thing that makes any of the 3.3V/12V signal
  levels described above *mean* what they're supposed to mean (2.3–2.4)

---

## 7. Multimeter Basics (Practical Skills)

You don't need a multimeter to finish this study plan, but if you have one (or
get one — they're cheap, ~$10-15), here's what each function means and when
you'd use it on *this* robot:

- **DC Voltage (V⎓ or V DC)** — measure the voltage between two points.
  *Probes go in **parallel** with (across) what you're measuring — touch one
  probe to each point, don't break the circuit.* Use this to check: "is the
  main battery actually at ~11-12.8V?", "is the buck converter actually
  outputting 5.12V?", "is the ESP32's 3.3V pin actually at 3.3V?"
- **Continuity (often a diode/speaker-symbol setting)** — checks whether two
  points are electrically connected (the meter beeps if resistance is near
  zero). *Use this — with the robot powered OFF — to verify the common ground
  rail: touch one probe to the main battery's GND terminal, the other to the
  ESP32's GND pin. It should beep (continuous = connected). If it doesn't beep,
  Section 2's "undefined logic levels" problem is about to happen to you.*
- **DC Current (A⎓)** — measure current flow. *Probes go in **series**
  (you must break the circuit and put the meter inline) — rarely needed for
  this project and easy to do wrong (wrong setting + measuring current like
  voltage can blow the meter's fuse or damage it), so don't worry about this
  one for now.*
- **Resistance (Ω)** — measure a component's resistance (with it
  disconnected/unpowered). Occasionally useful for checking if a motor winding
  or resistor is the value you expect.

**The single most useful habit from this section:** before connecting a new
board or sensor for the first time, use the continuity check to confirm its
GND is actually tied to the common ground rail, and use the DC voltage check to
confirm the supply pin is getting the voltage you expect (3.3V, 5V, 12V — not
something else) *before* powering on whatever's downstream of it.

---

## 8. Tying It All Together

Here's the full chain of reasoning this session built, start to finish:

1. **Voltage** is a difference between two points; **current** is the flow
   between them; **Ohm's Law** (`V=IR`) and **Power** (`P=VI`) relate them.
2. A circuit only does something when it's a **closed loop**; **ground** is
   just the agreed 0V reference point for that loop.
3. If two boards' grounds aren't tied together, their "0V" references
   disagree, and digital signals between them become **undefined** — this is
   why this robot has one **common ground rail**.
4. **Digital logic** = HIGH/LOW voltage bands; different chips use different
   **logic families** (3.3V here); mixing voltage domains without care causes
   either unreliable signals or permanent damage.
5. The **MDD10A motor driver** exists specifically to let small, safe 3.3V
   **GPIO** signals from the ESP32 control the robot's high-current 12V motor
   power, without the ESP32 ever touching that power directly.
6. **GPIO** pins are software-configurable as digital outputs (drive HIGH/LOW),
   digital inputs (read HIGH/LOW), or analog/special peripherals; **pull-up/
   pull-down resistors** give floating inputs a defined default state.
7. **Microcontrollers** (ESP32-P4/FreeRTOS) give *deterministic* timing for a
   small set of tasks; **SBCs** (RPi5/Ubuntu/ROS2) give *flexible, powerful, but
   non-deterministic* computing. The 1kHz PID loop's hard timing requirement is
   *why* this robot has both, with a clean division of responsibility.
8. The robot's actual **power tree** — 3S3P main battery, 4S RPi5 battery
   through a buck converter, USB bridging power+data to the ESP32, 3V3 logic
   power to the MDD10A's — is a real, traceable application of every concept
   above.

---

## Self-Test Checklist

Before moving to A2, you should be able to do **all** of these, out loud,
without notes:

- [ ] Explain Ohm's Law and Power using the water-tower analogy, then without it.
- [ ] Explain why "this wire is at 3.3V" is meaningless without specifying what
  it's 3.3V *relative to*.
- [ ] Explain, step by step, what goes wrong electrically if the ESP32's GND and
  the MDD10A's GND are not connected — in terms of voltage *thresholds*, not
  just "it breaks."
- [ ] Explain why a floating digital input pin can cause code to behave
  randomly, and how a pull-up/pull-down resistor fixes it.
- [ ] Explain why the ESP32 can't directly drive a motor, and what role the
  MDD10A plays as a "translator."
- [ ] Explain the difference between "fast" and "real-time," using the wheel
  PID loop as the example.
- [ ] Without looking, draw this robot's full power tree from memory — both
  batteries, the buck converter, both MDD10As, the ESP32, and the common
  ground rail.
- [ ] Explain what "3S3P" means and compute the resulting nominal voltage and
  capacity from individual cell specs.

If any of these feel shaky, re-read that section before starting A2 — A2 (PWM
& Motor Drivers) builds *directly* on the logic-level and GPIO concepts from
Sections 3–4.
