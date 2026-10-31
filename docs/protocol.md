# SwiftPro++ host protocol

The serial protocol as implemented by the `SwiftPro-plusplus` firmware (`P2203` → `V5.0.0-pp.1`, `P2204` → `V4.0.5`).

**Source of this document:** `src/{protocol,commands,parse,line,report,motion,kin}.cpp` and `test/golden/*.txt` of
`SwiftPro-plusplus`, branch `teach-limits` at `41537d1` **plus 39 uncommitted modified files** (read 2026-10-31). The
firmware's 20 host tests pass on a copy of that tree. Nothing here has been observed on the arm. The version string
is the same with and without those uncommitted changes, so it does not tell you which of them a flashed arm has.

The firmware's own `docs/PROTOCOL.md` describes the *original* firmware plus decisions; where it disagrees with this
file, the source wins (known case: it says extra trailing text is ignored, and that `M2200` replies at idle. Neither is
true now).

## 1. Transport

115200 8N1, on USART0 (USB) and USARTs 1–3; a reply goes to the port its command came from, events to the port that
last received a byte. RX buffer 256 bytes. Host-side DTR reset behaviour is a property of the board, not of this firmware.

## 2. Line assembly

- A line ends at `\n` or `\r`. `\r\n` is a line plus an empty line, which produces nothing.
- Bytes `<= ' '` are dropped, so all spaces vanish. `/` is dropped. `(…)` and `;…` are comments. Lower case is folded.
- At most **48 characters** after that, including `#N`. Longer: `$N E21` (nothing runs).
- RX overflow: the damaged line gets a bare `E23` and does not run. Its `#N` may have been lost, so **no `$N`**.

Realtime bytes are taken out of the stream wherever they appear, even mid-line:

| Byte | Action | Reply (never has `$N`) |
|---|---|---|
| `0x18` | Stop, as `S1100` | `ok` |
| `!` | Feed hold | `ok @9 V0` (a reply, not an event) |
| `~` | Resume | `ok` |
| `?`, `@` | Discarded | none |

`!`, `~` and `@` therefore cannot appear in any command text.

## 3. Routing

| Line starts with | Route | Reply |
|---|---|---|
| `$` | Refused | bare `E20` |
| `#<1-9 digits>` then `P`/`S` | Async | `$N …` |
| `#<1-9 digits>` then `G`/`M` | Sync queue | `$N …` |
| `#<digits>` then anything else | Refused | `$N E20` |
| `#` with 10 or more digits | Refused | **bare** `E21` |
| `P`/`S` | Async | no `$N` |
| `G`/`M` | Sync queue | no `$N` |
| anything else | Ignored | **no reply** |

`M2200` is routed as async even though it is an M command.

## 4. Reply format and ordering

```
[ "$" N " " ] CODE [ " " FIELD ]* "\n"
```

There is no `\r` (the boot banner has one). Floats are `dtostrf(v, 5, 4)`: 4 decimals, no plus sign.

**Async (P/S, M2200):** answered as soon as they finish, so they overtake sync replies. Encoder queries take a few ms,
and no new line is read meanwhile. Match replies by `$N`, not by order.

**Sync (G/M):**

- FIFO of **20 entries, counting commands that are waiting to run or waiting to reply**. Full: `$N E23` (or bare `E23`
  without `#N`), command not run.
- A motion command starts when the planner has room, so several motion commands run blended.
- A non-motion sync command starts only when motion is idle, the final lost-step check is done, and the buzzer and
  effector stepper are idle. This includes `M2220`–`M2222`.
- Replies are sent in FIFO order, **only when all of that is idle**, so `$N ok` for a move means "finished", and the
  reply to a long move can arrive arbitrarily late.
- `M2126 V1` (off after boot): a motion command's reply is sent on acceptance into the planner. Errors are still
  reported at once; non-motion commands still wait. **Without it, a continuous stream of moves stops being accepted at
  20 unanswered commands, because the arm never goes idle.**

Event order: a move's reply precedes its `@9`, unless a streamed reply was sent long before.

## 5. Error codes

| Code | Meaning | Motion? |
|---|---|---|
| `ok` | Success | |
| `E20` | Unknown or removed command; `$` lines; `M2233` in modes 4/5 | No |
| `E21` | Bad parameter: malformed number, anything after the expected parameters, F ≤ 0, line too long, sequence too long | No |
| `E22` | Target or path outside the joint limits, or unreachable. **Decided before anything moves**, including the whole path of a linear move | No |
| `E23` | Sync queue full, or RX overflow | No |
| `E25` | Aborted by `S1100`, Ctrl-X or lost steps (given to every queued sync command, after the arm stopped), or the reply did not fit | May have moved |
| `E26` | Encoder read failed; position unknown after a failed boot read; effector origin not found; external EEPROM did not answer; `P2244` fault (`E26 V<mask>`) | No |
| `E27` | A move while **any** of the three joints is released, whichever joint it would turn. Nothing is planned | No |

`E24` is never sent.

## 6. Unsolicited lines

### Boot

```
device name : SwiftPro\r\n
soft ver : V5.0.0-pp.1\r\n
api ver : V4.0.5\r\n
work mode : <n>\r\n
@1\n
```

Then `@5 V1` shortly after, on a powered arm, and a 260 ms beep at boot. Queued commands and `M2120`/`M2122`/`M2124`/
`M2125`/`M2126`/`M2213` state do not survive a reboot.

### Events

| Event | Format | When |
|---|---|---|
| `@1` | `@1` | End of boot |
| `@3` | see below | Every `M2120` period. Skipped, but counted in `S`, when a reply would not fit after it |
| `@4` | `@4 N<0 Menu\|1 Play> V<1 click\|2 long ≥1 s>` | On release, after `M2213 V0` |
| `@5` | `@5 V<0\|1>` | Motor power changed |
| `@6` | `@6 N0 V<0\|1>` | Effector limit switch changed (10 ms debounce) |
| `@8` | `@8 N<0 B\|1 L\|2 R> V<tally − encoder, deg>` | Lost steps: more than 1.0° off on 2 readings in a row. Arm brakes as `S1100`; queued commands get `E25`; tally is re-read |
| `@9` | `@9 V0` | Idle after a move, with `M2122 V1`. **Also fires after a move that was stopped** |

```
@3 X<f> Y<f> Z<f> R<f> B<f> L<f> AR<f> M<B|L|R><f> A<ms> T<ms> S<n>
@3 X<f> Y<f> Z<f> R<f> B<f> L<f> AR<f> M<B|L|R>- T<ms> S<n>        (no good encoder reading)
```

| Field | Meaning |
|---|---|
| `X Y Z` | Effector position from the step tally, not the encoders |
| `R` | Wrist commanded angle (modes 4/5: effector stepper) |
| `B` | Base from the tally, **in the −90 frame (0 = straight ahead)**, unlike `P2200` |
| `L`, `AR` | Left and right joints from the tally |
| `M<j>` | One joint measured by its encoder, rotating B, L, R, in the same frame as the tally field |
| `A` | Age of that reading, ms |
| `T` | ms since boot |
| `S` | Report counter, 0–65535 wrapping; a gap means reports were skipped |

Parsers must expect a two-letter prefix (`AR`, `MB`), and the token count varies.

## 7. Units and frames

- **Base, "user" frame:** 90 = straight ahead. Used by `P2200`, `P2206 N0`, `G2202 N0`, `G2206 B`, `M2220`/`M2221`, polar R.
- **Base, "−90" frame:** 0 = straight ahead. Used only by `@3 B` and `@3 MB`.
- **L:** lower arm, up from horizontal. **R:** upper arm, down from horizontal.
- **F:** 1 unit = 80°/min of joint-space vector speed (F50 = 66.7°/s), **not mm/min**. Modal; F100 after boot. Each joint
  is capped at 300°/s. Linear moves no longer clamp F to 100, as the original did. F ≤ 0 is `E21`.
- **Limits:** L −13.8…135.6; base −5…185 (user frame); wrist 0…180; R depends on L, from a table in
  `SwiftPro-plusplus/src/kin.cpp` (−18…112 overall). The table surface is **not** a limit.
- **Geometry:** lower 142.07, upper 158.81, shoulder radius 13.2, shoulder height 107.4 mm; mode 0 offsets front 56.65,
  height 74.55 mm.

## 8. Commands

Parameters are in fixed order except G0/G1, and nothing may follow them (`E21`). `[F]` is optional and modal.

### 8.1 Motion (sync, reply when finished)

| Command | Meaning | Errors |
|---|---|---|
| `G0 [X][Y][Z][F]` | Absolute effector XYZ, one joint-space move, path not straight. Any order; missing axes stay | E21 E22 E26 E27 |
| `G1 [X][Y][Z][F]` | As G0, straight line | E21 E22 E26 E27 |
| `G2201 S R H [F]` | Absolute polar, straight line | E21 E22 E26 E27 |
| `G2202 N<0-2> V [F]` | One joint, joint-space. 0 base, 1 L, 2 R | E21 E22 E26 E27 |
| `G2202 N3 V [F]` | Wrist, 0–180, immediate. F unused and not kept. Modes 4/5: effector stepper 0–360 | E21 E26 |
| `G2204 X Y Z [F]` | Relative Cartesian, straight line | E21 E22 E26 E27 |
| `G2205 S R H [F]` | Relative polar, straight line | E21 E22 E26 E27 |
| `G2206 B L R [F]` | Absolute joints, joint-space | E21 E22 E26 E27 |
| `G2207 N<0-2> F<signed>` | Linear jog X/Y/Z to 1° inside the limits. Sign of F is the direction; F not kept | E21 E22 E26 E27 |
| `G2208 N<0-2> F<signed>` | Joint jog to 1° inside the limits | E21 E22 E26 E27 |
| `G2209 [F]` | Re-read encoders, then move joints that are outside their limits 2° inside | E21 E26 E27 |
| `G2203` | Wait until queued motion is finished | E21 |
| `G2004 P<ms>` | Delay before the next sync command | E21 |

### 8.2 System (async)

| Command | Meaning | Reply |
|---|---|---|
| `S1000 V0` / `V1` | Feed hold / resume | ok, E21 |
| `S1100` | Stop: joints brake, queue aborted with `E25`, tally re-read at rest. Does **not** wedge the firmware. Rest of line ignored | `ok` at once |

### 8.3 Settings and peripherals (sync unless noted)

`*` = writes EEPROM.

| Command | Meaning | Reply |
|---|---|---|
| `M17` | Lock all, re-read position from encoders | ok, E21, E26 |
| `M18`, `M2019` | Release all, wrist too | ok, E21 |
| `M204 A<f>` `*` | Acceleration, A × 80°/s² per joint; 0 < A ≤ 50 | ok, E21 |
| `M2120 V<s>` | `@3` period. V ≤ 0 stops; 0.01–3600 s; otherwise E21. First report one period later | ok, E21 |
| `M2121` | Stop `@3` | ok, E21 |
| `M2122 V<0\|1>` | `@9` on/off | ok, E21 |
| `M2124 V<0\|1>` | Beep when a re-sync corrects a joint by more than 0.5° | ok, E21 |
| `M2125 V<0\|1>` | Lost-step check; on after every boot | ok, E21 |
| `M2126 V<0\|1>` | Streaming replies (§4) | ok, E21 |
| `M2200` | Moving? **Async, answered at once** | `ok V<0\|1>`, E21 |
| `M2201 N<0-3>` | Lock one joint and re-read its encoder; N3 wrist | ok, E21, E26 |
| `M2202 N<0-3>` | Release one; N3 wrist | ok, E21 |
| `M2203 N<0-3>` | Locked? N as in G2202 | `ok V<0\|1>`, E21 |
| `M2205 VUB<1-12 chars>` `*` | Store UID | ok, E21 |
| `M2210 F<16-20000> T<1-32767>` | Buzzer, Hz and ms. Reply after the tone | ok, E21 |
| `M2211 N<0\|1> A<addr> T<1\|2\|4>` | Read AVR (N0) or external (N1) EEPROM | `ok V…`, E21, E26 |
| `M2212 N1 A<addr> T<1\|2\|4> V<f>` `*` | Write external EEPROM | ok, E21, E26 |
| `M2213 V<0\|1>` | V0 buttons report `@4`; V1 default (none) | ok, E21 |
| `M2220 X Y Z` | IK, no legality check | `ok B L R`, E21 |
| `M2221 B L R` | FK | `ok X Y Z`, E21 |
| `M2222 X Y Z P<0\|1>` | Legal target? P1 = polar input | `ok V<0\|1>`, E21 |
| `M2231 V<0\|1>` | Pump | ok, E21 |
| `M2232 V<0\|1>` | Gripper: V1 catch, V0 release | ok, E21 |
| `M2233 V<0\|1>` | Laser | ok, E21, E20 in modes 4/5 |
| `M3`, `M5 [P<0-255>]` | Mode 1 only: **M3 laser off, M5 laser on**. Elsewhere ok, nothing happens | ok, E21 |
| `M2240 N<0-69> V<0\|1>` | Pin level; firmware-owned pins refused | ok, E21 |
| `M2241 N<0-69> V<0\|1>` | Pin direction; firmware-owned pins refused | ok, E21 |
| `M2400 S<0-11>` `*` | Work mode and its offsets. S4/S5 reply after origin search | ok, E21, E26 |
| `M2401 [B]` `*` | Current encoders become the reference (**calibration**) | ok, E21, E26 |
| `M2410 [S<h>]` `*` | Height offset = current Z, or the value given | ok, E21, E26 |
| `M2411 S<f>` `*` | Front offset, ±1000 | ok, E21 |
| `M2412 V<f>` `*` | Effector angle offset, ±360; new origin search in modes 4/5 | ok, E21, E26 |
| `M2413 H<f>` `*` | Height offset, ±1000 | ok, E21 |

Work modes: 0 normal, 1 laser, 2 3D printer, 3 pen, 4 stepper flat, 5 stepper standard, 6 touch pen, 7 user, 8 claw jaw,
9 steering gear, 10 clamp, 11 test. **Up to jss.7, 8 was the test mode.**

### 8.4 Queries (async)

Parameterless queries reject trailing text with `E21`.

| Command | Reply | Source |
|---|---|---|
| `P2200` | `ok B L R` (B user frame), E26 | Encoders |
| `P2201` | `ok VSwiftPro` | Constant |
| `P2202` | `ok V3.3.1` (board straps) | Board |
| `P2203` | `ok V5.0.0-pp.1` | Constant |
| `P2204` | `ok V4.0.5` | Constant |
| `P2205` | `ok V<uid>`; **`ok V` if none stored** | EEPROM |
| `P2206 N<0-3>` | `ok V<f>`, E21, E26. N3 is the commanded wrist angle | Encoder |
| `P2220` | `ok X Y Z`, E26 | Encoders, FK |
| `P2221` | `ok S R H`, E26 | Encoders, FK |
| `P2231` | `ok V<0 off\|1 working\|2 holding>` | Board |
| `P2232` | `ok V<0 released\|1 working\|2 holding>` | Board |
| `P2233` | `ok V<0\|1>` limit switch | Board |
| `P2234` | `ok V<0\|1>` motor power | Board |
| `P2235` | `ok V<0\|1>` laser | Board |
| `P2236 N<0\|1>` | `ok V0` pressed, `V1` released, E21 | Board |
| `P2240 N<0-69>` | `ok V<0\|1>`, E21 | Pin |
| `P2241 N<0-15>` | `ok V<0-1023>`, E21 | ADC |
| `P2242` | `ok B L R` reference counts | EEPROM |
| `P2243` | `ok B L R` raw counts, E26 | Encoders |
| `P2244` | `ok V0`, or `E26 V<mask>`: 4 L, 2 R, 1 base | Encoders |
| `P2245` | `ok` | |
| `P2246` | `ok B<status>,<agc>,<magnitude> L… R…`; `B-` if that chip did not answer | Encoders |
| `P2400` | `ok V<mode>` | RAM |

### 8.5 Removed

All replying `E20`: all Grbl G/M words beyond G0/G1, all `$` commands, `M2123` closed-loop correction, `M2215`, `M2234`,
`M2245`, `M2300`–`M2307`, `M2500` auto-calibration.

## 9. libswiftpro status (libswiftpro 0.3.0, 2026-10-31)

"Old" is the original firmware `V4.9.1-jss.7`, as recorded in the firmware's `docs/PROTOCOL.md`. Status is from the pty
test suite (`tests/`), which checks the library against the protocol text above. Nothing has been run against the arm.

| Command / line | API | Old behaviour | New behaviour | Status |
|---|---|---|---|---|
| `#N` / `$N` framing | all | Same; `#N$…` queued | Same; max 9 digits | ok; seq wraps at 999999999 |
| `M2120 V%.2f` | `set_report_interval` | Any V | 0 < V < 0.01 is E21 | ok; sent with 3 decimals, so too short a period is E21 |
| `M2122` | `set_motion_report` | Same | Same; `@9` also after a stop | ok |
| `M2126` | `set_streaming` | Absent | Streaming replies | ok |
| `M2400 S` | `set_mode` | S0–S8 | S0–S11, 8 is claw jaw, S4/S5 reply after search | ok; 5 s default may be short for S4/S5 (inferred) |
| `M204 A` | `set_acceleration` | Stored in Grbl settings; ≤ 0 ignored | 0 < A ≤ 50, else E21 | ok |
| `G0` | `set_position` | Missing-axis and F quirks | E22 atomic; E26, E27 new; F ≤ 0 E21 | ok |
| `G2204`, `G2201`, `G2205` | `set_position(relative)`, `set_polar` | F clamped ≤ 100; partial move before E22 | No clamp; path checked first | ok; **speed behaviour changed**, documented in the header |
| `G2206` | `set_joint_angles` | Supported | Same | ok |
| `G2202 N V F` | `set_servo_angle`, `set_wrist` | Wrist V wraps | Wrist outside 0–180 is E21 | ok |
| `S1000` | `pause_motion`, `resume_motion` | Same | Same | ok |
| `S1100` | `motion_reset` | Wedged in ALARM during motion | Stops, queue gets E25, stays usable | ok |
| `M17`, `M2019` | `set_servo_attach(-1)`, `set_servo_detach(-1)` | Enable/disable all | Same; re-reads encoders; E26 possible | ok |
| `M2201`, `M2202` | `set_servo_attach`, `set_servo_detach` | M2201 enabled all drivers | One joint; moves E27 until all locked | ok |
| `M2203` | `get_servo_attach` | N mapping mixed up | N as in G2202 | ok (now correct) |
| `M2231`, `M2232` | `set_pump`, `set_gripper` | Same | Same | ok |
| `M2210 F T` | `set_buzzer` | T in half-periods | T in ms, 1–32767 | ok (now correct) |
| `P2220`, `P2221`, `P2200` | `get_position`, `get_polar`, `get_joint_angles` | `ok` with a wrong value on read failure | E26 on read failure | ok |
| `P2206` | `get_servo_angle` | Same | E26 on read failure | ok |
| `M2222` | `is_reachable` | Old limits | New limits; blocked during motion | ok |
| `M2220`, `M2221` | `coord_to_angles`, `angles_to_coord` | Same | Same; blocked during motion | ok |
| `P2231`, `P2232` | `get_pump_status`, `get_gripper_status` | Same | Same | ok |
| `P2244` | `get_encoder_status` | `ok V0` or `E26 V<mask>` | Same; also flags no-ACK and no magnet | ok |
| `M2200` | `get_is_moving` | Reply at idle, so useless | Answered at once | ok (now useful) |
| `P2400` | `get_mode` | Same | Same | ok |
| `P2201`–`P2205` | `get_device_info` | `V4.9.1-jss.7` | `V5.0.0-pp.1`; empty UID `ok V` | ok |
| `M2240`, `M2241`, `P2240`, `P2241` | GPIO | Any pin | Firmware-owned pins E21 | ok |
| `$…` | `send_raw` (comment says Grbl) | Unreachable with `#N` | `$N E20` | ok; reply text is discarded |
| Bare `E23`, bare `E21` | none | Different text | No `$N` | ok; logged. Cannot be attributed, so a sequenced command lost this way fails when the pending map needs its slot, or at `@1` or link loss |
| `E25`, `E27` | all | Absent | New | ok; passed through as strings, but flattened to `-2` by SwiftRos2 |
| `@1` | none | Logged | Same | ok; pending commands fail with `FIRMWARE_RESET` |
| `@3` | `on_position`, `on_joint` | 7 tokens | 10–11 tokens | ok; measured joints, `T`, `S` unused |
| `@4` | none | Absent | New | unused; logged as unknown |
| `@5`, `@6`, `@9` | `on_power`, `on_limit_switch`, `on_motion_complete` | Same | Same | ok |
| `@8` | none | Absent | New | ok; `on_lost_steps` |

Firmware commands with no libswiftpro API (unused): `G1`, `G2004`, `G2203`, `G2207`–`G2209`, `M18`, `M2121`, `M2124`,
`M2125`, `M2205`, `M2211`–`M2213`, `M2233`, `M3`, `M5`, `M2401`, `M2410`–`M2413`, `P2233`–`P2236`, `P2242`, `P2243`,
`P2245`, `P2246`.
