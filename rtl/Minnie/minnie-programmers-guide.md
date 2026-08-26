# Minnie Sound Chip — Programmer's Guide

Minnie is a three-voice digital sound chip that lives on the cartridge and mixes
its output with the console's own audio. It is a wavetable synthesiser: each
voice plays a stored 64-sample waveform, or one of three shapes the chip
derives on the fly, at a frequency you set with 16-bit precision.

It has no hardware envelopes, no sequencer and no interrupt. Everything beyond
"play this pitch at this volume with this waveform" is your program's job,
usually driven once per video frame. That sounds limiting, and it is the single
most important thing to understand about writing music for it — but it also
means nothing is hidden from you.

---

## At a glance

| | |
|---|---|
| Voices | 3, fully independent |
| Synthesis | 64-sample wavetable, plus three derived waveforms |
| Waveforms | 2 in RAM (yours to define) + sawtooth, square, triangle |
| Output | 10-bit samples at 27,965 Hz (NTSC) or 27,710 Hz (PAL) |
| Pitch | 16-bit, about 0.43 Hz per step |
| Volume | 64 steps spanning 47.6 dB, per voice |
| Noise | 16 levels of phase modulation, per voice |
| Registers | 32, at `$0460`–`$047F`, write-only in practice |
| Envelopes | none — do them in software |

---

## Quick start

This plays a 440 Hz sine on voice 1 and silences the other two.

```asm
MINNIE = $0460

    ; Silence voices 2 and 3. Waveform 4 is the "off" code.
    LDA #$04
    STA MINNIE+11           ; TIMBRE2
    STA MINNIE+19           ; TIMBRE3

    ; Voice 1: A4, sine, comfortably loud
    LDA #$07
    STA MINNIE+0            ; FREQ1L   \  $0407 = 1031 = 440 Hz
    LDA #$04
    STA MINNIE+1            ; FREQ1H   /
    LDA #$00
    STA MINNIE+3            ; TIMBRE1  = waveform 0, no noise
    LDA #$1E
    STA MINNIE+2            ; VOL1
```

That is the whole interface. Everything below is detail and technique.

---

## Register map

Three identical voice blocks, then a control block. Every address is a byte.

| Offset | Voice 1 | Voice 2 | Voice 3 | Name | What it does |
|---|---|---|---|---|---|
| +0 | `$0460` | `$0468` | `$0470` | `FREQnL` | Frequency, low byte |
| +1 | `$0461` | `$0469` | `$0471` | `FREQnH` | Frequency, high byte |
| +2 | `$0462` | `$046A` | `$0472` | `VOLn` | Volume |
| +3 | `$0463` | `$046B` | `$0473` | `TIMBREn` | Waveform and noise level |
| +4 | `$0464` | `$046C` | `$0474` | `INDEXnL` | Phase, low byte |
| +5 | `$0465` | `$046D` | `$0475` | `INDEXnH` | Phase, high byte |
| +6 | `$0466` | `$046E` | `$0476` | — | Unused |
| +7 | `$0467` | `$046F` | `$0477` | — | Writes here do nothing useful |

| Address | Name | What it does |
|---|---|---|
| `$0478` | `CREG` | Chip control |
| `$0479` | `WAVEADDR` | Waveform upload pointer |
| `$047A` | `WAVEDATA` | Waveform upload data, auto-increments |
| `$047B`–`$047F` | — | Unused |

Because the blocks are eight bytes apart, indexing works nicely:

```asm
    ; X = 0, 8 or 16 selects the voice
    LDA freq_lo,Y
    STA MINNIE+0,X
    LDA freq_hi,Y
    STA MINNIE+1,X
```

### Do not read these registers

They are write-only for all practical purposes. A read returns the value from
the *previous* read, not the register you asked for, so getting a byte out takes
two reads and is only useful for hardware testing.

The consequence that will bite you: **read-modify-write instructions do not
work.** `INC $0462`, `DEC`, `ASL`, `LSR` and friends all read first, and they
will read garbage. Keep a shadow copy of every register in RAM, modify that,
and write the result out.

```asm
    ; Wrong — silently corrupts the register
    INC MINNIE+2

    ; Right
    INC vol1_shadow
    LDA vol1_shadow
    STA MINNIE+2
```

---

## Pitch

Each voice has a 16-bit phase accumulator called the *index*. Every sample tick
the chip adds the frequency constant to it, and the top bits of the result pick
which point of the waveform to output. A bigger constant walks through the
waveform faster, which means a higher pitch.

```
constant = frequency_in_Hz x 2.34349        (NTSC)
constant = frequency_in_Hz x 2.36506        (PAL)
```

One step is about 0.43 Hz, which at the bottom of the range is far finer than
you need and near the top is about a fifth of a cent. Pitch resolution is never
your problem on this chip.

**Constants double per octave.** Store one octave and shift.

| Note | Hz | Constant | | Note | Hz | Constant |
|---|---|---|---|---|---|---|
| C4  | 261.63 | `$0265` | | F#4 | 369.99 | `$0363` |
| C#4 | 277.18 | `$028A` | | G4  | 392.00 | `$0397` |
| D4  | 293.66 | `$02B0` | | G#4 | 415.30 | `$03CD` |
| D#4 | 311.13 | `$02D9` | | A4  | 440.00 | `$0407` |
| E4  | 329.63 | `$0304` | | A#4 | 466.16 | `$0444` |
| F4  | 349.23 | `$0332` | | B4  | 493.88 | `$0485` |

Shifting left goes up an octave and is exact. Shifting right goes down and
loses at most half a step, which is inaudible. If you want a full table without
shifting at run time, generate 8 octaves at build time — it is only 192 bytes.

### Range

| | |
|---|---|
| Lowest useful | about 20 Hz, constant `$002F` |
| A0 | 27.5 Hz, `$0040` |
| A4 | 440 Hz, `$0407` |
| Highest useful | about 10–12 kHz |

The chip will happily accept constants above that, but there is nothing sensible
up there — see *Aliasing* below.

---

## Waveforms

`TIMBREn` holds the waveform in its low three bits.

| Code | Waveform | Resolution |
|---|---|---|
| 0 | RAM waveform 0 — a sine unless you replace it | 64 steps per cycle |
| 1 | RAM waveform 1 — a hollow reed tone unless you replace it | 64 steps per cycle |
| 2 | Silent (behaves as off) | — |
| 3 | Silent (behaves as off) | — |
| 4 | **Off** | — |
| 5 | Sawtooth | 256 steps per cycle |
| 6 | Square | 2 steps per cycle |
| 7 | Triangle | 256 steps per cycle |

The two RAM waveforms are the interesting ones: they are 64 arbitrary samples
each, and you can overwrite them. The other three cost the chip nothing and are
always available.

**Turning a voice off means waveform 4.** There is no volume setting that
reaches true silence — the quietest volume still passes 1/128 of the sample.
Code 4 is how you actually stop a voice.

### Choosing a waveform

- **Sine** is the only shape you cannot derive from the index arithmetic, which
  is why it is worth one of the two RAM slots. It is also the only waveform that
  stays clean when you turn the noise up, and it never aliases.
- **Sawtooth** is bright and buzzy, the classic chiptune lead and bass. It has
  every harmonic, so it aliases badly on high notes.
- **Square** is hollow and reedy, and cuts through a mix. Same aliasing caveat.
- **Triangle** is soft and flute-like, close to a sine but with a little edge.
  Good for bass lines, where its low harmonic content keeps things clean.
- **The reed tone** in RAM waveform 1 sits between a square and a sine, with a
  bump around the fifth harmonic. Useful when you want character without the
  harshness of a raw square.

### Aliasing, and what to do about it

The chip produces 27,965 samples a second, so nothing above about 14 kHz can
exist. Harmonics that would land above it fold back down as inharmonic
whistling, which sounds like a cheap ring modulator.

Rules of thumb:

- The sine never aliases. Use it for high melody lines.
- The RAM waveforms hold at most 32 harmonics, so they behave up to about
  400 Hz before their top harmonics start folding.
- Sawtooth, square and triangle are generated at full index resolution and have
  far more harmonic content. They are excellent for bass and mid-range and get
  progressively grittier above about 1 kHz.
- That grit is not always wrong. Aliasing on a high square is a recognised
  chiptune sound. Just make it a decision rather than an accident.

---

## Volume

`VOLn` is the fiddliest register on the chip, so read this part twice.

```
 bit  7    6     5     4     3     2     1     0
      0   VOL5  VOL4  VOL3  VOL2  VOL1  VOL0   -
```

Bit 7 must be written as zero. Bit 0 is ignored. The six bits in between split
into a 3-bit exponent (`VOL5`–`VOL3`) and a 3-bit mantissa (`VOL2`–`VOL0`):

```
gain = (1 + mantissa/8) x 2^-exponent
```

That covers 1.875 down to 1/128 — 47.6 dB in 64 steps.

### The trap

The exponent is above the mantissa, so simply adding to or subtracting from the
register **does not** move the volume smoothly. Increasing the byte raises the
mantissa (louder) until it wraps and bumps the exponent (suddenly much quieter).
A naive fade is a sawtooth, not a fade.

### The fix

Think in *levels* instead: 0 is loudest, 63 is quietest, and each step is about
0.75 dB. Exactly 8 levels is 6 dB, which makes octave-style volume maths easy.

Converting a level to a register value is two instructions:

```asm
    ; A = level (0 = loudest, 63 = quietest)  ->  A = VOLn value
    EOR #$07
    ASL A
```

That is the whole conversion. Levels are monotonic, evenly spaced in decibels,
and easy to ramp, which makes envelopes trivial (see below).

| Level | Register | Gain | Relative |
|---|---|---|---|
| 0  | `$0E` | 1.875 | 0 dB (loudest) |
| 8  | `$1E` | 0.938 | −6 dB |
| 16 | `$2E` | 0.469 | −12 dB |
| 24 | `$3E` | 0.234 | −18 dB |
| 32 | `$4E` | 0.117 | −24 dB |
| 48 | `$6E` | 0.029 | −36 dB |
| 63 | `$70` | 0.0078 | −47.6 dB (quietest) |

---

## Headroom — read this before you make everything loud

The three voices are summed into a 10-bit accumulator that **wraps rather than
clips**. Overflow does not sound like gentle distortion; it sounds like the
waveform tearing in half, because a loud positive peak comes out as a loud
negative one.

A full-scale waveform peaks at 127, so the rule is:

```
sum over the three voices of (127 x gain) must stay below 512
```

| All three voices at | Peak sum | |
|---|---|---|
| Level 0 | 714 | **overflows** |
| Level 4 | 524 | **overflows** |
| Level 5 | 476 | safe |
| Level 8 | 357 | comfortable |

**Treat level 8 as your nominal "full volume".** It leaves 3 dB of headroom
with all three voices sounding at once, and you still have 40 dB of range below
it. Levels 0–7 exist for the times when only one or two voices are playing, or
when you want a single lead to punch above the rest.

If you must use the top levels, make sure the parts are not all peaking
together. Voices in the same register, moving in parallel, are the dangerous
case; contrapuntal parts rarely peak simultaneously.

---

## Phase

`INDEXnL` and `INDEXnH` are the raw phase accumulator, and they are writable.
This is more useful than it sounds.

- **Start every note from the same place.** Writing zero to both index bytes
  before a note makes the attack identical every time, which matters for short
  percussive sounds where a random start phase changes the timbre audibly.
- **Sync voices.** Two voices with the same frequency and index track exactly.
  Detune one slightly and you get a chorus; set indexes half a cycle apart
  (`$8000`) and you get a hollow, phasey sound.
- **Cancel a voice.** Two identical voices exactly half a cycle apart cancel to
  near silence. Occasionally useful as an effect, mostly useful to know about
  so it does not surprise you.

```asm
    ; Restart voice 1's waveform from the beginning
    LDA #$00
    STA MINNIE+4            ; INDEX1L
    STA MINNIE+5            ; INDEX1H
```

Reading the index back is not possible, so you cannot use it as a position
counter.

---

## Noise

The top four bits of `TIMBREn` set a noise level from 0 to 15. This is *phase*
noise: each tick the chip adds a small random amount to that voice's index, so
the waveform jitters back and forth in time instead of running at a steady rate.

Level 0 is off. Each level doubles the amount of jitter. The randomness is
centred on zero, so the pitch does not drift — it wobbles.

| Level | Character |
|---|---|
| 1–4 | A slight instability. Warms up a sustained note, like a very subtle chorus |
| 5–9 | Obvious wobble, then a rough, buzzing edge |
| 10–13 | Pitched noise. The note is still there but heavily gravelled |
| 14–15 | The pitch is swamped. Broadband noise, still coloured by the waveform |

Because the noise moves the *phase* rather than adding a separate noise
generator, the waveform matters a lot:

- **Sine plus noise** stays smooth and band-limited at every level. This is the
  combination that gives you a usable sweep from shimmer to filtered noise, and
  it is the right choice for wind, surf, engine rumble and breathy pads.
- **Sawtooth or square plus noise** gets harsh fast, because jittering an edge
  throws energy everywhere. Great for explosions and percussion, less good for
  anything you want to sustain.

### Percussion

There is no dedicated noise channel, so percussion costs you a voice. The
standard approach is a high noise level plus a fast volume decay:

```asm
    ; A snare-ish hit on voice 3
    LDA #$F7                ; noise level 15, triangle
    STA MINNIE+19           ; TIMBRE3
    LDA #$28                ; a low pitch; the noise dominates anyway
    STA MINNIE+16
    LDA #$01
    STA MINNIE+17
    LDA #$00                ; level 0 -> loudest
    STA drum_level
```

Then each frame, ramp `drum_level` up quickly (say 6 levels a frame for a snare,
3 for a longer crash) and drop the noise level with it. When the level passes
63, write waveform 4 and free the voice.

Dropping the noise level as the hit decays is what makes it sound like a real
drum rather than a burst of static — real percussion loses its high-frequency
content as it dies away.

---

## Custom waveforms

The two RAM waveforms are 64 samples each, 8-bit signed (−128 to +127). Upload
them through a pointer register and an auto-incrementing data register.

`WAVEADDR` holds a 7-bit pointer: bit 6 chooses the waveform (0 or 1), bits 5–0
choose the sample. `WAVEDATA` writes one byte at the pointer and advances it.

```asm
    ; Load 64 bytes into waveform 1
    LDA #$40                ; bit 6 set = waveform 1, sample 0
    STA MINNIE+25           ; WAVEADDR
    LDX #$00
copy_wave:
    LDA my_waveform,X
    STA MINNIE+26           ; WAVEDATA, pointer advances itself
    INX
    CPX #64
    BNE copy_wave
```

### Making good waveforms

- **Keep the average near zero.** A waveform with a DC offset wastes headroom
  and makes volume changes click. Sum your 64 samples; the total should be close
  to zero.
- **Match the endpoints.** Sample 63 runs straight into sample 0, so if they are
  far apart you get a step, and a step is a click sixty times a second at low
  pitches. Design the waveform as one seamless cycle.
- **Use the full range.** Scale so the peak sits near ±127. Anything quieter is
  throwing away resolution you cannot get back with the volume register.
- **Mind the harmonics.** 64 samples can only carry 32 harmonics. Anything
  sharper than that gets stair-stepped, and the steps alias. If you build a
  waveform by summing harmonics, stopping at the 16th usually sounds better than
  going to the limit.
- **Two slots is not many.** Both voices playing waveform 0 share the same 64
  bytes, so you cannot give each voice its own timbre unless one of them uses a
  derived waveform. Plan your instrument assignments around this.

Changing a waveform while it is sounding is allowed and takes effect
immediately. It also usually clicks, so change waveforms between notes.

---

## Chip control

`CREG` at `$0478`:

| Bit | Name | Effect |
|---|---|---|
| 5 | Reset | While set, the synthesis engine is held stopped and silent |
| 4 | Noise reset | While set, the noise generator is held at its starting value |
| others | — | Accepted and ignored |

The reset bit is genuinely useful at start-up. Set it, write all your registers
into a known state, then clear it — that way the chip does not produce whatever
noise the uninitialised registers happen to describe while you are still setting
up.

```asm
    LDA #$20                ; hold the engine in reset
    STA MINNIE+24           ; CREG
    ; ... set up all three voices, upload waveforms ...
    LDA #$00                ; release
    STA MINNIE+24
```

Setting the noise reset bit makes the noise sequence repeat identically, which
is handy if you want a sound effect to be bit-for-bit reproducible.

---

## Timing, and how many writes you can afford

The chip computes one output sample every 64 CPU cycles, about every 36
microseconds. Building that sample takes it 30 of those 64 cycles, and the other
34 are free for your writes. Each of your writes borrows one.

**You cannot overrun this budget with a 6502.** The fastest store takes four
cycles, so in 64 cycles you can manage at most 16 accesses, well under the 34
available. Write as fast as you like; the chip will keep up.

If it ever were overrun, the result is mild: the chip repeats one sample and
carries on, which is inaudible.

There is no interrupt and no status flag to poll. Drive the chip from your
existing vertical blank handler, which gives you roughly 466 sample ticks
between updates — far more resolution than music needs.

**Updating once per frame is the normal design.** Everything below assumes it.

---

## Envelopes in software

This is where most of your effort will go, because the chip does none of it.

An envelope is just a level that changes each frame. Because levels are already
spaced in decibels, a *linear* ramp on the level produces an *exponential* fade
in amplitude, which is exactly the shape a plucked or struck instrument has. You
do not need a curve table.

```asm
    ; Once per frame, per voice
    LDA v1_level
    CMP #64
    BCS v1_finished         ; already ran off the end
    CLC
    ADC v1_decay            ; how fast this instrument fades
    STA v1_level
    CMP #64
    BCS v1_silence
    EOR #$07                ; level -> register value
    ASL A
    STA MINNIE+2
    JMP v1_done
v1_silence:
    LDA #$04                ; waveform 4 = off
    STA MINNIE+3
v1_finished:
v1_done:
```

Useful decay rates, assuming 60 frames a second:

| Decay per frame | Time to fade 40 dB | Suits |
|---|---|---|
| 1 | about 0.9 s | Sustained strings, pads |
| 2 | about 0.45 s | Plucked strings, bells |
| 4 | about 0.22 s | Marimba, short blips |
| 8 | about 0.11 s | Percussion |

A full ADSR is the same idea with a small state machine: attack counts the level
*down* from a starting point, decay counts it up to a sustain level, sustain
holds, and release counts up to 64. Two bytes of state per voice covers it.

Do not try to run envelopes faster than the frame rate. There is no timer, and
nothing about this chip rewards sub-frame precision.

---

## Technique

### Arpeggios

With three voices and no chords to spare, the classic trick is to play a chord
one note at a time, changing pitch every frame or two. At 60 Hz this fuses into
something the ear hears as a chord with a distinctive shimmer.

```asm
    ; Cycle a major triad on voice 1: root, third, fifth
    LDX arp_step
    LDA arp_table,X         ; three entries: 0, 4, 7 semitones
    ; ... look up the constant for base_note + offset, write FREQ1L/H ...
    INX
    CPX #3
    BNE arp_ok
    LDX #0
arp_ok:
    STX arp_step
```

One voice arpeggiating plus a bass line plus a melody is a complete
three-voice arrangement, and it is what most music for this kind of chip does.

### Vibrato

Add a small oscillating offset to the frequency constant. A quarter-semitone
either way at about 6 Hz is a natural depth — that is roughly 1.5% of the
constant, so a simple table of small signed offsets scaled to the note works
well. Delay the onset by a few frames after the note starts; vibrato from the
very first frame sounds mechanical.

### Detune and thickness

Two voices on the same note, one with its constant a few steps higher, beat
against each other slowly and sound much fatter than either alone. Three or four
steps of difference is a good starting point at middle pitches. Scale the offset
with pitch to keep the beat rate even.

This costs two voices, so it is a lead-only trick.

### Bass

Triangle or square in the bottom two octaves, at a level around 6–10. Low notes
carry more energy than you expect and will eat your headroom if you let them.
Keep the bass a few decibels below the melody.

### Sound effects that do not wreck the music

Games usually reserve voice 3 for effects and write the music for two voices.
That is far less painful than trying to steal a voice mid-phrase. If you must
steal, take the voice whose note is furthest through its envelope — the ear
notices a truncated attack much more than a truncated tail.

Remember to restore whatever the music expected on that voice when the effect
finishes, including the waveform and the noise level.

---

## Gotchas

- **Read-modify-write does not work.** `INC`, `DEC`, `ASL`, `ROR` and friends on
  a Minnie register read garbage first. Shadow everything in RAM.
- **The volume register is not linear or even monotonic.** Adding to it makes
  the sound louder *then* abruptly quieter. Work in levels and convert with
  `EOR #$07 : ASL A`.
- **There is no silent volume.** Use waveform 4 to stop a voice.
- **The three-voice sum wraps.** Level 5 or quieter on all three voices is safe;
  level 8 is the sensible nominal maximum.
- **Waveform codes 2 and 3 are silent.** They are not spare timbres.
- **Changing pitch mid-note does not reset the phase**, which is what you want
  for slides and vibrato, and not what you want for a retrigger. Write the index
  explicitly when you need a clean restart.
- **Uploading a waveform affects every voice using it**, immediately, mid-note.
- **The chip is deaf to reads**, so you cannot query anything. Your RAM state is
  the only record of what you set.
- **PAL machines run 0.9% slower**, so constants computed for NTSC play slightly
  flat. If you care, keep two note tables; most music will not notice.

---

## A minimal frame-driven player

Everything above, assembled into the smallest thing that counts as a music
driver. It reads a stream of per-frame register writes — the simplest possible
format, and easy to generate from a tracker or a MIDI file offline.

```asm
; Stream format, per frame:
;     count, then <count> pairs of (register offset, value)
; A count of $FF ends the stream.

MINNIE = $0460
PTRL   = $80                ; zero page
PTRH   = $81
COUNT  = $82

play_init:
    LDA #$20                ; hold the chip in reset while we set up
    STA MINNIE+24
    LDA #$04                ; all three voices off
    STA MINNIE+3
    STA MINNIE+11
    STA MINNIE+19
    LDA #<song
    STA PTRL
    LDA #>song
    STA PTRH
    LDA #$00                ; release
    STA MINNIE+24
    RTS

; Call once per frame from your vertical blank handler.
play_frame:
    JSR next_byte
    CMP #$FF
    BEQ play_init           ; end of song, start over
    CMP #$00
    BEQ play_done           ; nothing to do this frame
    STA COUNT
play_loop:
    JSR next_byte
    TAX                     ; register offset, 0..31
    JSR next_byte
    STA MINNIE,X
    DEC COUNT
    BNE play_loop
play_done:
    RTS

next_byte:
    LDY #$00
    LDA (PTRL),Y
    INC PTRL
    BNE next_done
    INC PTRH
next_done:
    RTS
```

Note the `CMP #$00` before the write loop. `STA` does not affect the flags, so
testing the count straight after storing it would read the result of the
previous comparison instead — a bug that only shows up on frames where nothing
changes, which makes it unpleasant to find.

Only writing the registers that actually changed keeps a typical frame down to a
handful of bytes, and a few minutes of music inside a few kilobytes.

---

## Worked example: a simple instrument set

A starting point you can adapt, using two RAM waveforms and the three derived
ones.

| Part | Waveform | Noise | Start level | Decay | Notes |
|---|---|---|---|---|---|
| Lead | Sine (0) | 0 | 8 | 1 | Clean at any pitch, never aliases |
| Counter-melody | Reed (1) | 0 | 10 | 1 | Sits behind the lead without fighting it |
| Bass | Triangle (7) | 0 | 8 | 0 | Sustained; low harmonic content stays clean |
| Pluck | Sawtooth (5) | 0 | 6 | 4 | Bright attack, fast fade |
| Bell | Sine (0) | 2 | 4 | 2 | The touch of noise makes it shimmer |
| Snare | Triangle (7) | 15→0 | 0 | 6 | Ramp the noise down with the level |
| Kick | Sine (0) | 4→0 | 2 | 8 | Sweep the pitch down over three frames |
| Wind | Sine (0) | 13 | 24 | 0 | Sustained, slow level drift for movement |

The kick is worth a comment: sweeping the frequency constant downwards over the
first two or three frames, from around 150 Hz to around 50 Hz, is what turns a
click into a thump. The noise level dropping to zero over the same span adds the
beater sound at the front.
