# composite_decoder

A portable NTSC composite decoder. Give it a baseband composite signal and it
gives you RGB, decoded the way a cheap receiver does it: a half-subcarrier notch
splits luma from chroma, the chroma is demodulated against a locally generated
carrier, and the result is corrected against the colourburst measured in the
signal itself.

Nothing in it is specific to any console. MIT licensed, one file, no
dependencies.

## What you get

Composite decoding is lossy in specific, characteristic ways, and those losses
are the reason to use this rather than route RGB straight out:

- **Artifact colour.** Luma detail that lands on the subcarrier comes out as
  colour, because the notch cannot tell the two apart. Software that draws
  alternating pixels to get colours its video mode does not have works.
- **Chroma resolution around one subcarrier cycle**, against full-bandwidth
  luma. A chroma dither one cycle wide averages to the colour in between.
- **Colour that trails to the right** of the edge that caused it.
- **Cross-luminance.** Saturated colour edges leave dots in the luma.

## Signal format

`comp` is **signed 24-bit fixed point, Q2.21, where 1.0 = 1.0 V** at the
composite output. Range -4.000 V to +3.999999 V, resolution about 0.48 uV.

```
 -0.286 V   sync tip          -40 IRE
  0.000 V   blanking / black    0 IRE
  0.054 V   NTSC setup        7.5 IRE
  0.714 V   white             100 IRE
```

Nothing is clipped on the way in, and you should not clip on the way in either.
Ordinary saturated chroma already swings past both rails, to about 131 IRE and
-33 IRE. Sources that leave broadcast range entirely - the NES most notably,
whose brightest luma sits above white and whose `$0D` sits below blanking - are
supposed to reach the output stage intact, because reproducing what a set does
with them is the point. Clipping happens once, at the RGB output.

There is no automatic gain. White is 0.714 V and that is what becomes 255. A
source that runs hot is meant to come out bright and clip, not to be normalised.

## Sync

Sync is **not** separated from the signal. `hs_in / vs_in / hb_in / vb_in` come
in as positive-pulse flags in the same sample domain as `comp`. Every FPGA core
already has them, so recovering them from the signal would add a whole
lock-failure surface for no fidelity. This is the one deliberate departure from
a real receiver.

## Choosing a sample rate

`SPC` is samples per subcarrier cycle and **must be even**, so that the notch
delay `SPC/2` is a whole number of samples and lands exactly half a cycle back.

Pick a sample clock that is a whole multiple of the colour subcarrier. For NTSC
at 3.579545 MHz:

| SPC | sample clock | notes |
|---|---|---|
| 4  | 14.318 MHz | the D2 rate. Works, coarsest chroma filter |
| 8  | 28.636 MHz | recommended. Demod coefficients degenerate to `0, +/-0.707, +/-1` |
| 12 | 42.955 MHz | finer, costs more |

`SPC` also sets the horizontal expansion. Decoding at 8 samples per cycle a
source whose pixel clock is the subcarrier gives 8 output samples per input
pixel; a source at twice the subcarrier gives 4. Size whatever consumes the
output accordingly.

The carrier accumulator free-runs and is never reset at hsync, on purpose.
Sources whose line is a whole number of subcarrier cycles then hold a stable
artifact hue down each column, and sources whose line is a half cycle get their
dot crawl, without either having to be declared.

## Burst window

`burst_start` and `burst_len` are in samples counted from the **trailing edge of
`hs_in`**.

- `burst_start` should point a few samples *into* the burst, so its leading edge
  is not measured. 16 samples are averaged from there.
- `burst_len` must cover the rest of the burst. The black level is measured from
  the back porch starting at `burst_start + burst_len`, and 32 samples are
  averaged, so leave that much back porch before active video begins.

If the burst is too weak to measure, colour is killed and the output is
monochrome, which is what a receiver does with a monochrome signal.

## Config

| port | |
|---|---|
| `sat` | saturation, 128 = unity |
| `hue` | 256 = one full cycle. Applied to the burst reference once per line, not per sample |
| `smear` | chroma trail length, 0 = off. Larger is longer |
| `luma_delay` | luma path delay in samples, 0 to 15 |

**`luma_delay` is meant to be set too low.** A symmetric filter with the luma
delay trimmed to match would bleed colour evenly both ways, which is not what
composite looks like. A real set lowpasses demodulated chroma with a causal
analogue filter, an exponential decay to the right, and runs the chroma path at
a longer group delay than the luma path, which is only notched. Both push colour
rightward. So `smear` supplies the tail and `luma_delay` set below the chroma
path's own delay supplies the lag. Start around `SPC/2` and trim by eye.

## Cost

Roughly 12 multiplies in the datapath: two to demodulate, two for the boxcar
divide, four for the burst correction, six for the output matrix, and at `SPC` a
power of two the demodulation degenerates to a select and a shift-add. One
sequential division per line, which has a whole active line to complete. No line
buffer: there is no comb filter, deliberately, because a comb assumes 227.5
subcarrier cycles per line and cancels the wrong one of the two on sources whose
line is a whole number of cycles.

## Status

The scale constants `LUMA_GAIN` and `BURST_SHIFT` are first estimates. They set
where white lands and how saturated chroma comes out, and they are pinned by a
colour-bar test that has not been written yet. Expect to adjust them.
