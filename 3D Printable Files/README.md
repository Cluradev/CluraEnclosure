# 3D Printable Files

> [!WARNING]
> **Printing is not recommended yet.** These files are ready to review, but small
> improvements may still land before delivery. Please hold off on printing until this note
> is removed, to avoid reprinting a part that changes.

The parts you print yourself, as `.3mf`.

> **Not sure which files you need?**
> [CluraDocs has a picker](https://docs.clura.dev/the-build/preparations/printed-parts):
> choose your size and options and it lists exactly which files to print, how many of each,
> with a preview and a download link per part.

## Print settings

These work for every part except one:

| Setting | Value |
| --- | --- |
| Material | PETG |
| Layer height | 0.2 mm |
| Perimeters | 4 |
| Infill | 20% |

`1x_ServoHead_0.1mm` is the exception and needs a **0.1 mm layer height**. It is a small
splined part that grips the servo shaft, and at 0.2 mm the splines come out too coarse to
engage properly.

Supports are only needed on a few parts.

PLA is acceptable for most parts if that is what you have, with one exception: do not use it
for the lighting fixtures, which sit close to the LEDs and run warmer than the rest of the
build.

## Folder layout

```
Printer_Dependent/     electronics cooling, Bambu A1 and A1 mini only
Size Agnostic/         the same on every enclosure size
  AMS Holder             optional
  Electronics            mainboard mount, fan holder, cable organisers
  Filtration Assembly    one assembly per set of files, see below
  Handles                side, front and back, top
  Hinges                 front and top
  Magnet Holders         back panel and side panels
  Misc Parts
  Screen Assembly
  Sensors Assembly
  Spool Holders          weight sensing (Pro) or plain (Base)
Size Specific/
  PE1/  BE1/  ME1/      acrylic guides, bed cable passage, cable management,
                        jigs, lighting
```

## Quantities

The number at the start of a filename is how many to print for **one enclosure**, so
`2x_Cable_Organizer` means two.

A few depend on your build:

**Filtration assembly.** The files make one assembly. ME1 needs one, PE1 and BE1 need two, so
every quantity in that folder doubles on those sizes: a `2x_` becomes 4.

**Spool holders.** Pro uses the weight-sensing set, which carries the load cells. Base uses
the plain holders. PE1 and BE1 print both a left and a right, ME1 only has room for the
right.

**Front handles.** Without the lock addon print 4 of `3or4X_Handle_NoLock`. With the lock
print 3, plus `1x_Handle_Lock` and `1x_LockStopper`.

**Side handles.** Six files, three left and three right. Print one of each: plain, with a
hole, or with a PTFE passthrough for feeding filament in from outside.

**Magnet mounts.** Two mutually exclusive options. Either the reinforced pair, one left and
one right, or two of `2x_Magnet_Mount_Side_panels`. Do not print both sets.

**Top lighting.** The `Lighting/OPTIONAL` folder is an extra light bar on PE1 and BE1. ME1
does not have the variant.

**Printer dependent.** The electronics cooling ducts only apply to the Bambu Lab A1 and A1
mini. Skip the folder on any other printer.
