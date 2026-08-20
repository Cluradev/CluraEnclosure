# Production Files

Everything needed to have the enclosure parts made. Files are split in two groups:

* **Enclosure Size Specific Parts** holds the parts that change with the enclosure size (PE1, BE1, ME1)
* **General Parts** holds the parts that are the same on every size

Each file name starts with the quantity needed, for example `2x_` means two pieces. Acrylic and sheet metal file names also carry the thickness and the material colour.

---

## Folder tree

```
Production
│
├── Enclosure Size Specific Parts
│   │
│   ├── PE1                          Prusa size enclosure
│   │   ├── AcrylicPE1
│   │   │   ├── DXF                  cutting files for the laser shop
│   │   │   └── Drawings             PDF drawings of the same panels
│   │   ├── BottomMetalPlatePE1      DXF, STEP and PDF for the steel bottom plate
│   │   └── ExtrusionsPE1
│   │       ├── Step                 3D models of every cut extrusion
│   │       └── Drawings             PDF drawings with lengths, taps and holes
│   │
│   ├── BE1                          Bambu size enclosure
│   │   ├── AcrylicBE1
│   │   │   ├── DXF
│   │   │   └── Drawings
│   │   ├── BottomMetalPlateBE1
│   │   └── ExtrusionsBE1
│   │       ├── Step
│   │       └── Drawings
│   │
│   └── ME1                          Mini size enclosure
│       ├── AcrylicME1
│       │   ├── DXF
│       │   └── Drawings
│       └── ExtrusionsME1
│           ├── Step
│           └── Drawings
│
└── General Parts
    │
    ├── Electronics
    │   ├── CircuitBoards
    │   │   ├── Mainboard
    │   │   │   ├── GBR              gerbers and drill files
    │   │   │   ├── CPL              pick and place position files
    │   │   │   └── BOM_FINAL_MAINBOARD.xlsx
    │   │   └── Sensorboard
    │   │       ├── GBR
    │   │       ├── CPL
    │   │       └── BOM_FINAL_SENSORBOARD.xlsx
    │   └── CablesDrawings           drawings for the four custom cables
    │
    └── MagneticPlates               0.5 mm steel plates for the screen and the filters
```

> The `.3mf` files you print yourself are **not** in this folder. They live in
> **`3D Printable Files/`** in the repository root.

---

## What is inside

### Enclosure Size Specific Parts

Three sizes are available. Pick the folder that matches your enclosure and use only the files inside it.

| Folder | Enclosure |
| --- | --- |
| PE1 | Prusa size |
| BE1 | Bambu size |
| ME1 | Mini size |

**Acrylic**
`DXF` holds the flat cutting profiles, one file per panel type. Send these to the laser cutting shop. `Drawings` holds the matching PDF drawings so you can check dimensions and part count before ordering. Clear panels are 4 mm, black electronics panels are 3 mm.

**Extrusions**
All parts use 2020 aluminium profile. `Step` holds the 3D model of each cut length, `Drawings` holds the PDF with the length, the tapped ends and the drilled holes. The `2020R` files are the rounded profile version. (they are not essential)

**Bottom metal plate**
1.5 mm steel plate, supplied as DXF for cutting, STEP for reference and PDF for checking. Available for PE1 and BE1. 

### General Parts

**Electronics**
Two boards, the Mainboard and the Sensorboard. Each one has a `GBR` folder with the gerbers and the drill files, a `CPL` folder with the component positions for the assembly house, and a BOM spreadsheet. This is the full package most PCB manufacturers ask for.

**Cables Drawings**
Inside `Electronics/CablesDrawings` there are PDF drawings for the four cables that are not off the shelf: the PMS5003 particle sensor cable, the screen cable, the sensorboard cable and the smoke sensor cable. Use these when ordering from a cable assembly shop.

**Magnetic Plates**
Thin 0.5 mm steel plates that the magnets hold on to. Three screen plates (left, right, top) and two filter plates. Supplied as DXF.

---

## Elsewhere in the repository

* **`3D Printable Files/`** - the `.3mf` files for the parts you print yourself.
  [CluraDocs](https://docs.clura.dev/the-build/preparations/printed-parts) works out which
  ones your build needs.
* **`PCBs/`** - KiCad 9 sources for the mainboard and the sensorboard, if you want to modify
  them rather than just have them made.

