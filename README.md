# CluraEnclosure

**The Clura Enclosure** is the 100% open-source, modular, and smart enclosure designed for desktop 3D printers. Developed with a focus on safety and air quality, it incorporates active filtration and smart sensors to enhance your 3D printing experience.

![Clura Enclosure](IntroPicture.avif)

## Why We Built It

The project started with me (Fabrizio), an aerospace engineering student who wanted to print in his room without compromising his safety. In fact, it is widely documented that most 3D printer filaments emit VOCs and particulate matter, which can be harmful to health. I searched for a smart, affordable, open-source enclosure but couldn't find one, so I decided to build my own. 18 months of hard work later, here we are!

## Kits

Kickstarter orders are being fulfilled. Future kit batches are being planned, but nothing is finalized yet, so there is no ordering page and no dates to give.

You can still build the enclosure today without waiting on us. Everything here is open source and the [Bill of Materials](Documentation/README.md) lists every part with supplier links, so nothing is gated behind a kit.

The original campaign page is still up for reference: [Kickstarter](https://www.kickstarter.com/projects/clura/enclosure)

## Project Status

> [!NOTE]
> **Some Files are currently being organized and will be uploaded soon.** So if you don't find something you're looking for just email us to check if we made a mistake or if it's something that we are still preparing.


## Documentation

All project documentation, including assembly manuals and Bills of Materials (BOM), will also be published on **CluraDocs**. If you're looking to assemble your own Clura Enclosure, that is the place to go. Note: CluraDocs is still a work in progress, we are always working on improving it. If you have any suggestions or feedback, please let us know!

Check it out here: [CluraDocs](https://docs.clura.dev)

## Key Features

- **Safety & Air Quality:** Dual-layer HEPA and carbon filtration system designed to capture VOCs and UFPs.
- **Smart Controls:** Integrated 4.3" touchscreen for real-time monitoring of temperature, humidity, and air quality.
- **Modular Design:** Fully customizable hardware and software modules.
- **Open Source:** 100% open source CAD files, firmware, and guides.

![Clura Enclosure Features](FeaturesPictures.avif)

## Repository Structure

- `Documentation/`: Assembly manual and BOM.
- `Firmware/`: Firmware for the screen and the mainboard, plus the SD card files.
- `CAD/`: STEP models of the full enclosure, one per size (PE1, BE1, ME1).
- `PCBs/`: KiCad 9 sources for the mainboard and the sensorboard.
- `Production/`: Ready-to-use production files (laser cutting, extrusions, metal plates, PCB gerbers).
- `3D Printable Files/`: The `.3mf` files you print yourself.

In each folder there is a readme file with more info.

> For the printable parts, [CluraDocs](https://docs.clura.dev/the-build/preparations/printed-parts) has a picker that works out exactly which files and quantities your build needs.

## License

Every file in this repository is released under the **CC BY-NC (Creative Commons Attribution-NonCommercial)** license. 

For more details on the reasoning behind this choice, you can read our blog post: [The License - Clura Blog](https://www.clura.dev/blog/the-license)

---
*For more information, visit [clura.dev](https://www.clura.dev).*
