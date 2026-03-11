# Hardware

| Folder | Contents |
|--------|----------|
| `pcb/` | Gerber file, BOM, CPL, and equipment list for ordering and assembling the PCB |
| `3d_prints/` | STL file for the 3D-printed pump-to-tubing nozzle connector |

## PCB ordering (JLCPCB)

1. Upload `pcb/gerber.zip` to [JLCPCB](https://jlcpcb.com/).
2. Enable **PCB Assembly** (both sides) and upload `pcb/bom.csv` and `pcb/cpl.csv`.
3. Tick **"Confirm Parts Placement"** before ordering.
4. Parts not available through JLCPCB (air pump, Figaro CH₄ sensor/connector, K33 CO₂ sensor) must be sourced separately — see [`pcb/equipment_list.md`](pcb/equipment_list.md).

## 3D prints

`3d_prints/pump_nozzle.stl` — connector between the air pump outlet and the flexible tubing of the floating chamber.
