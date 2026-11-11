# Robotic Car Painting Cell Simulation

- **Your name:** Pavan C S
- **Software used:** RoboDK (Python OLP API)
- **Robot selected:** ABB IRB 5500-27 (with Generic Paint Sprayer)

## Project Overview
This repository contains the RoboDK station and Python offline programming (OLP) scripts for a 4-stage 360-degree automotive paint booth cell (`4800 mm x 1900 mm x 1600 mm` reference sedan inside a `12 m x 8 m` paint booth).

## Files Included
- `Robotic_Paint_Station.rdk`: Complete RoboDK simulation station file.
- `build_cell.py`: Procedural 3D booth/sedan geometry generator and 4-stage `MoveJ`/`MoveL` trajectory builder.
- `Run_4Stage_Full_Car.py`: Interactive turntable indexing script (`0°`, `90°`, `180°`, `270°`) with target visibility filtering.
- `Technical_Report.pdf`: Technical report covering cell layout, custom Jacobian solver, trajectory planning, and collision checking.
