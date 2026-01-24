# Industrial Pipe Bending Simulator
<p align="center">
  <img src="assets/output.gif" width="100%" alt="Симуляция работы станка">
</p>
A high-performance, real-time 3D simulation software for industrial CNC pipe bending machines. Built with **Rust** and the **Bevy** game engine, this project bridges the gap between low-level CAD topology analysis and real-time kinematic simulation.

## 🚀 Key Features

### 🏗️ CAD & Computational Geometry
* **STEP File Import (ISO 10303):** Custom implementation of a STEP parser using `ruststep`. Extracts complex B-Rep topology (Shells, Faces, Oriented Edges) directly from raw CAD data.
* **Topology Analysis:** Algorithms to recognize geometric features such as cylindrical segments and toroidal bends from NURBS/B-Spline curves.
* **Automatic "Unbending" Algorithm:** Mathematically analyzes bent pipe models to calculate the raw straight pipe length and generates the precise **LRA** (Length, Rotation, Angle) bending schedule for CNC machines.
* **Adaptive Triangulation:** Dynamic tessellation of curved surfaces (tori/cylinders) with adjustable segment count for optimal balance between accuracy and performance.

### 🏭 Physics & Kinematics Simulation
* **Finite State Machine (FSM):** Manages the complex lifecycle of the CNC robot, handling states like `Feeding`, `Rotating`, `Bending`, `Clamping`, and `Unclamping`.
* **Virtual Machine Registers:** A digital twin architecture that maps physical motor parameters (positions, angles, velocity) to the 3D visual representation in real-time.
* **Collision-Aware Movement:** Simulates the physical constraints and mechanical interactions of machine parts (mandrels, wipers, pressure dies).

### 🎨 Rendering & Architecture
* **Bevy ECS Architecture:** Highly modular design separating logic (`MachineControl`), rendering (`PipeView`), and data processing.
* **Procedural Mesh Generation:** Generates machine parts (rollers, dies) and deformable pipe meshes on the fly using `truck_meshalgo` and `truck_modeling`.
* **Custom Shaders:** Specialized WGSL shaders for technical drawing lines (`LineMaterial`) and auxiliary geometry.

## 🛠️ Tech Stack

* **Language:** Rust
* **Engine:** [Bevy](https://bevyengine.org/) (ECS, Rendering)
* **CAD Kernel:** `truck` ecosystem (Topology, Meshing, Boolean operations)
* **Math:** `cgmath`, `glam`
* **UI:** `bevy_egui`
* **Data Parsing:** `ruststep`

## 🎮 Controls

The simulation features a CAD-style camera controller:

| Input | Action |
| :--- | :--- |
| **Right Mouse Button + Drag** | Orbit / Rotate Camera |
| **Middle Mouse Button + Drag** | Pan Camera |
| **Mouse Wheel** | Zoom In / Out |

## 🧩 Architecture Overview

The project is structured around the Entity Component System (ECS) pattern:

1.  **`MachineRegisters` Resource:** Acts as the "brain" of the CNC machine, holding the current state of all axes (Y-rotation, Z-feeding, Bend Angle).
2.  **`LRACLR