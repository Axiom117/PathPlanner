# 📌 Simulink Mechanism Modeling & Inverse Kinematics Simulation Prompt

## 🎯 Objective
Design and simulate a **modular, closed-loop kinematic chain mechanism** in **Simulink using Simscape Multibody**, to perform **RCM (Remote Center of Motion)** motion. The system is driven by two 3-DOF prismatic-joint micro-manipulators, with motion planning based on an inverse kinematics solver. The target output is the required input displacements of both manipulators given a desired end-effector pose.

---

## 🔍 Mechanism Overview

- **Mechanism Type**: Closed-chain, planar, 3-link mechanism (links A–B–C)
- **Topology**: Serial connection of three equal-length links (A–B–C) using two revolute joints
- **RCM Motion**: End-effector constrained to rotate about a fixed point in space (origin)

### 🦾 Actuation
- Two micro-manipulators control the deformation of the passive linkage
- Manipulators connect to:
  - Midpoint of Link A (left)
  - Midpoint of Link C (right)
- End-effector is mounted on the midpoint of Link B

---

## 🧭 Coordinate System & Geometry

| Feature               | Description                                     |
|-----------------------|-------------------------------------------------|
| Link Length           | `l` (equal for A, B, C)                         |
| End-effector Height   | `h`, Radius: `r`                                |
| Needle Tip Location   | `[0, 0, 0]` (origin)                            |
| Mount Point Location  | `[0, 0, h]` (top of cylinder)                   |

### ⛳ End-Effector Frame Definition

| Axis | Direction             |
|------|------------------------|
| x+   | Needle direction       |
| y+   | Normal to paper plane  |
| z+   | Upward                 |

---

## 🔁 Input / Output Specification

### ✅ Inputs
- **Desired end-effector pose**:  
  `(x_E, y_E, z_E, φ_E, θ_E, ψ_E)`  
  - For RCM simulation, simplify as: `(x, y, z, φ, 0, 0)`

### 🧮 Outputs
- Micro-manipulator 1 pose: `(x₁, y₁, z₁, 0, 0, 0)`
- Micro-manipulator 2 pose: `(x₂, y₂, z₂, 0, 0, 0)`

---

## 🛠️ Simulink Model Structure

### 🔧 Core Subsystems and Components

| Component                     | Description                                      |
|-------------------------------|--------------------------------------------------|
| **Prismatic Joint ×6**        | 3-DOF × 2 micro-manipulators                    |
| **Revolute Joint ×2**         | Between links A–B and B–C                       |
| **Rigid Transform ×2**        | Offset links A and C from manipulators          |
| **Cartesian Joint ×1**        | End-effector translational DOF (XYZ)           |
| **Revolute Joint ×1**         | End-effector rotational DOF (around x-axis)    |

### 📦 Subsystem Encapsulation

- Subsystem encapsulates the full mechanism.
- Input:  
  `pose` from workspace (`[x, y, z, φ, θ, ψ]`)
- Output:  
  `out.q` to workspace (6×1 vector for prismatic joint positions)

---

## 🧱 System Hierarchy & Connection Diagram

World Origin
├── RigidTransform1: [-l, 0, h]
│ └── Micro-manipulator 1 (Prismatic ×3)
│ └── Link A
├── RigidTransform2: [+l, 0, h]
│ └── Micro-manipulator 2 (Prismatic ×3)
│ └── Link C
├── Link A ── Revolute Joint ── Link B ── Revolute Joint ── Link C
│ │
│ └── End-effector (at midpoint of Link B)
│ └── Revolute Joint ── Cartesian Joint
│ │
│ └── World Origin


---

## 💡 Modeling Notes

- **Micro-manipulator initial transforms**:
    ```matlab
    % Micro-manipulator 1
    rigidTransform1.Translation = [-l, 0, h];

    % Micro-manipulator 2
    rigidTransform2.Translation = [l, 0, h];
    ```

- **End-effector motion input**:
  - Provided via `pose` signal (6×1 vector from workspace)
  - Only `x`, `y`, `z`, and `φ` are varied (`θ = ψ = 0`)

- **Joint actuation**:
  - Enable actuation on **Cartesian Joint** and **End-effector Revolute Joint**
  - Use external signals to drive to target pose

- **Inverse kinematics logic**:
  - By constraining the end-effector trajectory
  - Simscape calculates reaction at micro-manipulator prismatic joints
  - Extract displacements of 6 prismatic joints via `out.q`

---

## 📤 Simulation I/O

- `from workspace`:  
  Input pose as timeseries or array with format `[time, x, y, z, φ, θ, ψ]`

- `to workspace`:  
  Captures joint displacement as `[time, x₁, y₁, z₁, x₂, y₂, z₂]`

---

## 📌 Remarks
- This simulation validates feasibility of **RCM generation** using a **purely passive mechanical chain** driven at both ends.
- System supports inverse design: Given a surgical task trajectory, compute manipulator inputs.
- Future extension: Add closed-loop control for manipulator actuation based on real-time feedback.

---