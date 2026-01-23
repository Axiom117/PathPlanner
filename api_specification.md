# Microsupport Quick Pro Specification

**Resolutions:** movement resolution for each axis (μm/pulse)

* `axisX`: X-axis resolution
* `axisY`: Y-axis resolution
* `axisZ`: Z-axis resolution

# API Specification (v2.1)

## 1. Controller Server (WinForms C# APP)

### 1.1 Overview

The Controller Server listens for TCP requests from the PathPlanner Server and controls two manipulators. All messages are plain-text, comma-separated. On error, the server responds with:

```text
ERROR, <code>, <message>
```

### 1.2 Methods

#### GetStatus

* **Trigger:** `GET_STATUS,<id1>,<id2>`
* **Action:**

  1. Validate manipulator IDs.
  2. Read current displacement `(X, Y, Z)` from center for each manipulator.
* **Response:**

  ```text
  STATUS, <id1>, <X1>, <Y1>, <Z1>, <id2>, <X2>, <Y2>, <Z2>
  ```

#### StepAbsFromCenter(id1, X, Y, Z, speed)

* **Trigger:** `START_STEP,<id1>,<X1>,<Y1>,<Z1>,<speed1>,<id2>,<X2>,<Y2>,<Z2>,<speed2>`
* **Action:**

  1. Validate manipulator IDs and bounds of coordinates for each manipulator.
  2. Perform an absolute move for each manipulator to the specified `(X, Y, Z)` coordinates from the center with the specified speeds `<speed>`.
  3. On success, respond with a completion message.
* **Response:**

  ```text
  STEP_COMPLETED, <id1>, <id2>, ...
  ```

#### ProcessPathData(rawData)

* **Trigger:** `PATH_DATA, <controllerId>, <payload>`
* **Action:**

  1. Parse the comma-separated `payload` into a sequence of 3-tuples `(X, Y, Z)`.
  2. Validate the format and store the trajectory for the specified `controllerId`.
* **Response:**

  ```text
  PATH_DATA_RECEIVED, <controllerId>
  ```

#### StartPathTrackingPTP (Point-to-Point)

* **Trigger:** `START_PATH_PTP,<duration>,<id1>,<id2>,...`
* **Action:**

  1. Validate parameters and manipulator IDs.
  2. Immediately send an acknowledgment that path tracking has started.
  3. Asynchronously execute the stored trajectory for each specified manipulator using Point-to-Point (PTP) motion. Each segment's duration is controlled by the `<duration>` parameter.
  4. After each manipulator completes its path, send a `PATH_COMPLETED` or `ERROR` message for that specific manipulator.
* **Initial Response (Acknowledgment):**

  ```text
	PATH_TRACKING_PTP_STARTED,<id1>,<id2>,...
  ```

* **Final Response (per manipulator):**

  ```text
  PATH_COMPLETED, <id>
  ```


#### StartPathTrackingCP (Continuous Path)

* **Trigger:** `START_PATH_CP,<duration>,<id1>,<id2>,...`
* **Action:**

  1. Validate parameters and manipulator IDs.
  2. Immediately send an acknowledgment that path tracking has started.
  3. Asynchronously execute the stored trajectory for each specified manipulator using high-precision Continuous Path (CP) motion. This involves overriding moves to create a smooth path without stopping at each point.
  4. After each manipulator completes its path, send a `PATH_COMPLETED` or `ERROR` message for that specific manipulator.
* **Initial Response (Acknowledgment):**

  ```text
	PATH_TRACKING_CP_STARTED,<id1>,<id2>,...
  ```
* 
* **Final Response (per manipulator):**

  ```text
  PATH_COMPLETED, <id>
  ```


### 1.3 Request → Response Map

| Request                                                     | Response                                                                                             |
| ----------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `HEARTBEAT`                                                 | `HEARTBEAT_OK`                                                                                       |
| `GET_STATUS,<id1>,<id2>,...`                                | `STATUS,<id1>,<X1>,<Y1>,<Z1>,<id2>,<X2>,<Y2>,<Z2>,...`                                                 |
| `START_STEP,<id1>,<x>,<y>,<z>,<speed>,...`                   | `STEP_COMPLETED,<id1>,...`                                                                            |
| `PATH_DATA,<id>,<payload>`                                  | `PATH_DATA_RECEIVED,<id>`                                                                            |
| `START_PATH_PTP,<duration>,<id1>,...`                       | `PATH_TRACKING_PTP_STARTED,<id1>,...` (followed by `PATH_COMPLETED,<id>` for each manipulator)        |
| `START_PATH_CP,<duration>,<id1>,...`                        | `PATH_TRACKING_CP_STARTED,<id1>,...` (followed by `PATH_COMPLETED,<id>` for each manipulator)         |
| *invalid or unknown request*                                | `ERROR,<code>,<message>`                                                                             |

---

## 2. PathPlanner Server (MATLAB APP)

### 2.1 Overview

The PathPlanner Server computes kinematics and coordinates with Controller. All messages are plain-text, comma-separated. On error, it sends:

```text
ERROR, <code>, <message>
```

### 2.2 Properties

* `X0, Y0, Z0`            — current end-effector position
* `Phi0, Theta0, Psi0`    — current end-effector orientation

### 2.3 Methods

#### GetStatus(id1, id2)

1. Send `GET_STATUS, <id1>, <id2>`.
2. Parse response `STATUS, <id1>,<X1>,<Y1>,<Z1>,<id2>,<X2>,<Y2>,<Z2>`.
3. On parse error or timeout, handle error.
4. Return `[(X1,Y1,Z1),(X2,Y2,Z2)]`.

#### StepMove(id1, id2, X, Y, Z)

* Send `START_STEP, <id1>,<id2>,<X>,<Y>,<Z>`.
* Await `STEP_COMPLETED, <id1>,<id2>` or error.

#### onCalcFK()

1. Call `GetStatus(id1,id2)`.
2. Call `SolveFK(X1, Y1, Z1, X2, Y2, Z2, model, params)`.
3. Update `X0, Y0, Z0, Phi0, Theta0, Psi0`.

#### onCalcIK(Xt, Yt, Zt, Phit, Thetat, Psit)

1. Use current pose `(X0...Psi0)`.
2. Call `SolveIK(X0, Y0, Z0, Phi0,...,Psit)`.
3. Return trajectory array `N×6`.

#### PlanPath(id1, id2)

1. Invoke `onCalcFK()`.
2. Invoke `onCalcIK(...)` to get trajectory.
3. Pack into payload:


   ```text
   PATH_DATA,
   X11,Y11,Z11,X12,Y12,Z12, ...
   ```
4. Send `PATH_DATA, <payload>` and await `PATH_DATA_RECEIVED`.

#### ExecutePath(id1, id2)

* Triggered when `PATH_DATA_RECEIVED` arrives.
* Send `START_PATH, <id1>, <id2>`.
* Await `PATH_COMPLETED`.

### 2.4 Controller → MATLAB Map

| From Controller      | MATLAB Action / Response                  |
| -------------------- | ----------------------------------------- |
| `HEARTBEAT`          | reply `HEARTBEAT_OK`                      |
| `STATUS,...`         | update internal state; no immediate reply |
| `STEP_COMPLETED,...` | optionally send next `GET_STATUS`         |
| `PATH_DATA_RECEIVED` | enable `ExecutePath()`                    |
| `PATH_COMPLETED,...` | disable `ExecutePath()`                   |
| *invalid or timeout* | send `ERROR, <code>, <message>`           |

---

## 3. Error Handling

### 3.1 Format

```text
ERROR, <code>, <message>
```

### 3.2 Error Codes

* `100` – Unknown request
* `101` – Invalid parameters
* `102` – Manipulator ID out of range
* `103` – Trajectory parse failure
* `104` – Motion execution failure
* `105` – Timeout waiting for response

### 3.3 Behavior

* Validate all inputs; on failure, respond with `ERROR, code, message` immediately.
* Log errors with timestamp and full payload.
* Gracefully abort long-running operations on error and notify peer.
* Support future protocol versioning via optional `v<major>.<minor>` prefix.