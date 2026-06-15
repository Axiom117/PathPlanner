# PathPlanner API Specification (v1.0)

## 1. Overview

This document defines the PathPlanner-side API for the modular closed-loop mechanism platform.

Unlike Controller Panel, PathPlanner does not currently expose a standalone TCP server interface. Its primary programmable surface is an in-process MATLAB class API centered on `PathPlannerClient`, plus an event contract used by the GUI and by automation scripts.

This specification is intended to align with the Controller Panel API document while reflecting the actual integration boundary implemented in this repository.

### 1.1 Scope

This document covers:

- The public MATLAB API exposed by `PathPlannerClient`
- Shared runtime data exposed through `PathPlannerParam`
- Event payloads and lifecycle events
- The downstream TCP commands PathPlanner emits to Controller Panel
- Error and status propagation behavior

### 1.2 Entry Point

The primary entry point is:

```matlab
client = PathPlannerClient();
```

This facade object initializes the following internal modules:

- `PathPlannerParam`
- `PathPlannerComm`
- `PathPlannerSyncOps`
- `PathPlannerAsyncTask`

---

## 2. Primary Interface: PathPlannerClient

### 2.1 Public Properties

### `isConnected`

- Type: `logical`
- Access: read-only external access
- Meaning: whether PathPlanner currently maintains an active TCP connection to Controller Panel

### 2.2 Public Events

All events use `PathPlannerEventData` as the payload unless noted otherwise.

| Event | Meaning | Typical Trigger | Data Payload |
| --- | --- | --- | --- |
| `StatusUpdate` | General runtime or communication update | Connection progress, command send, parse result, planning status | Usually empty |
| `TrajectoryReady` | Trajectory planning completed successfully | `PlanPath()` success | Downsampled controller-ready trajectory struct |
| `TrajectoryExecuted` | Controller reported execution completed | `PATH_COMPLETED` received | Usually empty |
| `ConnectionStateChanged` | Connection established or closed | `connect()` / `disconnect()` | Usually empty |
| `PathDataReceived` | Controller confirmed trajectory data receipt | `PATH_DATA_RECEIVED` received | Usually empty |
| `PathExecutionStarted` | Controller acknowledged path execution start | Path tracking start notification | Usually empty |
| `PathExecutionFailed` | Controller reported execution error | `ERROR` received during execution | Usually empty |
| `ConfigurationLoaded` | Configuration finished loading | Constructor / parameter initialization | Usually empty |

### 2.3 Public Methods

### Constructor

#### `client = PathPlannerClient()`

- Initializes all core modules.
- Loads configuration.
- Sets up event forwarding so the caller only listens to one facade object.

### connect

#### `success = client.connect()`

- Establishes the TCP connection to Controller Panel.
- Starts the background message listener.
- Sends a heartbeat for connection verification.
- Returns `true` on success, otherwise `false`.

Typical emitted events:

- `StatusUpdate`
- `ConnectionStateChanged`

### disconnect

#### `client.disconnect()`

- Closes the TCP connection.
- Stops the background message listener.
- Resets local connection state.

Typical emitted events:

- `StatusUpdate`
- `ConnectionStateChanged`

### GetStatus

#### `[status1, status2] = client.GetStatus()`

- Sends `GET_STATUS` to Controller Panel for the currently selected manipulator IDs.
- Parses the returned `STATUS` message.
- Updates the shared parameter object fields:
  - `XMC1`, `YMC1`, `ZMC1`
  - `XMC2`, `YMC2`, `ZMC2`
- Returns two structures:

```matlab
status1 = struct('id', id1, 'X', X1, 'Y', Y1, 'Z', Z1);
status2 = struct('id', id2, 'X', X2, 'Y', Y2, 'Z', Z2);
```

Notes:

- The numeric values returned here reflect controller-side position values as reported by `GET_STATUS`.
- If the controller responds with an error or malformed status packet, the method raises a MATLAB error.

### GetPose

#### `[pose, elapsedTime] = client.GetPose()`

- Refreshes controller position state through `GetStatus()`.
- Runs the active FK model.
- Updates the shared parameter object fields:
  - `X0`, `Y0`, `Z0`
  - `Phi0`, `Theta0`, `Psi0`
- Returns:
  - `pose`: a `1 x 6` pose vector
  - `elapsedTime`: FK execution time in seconds

Pose layout:

```text
[X, Y, Z, Phi, Theta, Psi]
```

### PlanPath

#### `success = client.PlanPath()`

- Reads the current pose from hardware using `GetPose()`.
- Reads the current target pose from `PathPlannerParam`.
- Calls the active IK model through `onCalcIK()` and `solverIK()`.
- Dynamically downsamples the raw trajectory.
- Caches a controller-ready version of the trajectory.
- Returns `true` on success, otherwise `false`.

On success, emits:

- `TrajectoryReady`

`TrajectoryReady` payload structure:

```matlab
struct(
    'id1', manipulatorID1,
    'id2', manipulatorID2,
    'qTime', qTime,
    'elapsedTime', elapsedTime,
    'qData', qDataControllerReady
)
```

### SendPath

#### `success = client.SendPath()`

- Reads the cached trajectory produced by `PlanPath()`.
- Splits the `N x 6` trajectory into two `N x 3` manipulator payloads.
- Sends one `PATH_DATA` command per manipulator.
- Waits for `PATH_DATA_RECEIVED` confirmation for each transmitted payload.
- Returns `true` on success, otherwise `false`.

On success, emits:

- `PathDataReceived`

### StartPath

#### `success = client.StartPath()`

- Reads the execution interval from `PathPlannerParam.interval`.
- Sends `START_PATH_CP` for the currently selected manipulator IDs.
- Marks execution as active and returns `true` if the command is sent successfully.
- Final execution result is asynchronous and reported through events.

Typical follow-up events:

- `PathExecutionStarted`
- `TrajectoryExecuted`
- `PathExecutionFailed`

### sendHeartbeat

#### `success = client.sendHeartbeat()`

- Sends `HEARTBEAT` to Controller Panel.
- Returns `true` if `HEARTBEAT_OK` is received.

### isTrajectoryReady

#### `ready = client.isTrajectoryReady()`

- Returns whether a valid trajectory is currently cached and ready for transmission or execution.

### getLastTrajectoryData

#### `data = client.getLastTrajectoryData()`

- Returns the last cached trajectory payload structure.
- This is the same controller-oriented trajectory structure exposed by the `TrajectoryReady` event.

### isPathExecuting

#### `executing = client.isPathExecuting()`

- Returns whether a path execution is currently in progress.

### getParameterSummary

#### `params = client.getParameterSummary()`

- Returns a summary structure with grouped configuration and runtime values.
- This is intended for inspection and UI display rather than model execution.

### getParamObj

#### `paramObj = client.getParamObj()`

- Returns the shared `PathPlannerParam` handle.
- This gives direct access to runtime state and selected configuration fields.
- Use with care because updates affect the entire live session.

---

## 3. Shared Runtime State: PathPlannerParam

`PathPlannerParam` is the shared runtime state object used across the entire PathPlanner session.

### 3.1 Current Pose State

Current mechanism pose fields:

- `X0`, `Y0`, `Z0`
- `Phi0`, `Theta0`, `Psi0`

### 3.2 Target Pose State

Target mechanism pose fields:

- `XTarget`, `YTarget`, `ZTarget`
- `PhiTarget`, `ThetaTarget`, `PsiTarget`

### 3.3 Manipulator State

Current controller-side manipulator position fields:

- `XMC1`, `YMC1`, `ZMC1`
- `XMC2`, `YMC2`, `ZMC2`

### 3.4 Core Session Configuration

Selected session configuration fields include:

- `controllerHost`
- `controllerPort`
- `connectionTimeout`
- `responseTimeout`
- `maxRetryAttempts`
- `manipulatorID1`
- `manipulatorID2`
- `modelFK`
- `modelIK`
- `interval`

### 3.5 Unit Conventions

PathPlanner currently spans multiple unit domains.

Stable integration rules are:

- Controller TCP status and trajectory payloads are controller-facing position values aligned with Controller Panel protocol semantics.
- `lastTrajectoryData.qData` is stored in controller-ready integer micrometer units.
- FK and IK model input/output units must remain internally consistent across the active model pair.

Because UI-facing units and model-facing units may differ by layer, integrations should prefer the method-level contracts in this document over assumptions based on GUI labels.

---

## 4. Event Payload Contract

All PathPlanner facade events use `PathPlannerEventData`.

Structure:

```matlab
eventData.Message    % string / char
eventData.Success    % logical
eventData.Data       % arbitrary payload
eventData.Timestamp  % datetime
```

Construction rules:

- `Success` defaults to `true`.
- `Data` defaults to empty.
- Errors and failures are typically reported by setting `Success = false` and providing a diagnostic message.

---

## 5. Controller Command Emission Map

Although PathPlanner does not expose a TCP server API of its own, it emits the following downstream controller commands to Controller Panel.

| PathPlanner Action | Command Sent to Controller | Expected Response |
| --- | --- | --- |
| Heartbeat | `HEARTBEAT` | `HEARTBEAT_OK` |
| Refresh manipulator status | `GET_STATUS,<id1>,<id2>` | `STATUS,<id1>,<X1>,<Y1>,<Z1>,<id2>,<X2>,<Y2>,<Z2>` |
| Send manipulator path | `PATH_DATA,<id>,<payload>` | `PATH_DATA_RECEIVED,<id>` |
| Start continuous path execution | `START_PATH_CP,<interval>,<id1>,<id2>` | Path tracking start notification, then `PATH_COMPLETED` or `ERROR` |
| Manual / batch step execution | `START_STEP,...` | `STEP_COMPLETED,...` or `ERROR,...` |

---

## 6. Typical Operation Sequences

### 6.1 Current Pose Acquisition

```text
GetPose()
  -> GET_STATUS
  -> update XMC1..ZMC2
  -> run FK model
  -> update X0..Psi0
  -> return pose
```

### 6.2 Full Planning and Execution Sequence

```text
PlanPath()
  -> refresh current pose
  -> run IK model
  -> cache trajectory
  -> emit TrajectoryReady

SendPath()
  -> send PATH_DATA for manipulator 1
  -> wait PATH_DATA_RECEIVED
  -> send PATH_DATA for manipulator 2
  -> wait PATH_DATA_RECEIVED
  -> emit PathDataReceived

StartPath()
  -> send START_PATH_CP
  -> emit PathExecutionStarted when controller acknowledges start
  -> emit TrajectoryExecuted on completion
  -> or emit PathExecutionFailed on controller error
```

---

## 7. Error Handling

### 7.1 General Rules

- Connection, planning, transmission, and start methods generally return `false` when a handled failure occurs.
- Synchronous query methods such as `GetStatus()` and `GetPose()` may raise MATLAB errors if the response is malformed or the connection is unavailable.
- Runtime modules also emit `StatusUpdate` with `Success = false` when an operation fails.

### 7.2 Error Sources

Typical failure sources include:

- No active TCP connection
- Heartbeat failure
- Controller timeout
- Malformed `STATUS` or `PATH_DATA_RECEIVED` response
- IK or FK simulation failure
- Attempting to send or start without a cached trajectory
- Controller-side execution error during path tracking

### 7.3 Recommended Integration Pattern

Consumers should treat PathPlanner as an event-driven API:

- Use method return values for immediate success or failure.
- Use events for long-running lifecycle milestones.
- Read shared runtime state from `getParamObj()` or `getParameterSummary()` when the latest synchronized values are needed.

---

## 8. Versioning Note

This document describes the current PathPlanner facade implemented in this repository.

If the project later adds an external TCP server or RPC interface on the PathPlanner side, that network API should be documented as a separate specification rather than overloading this document.