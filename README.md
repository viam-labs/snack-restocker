# Module snack-restocker

Drives an arm and gripper through a full snack restock: open the drawer, pick up a snack, drop it in the drawer, and push the drawer shut. Every arm move goes to a pose saved on an [`erh:vmodutils:arm-position-saver`](https://app.viam.com/module/erh/vmodutils) switch. A state machine tracks which phase the restock is in.

## Models

This module provides the following model(s):

- [`viam:snack-restocker:restocker`](viam_snack-restocker_restocker.md) - Generic service that runs the restock sequence and exposes its state machine through DoCommand.
- `viam:snack-restocker:pick-status` - Sensor whose readings are a restocker's live `pick_snack` / `pick_and_place` status (see [`get_pick_status`](#get_pick_status)).

## Configuration

The service takes no attributes:

```json
{}
```

It requires these resources on the machine, under these exact names:

| Resource | Type | Used for |
|----------|------|----------|
| `gripper` | gripper | Opening and closing around the snack, via `DoCommand({"set": <0-850>})` (0 = fully closed, 850 = fully open). |
| `open-drawer-0-position` | switch | Find the drawer, and the final resting pose. |
| `open-drawer-1-position` … `open-drawer-6-fullpull` | switch | Open the drawer (6 poses). |
| `restock-1-find` … `restock-5-drop` | switch | Pick up the snack and drop it in the drawer (5 poses). |
| `close-1-set-back`, `close-2-lower`, `close-3-push1`, `close-4-push2` | switch | Push the drawer shut. |

These are optional. Without them the service still starts and `restock` works:

| Resource | Type | Used by | Used for |
|----------|------|---------|----------|
| `drawer-open` | generic service (`viam-labs:restock-bot:fixture-teacher`) | `restock_fixture` | Opening the drawer with its taught `open` motion. |
| `draw-closer` | generic service (`viam-labs:restock-bot:fixture-teacher`) | `restock_fixture` | Closing the drawer with its taught `close` motion. |
| `sam3-vision` | vision service (`viam:sam3-detector:sam3-segments`) | `pick_snack` | Finding snack bags. Needs a version that supports the `get_object_point_clouds` DoCommand. |
| `builtin` | motion service | `pick_snack` | Moving the gripper TCP to the bag. |
| `gripper-tcp` | frame | `pick_snack` | The point the motion planner moves onto the bag. |

Each switch is moved with `SetPosition(2)`, which sends the arm to that switch's saved pose. The module waits 1 second after every move and every gripper command.

## State machine

The states, in order:

| State | What happens |
|-------|--------------|
| `idle` | Resting, waiting for a restock. |
| `finding_drawer` | Close the gripper fully, then go to `open-drawer-0-position`. |
| `opening_drawer` | Run `open-drawer-1-position` through `open-drawer-6-fullpull`. |
| `picking_up_snack` | Go to `restock-1-find`, open the gripper, go to `restock-2-reach-snack`, close the gripper (30) on the snack, then go to `restock-3-lift-snack`. |
| `restocking` | Go to `restock-4-view-drawer`, then `restock-5-drop`, open the gripper to release the snack, then close it fully. |
| `closing_drawer` | Go to `close-1-set-back`, `close-2-lower`, `close-3-push1`, `close-4-push2`, `close-1-set-back` again, then back to `open-drawer-0-position`. |
| `done` | The drawer is closed and the arm is back at the find pose. |

Allowed transitions: each state can move to the next one in the list, `done` can move to `idle`, and any state can move back to `idle` (abort/reset). Any other transition is rejected unless forced.

The state resets to `idle` whenever the service is reconfigured.

## DoCommand

Every command returns its result in the response map. Errors, such as a rejected transition, appear in an `error` field instead of failing the call.

The state commands (`get_state`, `advance_state`, `set_state`, `reset_state`) all return the same base report:

| Field | Description |
|-------|-------------|
| `state` | Current state. |
| `next_state` | The next state in the sequence, or `""` from `done`. |
| `allowed_transitions` | States the current state can move to without `force`. |

### `restock`

Runs the whole restock sequence from start to finish. **This moves the arm.** The call returns only after the sequence finishes or fails.

The sequence always starts from `idle`, whatever state the machine was in, then walks through every state and ends back in `idle`.

```json
{ "restock": true }
```

On success:

```json
{
  "restock": "complete",
  "steps_completed": 23,
  "steps": ["finding_drawer: gripper: do_command(set=0)", "finding_drawer: open-drawer-0-position: set_position(2)", "..."],
  "state": "idle"
}
```

If any move or gripper command fails, the sequence stops where it is, the state resets to `idle`, and the call returns:

```json
{ "restock": "failed", "error": "<reason>", "state": "idle" }
```

The arm is left wherever it stopped.

### `restock_fixture`

Runs the same restock sequence as `restock`, except the drawer is opened and closed by the `drawer-open` and `draw-closer` services instead of the `open-drawer-*` and `close-*` poses. Those services find the drawer's AprilTag and replay motions taught relative to it, so they still work if the drawer moves. **This moves the arm.** `restock` is unchanged.

```json
{ "restock_fixture": true }
```

| State | What happens |
|-------|--------------|
| `finding_drawer` | Close the gripper fully. `drawer-open` moves to its own view pose to find the drawer. |
| `opening_drawer` | `drawer-open`: `{"cmd": "run", "motion": "open"}`. |
| `picking_up_snack` | Same as `restock`. |
| `restocking` | Same as `restock`. |
| `closing_drawer` | `draw-closer`: `{"cmd": "run", "motion": "close"}`, then go to `open-drawer-0-position`. |

It uses the same state machine and returns the same shape as `restock`, under a `restock_fixture` key. If a drawer motion reports an `error`, the sequence stops there, the state resets to `idle`, and it returns `{"restock_fixture": "failed", "error": "<reason>", "state": "idle"}`.

To check where the drawer motions will go without running them, send `{"cmd": "run", "motion": "open", "dry_run": true}` to `drawer-open` (or `"close"` to `draw-closer`) directly.

### `get_state`

Returns the current state report. It does not change anything.

```json
{ "get_state": true }
```

```json
{ "state": "idle", "next_state": "finding_drawer", "allowed_transitions": ["finding_drawer"] }
```

### `advance_state`

Moves the state machine one step forward along the sequence. It only changes the recorded state and **does not move the arm**. Returns the state report plus `"advanced": true`, or an `error` field if there is no next state (from `done`).

```json
{ "advance_state": true }
```

### `set_state`

Sets the state machine to a specific state. It only changes the recorded state and **does not move the arm**. The transition must be allowed from the current state unless `force` is `true`.

```json
{ "set_state": "restocking", "force": false }
```

Returns the state report plus `"set_state": true`. If the transition isn't allowed, it returns an `error` field. If the state name is unknown, it returns `error` and `valid_states`.

### `reset_state`

Forces the state machine back to `idle` from any state. It **does not move the arm**. Returns the state report plus `"reset": true`.

```json
{ "reset_state": true }
```

### `pick_snack`

Grabs the snack bag SAM3 is most confident about, or the next most confident one it can reach. **This moves the arm.** It is separate from `restock`: it does not use or change the state machine.

```json
{ "pick_snack": true }
```

The steps:

1. Go to `restock-1-find` so the camera sees the bin.
2. Ask `sam3-vision` for point clouds, which come back highest confidence first, and keep the ones that are plausible bags in the bin:
   - its position is in the `world` frame,
   - its center is inside the `bin` obstacle's footprint (centered at (250, -439), 555 × 355 mm) and between z = -50 and 450 mm,
   - its largest side is 40 to 400 mm.

   Segments that fail these checks come from bad depth readings, such as far-away background pixels inside a mask. They are skipped and counted in `rejected`. If none pass, the command fails without moving toward a bag.
3. Open the gripper.
4. For each plausible bag, most confident first (up to `max_bags`), and for each gripper rotation (0°, then 90° about the vertical):
   1. Move `gripper-tcp`, pointing straight down, to the hover point above the bag: `approach_mm` above its top, but no higher than `hover_above_rim_mm` above the bin rim (z = 360 mm), and always at least 30 mm above the top of the bag.
   2. Descend in a straight line to `grasp_depth_mm` below the top of the bag.

   If the motion planner refuses the hover or the descend, for example because the open claws would hit a bin wall, the refusal is recorded in `attempts` and it tries the next rotation, then the next bag. A rotation of 90° turns the claws parallel to a wall the bag is lying against. After both rotations straight down, each rotation whose **hover** was refused is retried with the gripper tilted 15° off vertical, pointing away from the bin center so the wrist leans in toward the middle of the bin; the descend and lift then keep that tilt. A refused descend is never retried tilted. The arm stays at the last pose it reached, and the next move is planned from there.
5. Once at a bag: close the gripper (30), then lift straight back up to the hover point. A failure here is not retried.

The arm stays at the hover point holding the bag.

Optional fields:

| Field | Default | Description |
|-------|---------|-------------|
| `dry_run` | `false` | Go to `restock-1-find` and find the bags, but do not open the gripper or move toward a bag. Returns the poses for the most confident plausible bag at rotation 0. |
| `hover_only` | `false` | Stop at the first hover point it can reach, with the gripper open. |
| `approach_mm` | `120` | Height of the hover point above the top of the bag, before the rim cap. |
| `hover_above_rim_mm` | `50` | Highest hover point, above the bin rim. A high hover far from the arm base has no straight-down IK solution. |
| `grasp_depth_mm` | `20` | How far below the top of the bag the gripper closes. |
| `max_bags` | `3` | How many plausible bags to try before giving up. |

Test in this order: `dry_run`, then `hover_only`, then a full pick.

On success:

```json
{
  "pick_snack": "complete",
  "bag": 1,
  "label": "snack bag",
  "confidence": 0.93,
  "rotation_deg": 90,
  "tilt_deg": 0,
  "candidates": 27,
  "rejected": 4,
  "plausible": 3,
  "center_mm": { "x": 121, "y": -419, "z": 181 },
  "top_z_mm": 264,
  "hover_mm": { "x": 121, "y": -419, "z": 384 },
  "grasp_mm": { "x": 121, "y": -419, "z": 244 },
  "attempts": [
    {
      "bag": 1,
      "confidence": 0.93,
      "center_mm": { "x": 121, "y": -419, "z": 181 },
      "rotation_deg": 0,
      "tilt_deg": 0,
      "stage": "descend",
      "error": "descend to snack: ... violation between bin:bin-left and gripper:claws ..."
    }
  ],
  "steps": ["..."],
  "steps_completed": 8
}
```

- `bag` is the rank of the bag it grabbed among the plausible bags (1 = most confident). `rotation_deg` and `tilt_deg` are the gripper rotation and tilt (0 or 15) it used.
- `candidates` is the number of SAM3 segments, `rejected` how many failed the plausibility checks, and `plausible` how many bags it would try (at most `max_bags`).
- `attempts` lists every refused hover or descend.

`pick_snack` is `"planned"` for a dry run and `"hovering"` for `hover_only`. If no bag or rotation can be reached, it returns `"pick_snack": "failed"` with an `error` and the full `attempts` list, and the gripper is left open at the last pose reached. If no bag is found, `sam3-vision` returns an error, or the gripper or lift fails, it returns `{"pick_snack": "failed", "error": "<reason>"}` and the arm is left where it stopped.

### `pick_and_place`

Runs `pick_snack` and then drops the bag over the drawer, twice by default:

```json
{ "pick_and_place": true }
```

Each round grabs the best plausible bag with `pick_snack`, moves through `restock-4-view-drawer` and `restock-5-drop`, and opens the gripper to release it. It doesn't open or close the drawer. It takes every `pick_snack` option, plus `rounds` (default `2`).

It stops at the first round whose pick doesn't complete, without going to the drawer, and returns that round's status (`"failed"`, or `"planned"`/`"hovering"` for `dry_run`/`hover_only`). The result lists each round's pick result and place steps:

```json
{
  "pick_and_place": "complete",
  "rounds": 2,
  "rounds_completed": 2,
  "results": [{ "round": 1, "pick": { "pick_snack": "complete", "...": "..." }, "place_steps": ["..."] }, "..."]
}
```

### `get_pick_status`

Returns the live status of the current or last `pick_snack` / `pick_and_place` run. It answers while a pick is still moving the arm, so you can poll it to see the target before the arm reaches it and each refused move as it happens:

```json
{ "get_pick_status": true }
```

```json
{
  "run": 3,
  "command": "pick_snack",
  "running": true,
  "phase": "hovering",
  "message": "moving above bag 2 (0.95) at rotation 0 tilt 0",
  "seconds_in_phase": 1.4,
  "candidates": 28, "plausible": 22, "rejected": 6, "trying": 3,
  "target": { "bag": 2, "segment": 2, "confidence": 0.95, "center_mm": { "x": 250, "y": -314, "z": 157 }, "top_z_mm": 190, "rotation_deg": 0, "tilt_deg": 0, "label": "snack bag" },
  "attempts": 2,
  "last_refusal": { "bag": 1, "rotation_deg": 90, "tilt_deg": 0, "stage": "hover", "error": "..." },
  "events": [{ "at": "2026-11-19T19:02:11.4Z", "phase": "moving_to_view", "message": "moving to restock-1-find" }, "..."]
}
```

Phases, in order: `moving_to_view`, `segmenting`, `targets_found` (or `no_target`), then per try `hovering`, `descending`, `hover_refused` / `descend_refused`, `gripping`, `lifting`; for `pick_and_place` also `placing`, `dropping`, `round_complete` (with `round`, `rounds`, `rounds_completed`). The run ends in the command's final status (`complete`, `planned`, `hovering`, `failed`) with `running: false`, and the snapshot fields (`snapshot_binary_data_id` or `snapshot_error`) once uploaded. `events` keeps the last 50 phase changes. A new run replaces the previous one's status.

### `pick-status` sensor

To see the status in the CONTROL tab, capture it to data, or set triggers on it, add a sensor that reads it:

```json
{
  "name": "pick-status",
  "api": "rdk:component:sensor",
  "model": "viam:snack-restocker:pick-status",
  "attributes": { "restocker": "code-1" }
}
```

`Readings` returns `get_pick_status` without `events`, so each captured reading stays small. Configure data capture on its `Readings` (for example at 2 Hz) to keep a history of every pick in DATA.

### Unknown commands

Any other command returns an `error` field with a `usage` string, `supported_commands`, all `states`, and the current `state`.

## Status

`GetStatus` returns the current state:

```json
{ "state": "idle" }
```
