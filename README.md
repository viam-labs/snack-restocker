# Module snack-restocker

Drives an arm and gripper through a full snack restock: open the drawer, pick up a snack, drop it in the drawer, and push the drawer shut. Every arm move goes to a pose saved on an [`erh:vmodutils:arm-position-saver`](https://app.viam.com/module/erh/vmodutils) switch. A state machine tracks which phase the restock is in.

## Models

This module provides the following model(s):

- [`viam:snack-restocker:restocker`](viam_snack-restocker_restocker.md) - Generic service that runs the restock sequence and exposes its state machine through DoCommand.

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

### Unknown commands

Any other command returns an `error` field with a `usage` string, `supported_commands`, all `states`, and the current `state`.

## Status

`GetStatus` returns the current state:

```json
{ "state": "idle" }
```
