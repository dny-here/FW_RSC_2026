# Design — Action-based, self-contained `airdrop_planning` node

**Date:** 2026-07-24
**Branch:** `feat/airdrop-action` (off `main`)
**Status:** approved design, pending spec review

## 1. Problem

On `main`, the `airdrop_planning` node and `target_recognition` do **not** share an
interface (the "inconsistent merge" documented in `CLAUDE.md`):

- `target_recognition` is an **action client** of `execute_airdrop`
  (`interfaces/action/TargetAirdrop`); its goal sets `bay_index`, it reads feedback
  `current_status.{state,distance_to_release,wind_speed}` and result `drop_successful`.
- `main`'s `airdrop_planner_node` is **service/topic based**: it provides an
  `airdrop/start` `StartAirdrop` service, subscribes to a `target_estimate` topic that
  nobody publishes, and emits a `drop_command` `UInt8` topic. It exposes **no action
  server** and sends **no vehicle commands at all**.
- `main`'s `TargetAirdrop.action` goal has **no `bay_index` field**, so a clean rebuild
  of `interfaces` breaks `target_recognition`'s compile (it currently only links against
  stale build artifacts).

We want `airdrop_planning` to become the **self-contained action server** shown in the
system-architecture diagram: it takes the vision goal, flies the truncated-Dubins
approach by streaming GUIDED position setpoints (ArduPilot L1/TECS does the tracking —
we do **not** implement our own LOS guidance), fires the drop, and resumes AUTO. The
team runs only `airdrop_planning` + `target_recognition` on the Raspberry Pi, so there is
**no separate `mission_bridge` node** — its setpoint/mode responsibilities fold into this
node.

## 2. Goals / non-goals

**Goals**
- `airdrop_planning` is an `rclcpp_action` **server** of `execute_airdrop`, matching the
  goal/feedback/result `target_recognition` already uses.
- The node owns the full drop: plan → GUIDED approach via `DO_REPOSITION` → drop via a
  `DropPayload` service → resume AUTO — and streams `AirdropStatus` feedback at 10 Hz.
- Port the corrected, ROS-free release-point geometry (loiter on `s`, entry-capture at
  `p`) so the drop trigger actually fires.

**Non-goals (explicitly deferred / out of scope)**
- No AUTO-mission orchestration or loiter-2× counting in this node — that lives in the
  Mission Planner AUTO waypoints. `target_recognition` fires the goal when it has a
  locked target.
- No `DropAuthorization` gate / authorization revocation on abort (safety item already
  deferred to the SITL phase in `CLAUDE.md`).
- No re-tuning of the ±50 m geofence vs. `approach.distance` = 300 m — kept at 300 m for
  the first SITL observation (`CLAUDE.md` open constraint).
- Simulation stack (`feat/simul`) is out of scope; only branch-agnostic header math is
  ported.

## 3. Architecture

Two companion nodes on the Raspberry Pi:

```
target_recognition ──execute_airdrop goal(gps_estimation, bay_index)──▶ airdrop_planner_node
   (action client)  ◀──feedback AirdropStatus @10Hz ── / ── result drop_successful ──┘  (action server)

airdrop_planner_node ⇄ MAVROS
   subs:  mavros/global_position/global        (sensor_msgs/NavSatFix)     — current position
          mavros/global_position/rel_alt       (std_msgs/Float64)          — altitude (relative frame)
          mavros/local_position/velocity_local (geometry_msgs/TwistStamped, ENU) — ground velocity
          mavros/wind_estimation               (geometry_msgs/TwistWithCovarianceStamped)
   cmds:  mavros/cmd/command_int (MAV_CMD_DO_REPOSITION)  — GUIDED destination + orbit
          mavros/set_mode                                 — GUIDED entry backstop / resume AUTO

airdrop_planner_node ──DropPayload.srv(bay_index)──▶ drop-mechanism / relay node
```

The AUTO mission (corridor, loiter 2×, mapping) is flown from Mission Planner waypoints.
When the vision node locks a target it dispatches the goal; the node takes over in GUIDED
for the drop, then returns the plane to AUTO.

## 4. Guidance: L1, not our own LOS

The node **never** computes roll/pitch/throttle. It sends a position setpoint and lets
ArduPlane's L1 (lateral) + TECS (longitudinal) controllers track it.

`DO_REPOSITION` as **COMMAND_INT** via `mavros/cmd/command_int` (the only mechanism that
works on fixed-wing — `setpoint_position/global` discards lat/lon in
`handle_set_position_target_global_int()`; see `CLAUDE.md`):

| field | value | meaning |
|---|---|---|
| `frame` | `6` | `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` (altitude relative → no geoid climb) |
| `command` | `192` | `MAV_CMD_DO_REPOSITION` |
| `param1` | `0` | ground speed 0 → vehicle default |
| `param2` | `1` | `MAV_DO_REPOSITION_FLAGS_CHANGE_MODE` → ArduPlane enters GUIDED itself (no race with `set_mode`) |
| `param3` | `loiter_radius` | orbit radius (sent explicitly → tangent geometry consistent by construction) |
| `param4` | `0` | yaw 0/NaN → clockwise |
| `x`,`y` | lat,lon ×1e7 | destination (int32, COMMAND_INT scaling) |
| `z` | `release_alt_agl` | altitude in the relative frame |

"Koreksi bila salah": it is a **command, not a stream** — sent once per target change,
retried only if the FCU rejects it (logged). Resume AUTO via `mavros/set_mode`.

## 5. FSM

Reuses the existing `AirdropStatus` state enum on `main` (it already defines
`STATE_LOITER` and all fields the feedback needs). Single folded FSM driven by a timer.

| State | Behaviour |
|---|---|
| `STATE_IDLE` | No active goal. A new goal is **rejected** unless: state is IDLE, required MAVROS data is present, and `bay_index < num_bays`. |
| `STATE_PLANNING` | Convert target lat/lon → local NED about the home datum. `solve()`: approach heading **into the wind**, build truncated-Dubins geometry → loiter center `s`, entry point `p`, release point. `DO_REPOSITION` to **`s`** (radius `r`, CW, `param2=1` to enter GUIDED). → `STATE_TRANSIT`. |
| `STATE_TRANSIT` → `STATE_LOITER` | Fly toward `s`, then orbit CW. Each tick evaluate **`entryCaptured(plan, uav, course, gate)`** — on a CW orbit of radius `r` about `s`, `p` is the tangent point where cross-track = 0 **and** course = `approach_heading` coincide exactly once per lap. |
| `STATE_FINAL_APPROACH` | On capture: **`solveAlongHeading()`** re-solves the release point using the vehicle's **actual ground speed** (heading unchanged, wind not re-sampled — per paper). `DO_REPOSITION` to an **overshoot point** `overshoot_distance` beyond the release point along `approach_heading`, so the pass through the release point is straight (zero commanded cross-track). Fire `DropPayload.srv(bay_index)` when inside the release **proximity circle**. → `STATE_RELEASED`. If the vehicle passes the release point without firing → `STATE_MISSED`. |
| `STATE_RELEASED` | Await `DropPayload` ack (≤ `drop_ack_timeout`). Success → **resume AUTO** → `goal_handle->succeed(drop_successful=true)`. Failure/timeout → resume AUTO → `abort`. |
| `STATE_MISSED` | `replans++`. `replans < max_replans` → `STATE_PLANNING` (re-solve from current position). Else → `abort`. |

Feedback (`AirdropStatus current_status`: `state`, `target_id`, `payloads_remaining`,
`distance_to_release`, `wind_speed`, `wind_direction`) is published at 10 Hz throughout.

On **any** abort or cancel the node resumes AUTO, so the plane is never left orphaned in a
GUIDED orbit.

## 6. Interface changes (`interfaces` package)

- `action/TargetAirdrop.action`: **add `uint8 bay_index`** to the goal (after
  `gps_estimation`). Unblocks a clean `target_recognition` compile. Feedback
  (`AirdropStatus current_status`) and result (`bool drop_successful`) already match.
- `srv/DropPayload.srv`: **new** — request `uint8 bay_index`; response
  `bool success`, `string message`. The diagram's "Drop Service"; replaces the
  `drop_command` `UInt8` topic.
- `msg/AirdropPlan.msg`: **add** `entry_point_lat/lon`, `release_point_lat/lon`,
  `loiter_center_lat/lon` (`float64`, WGS-84) for telemetry/logging.
- `srv/StartAirdrop.srv`: **remove** (obsolete — the action replaces it).
- `CMakeLists.txt`: register `srv/DropPayload.srv`, drop `srv/StartAirdrop.srv`.
- **Not** adding `DropAuthorization.msg` (mission-orchestration; out of scope).

## 7. `airdrop_planning` package changes

- **Port** (from `feat/airdrop-planning`, ROS-free headers/tests):
  - `include/airdrop_planning/release_point_solver.hpp` — `entryCaptured()`,
    `CaptureGate`, `solve()` / `solveAlongHeading()` / `buildPlan()`.
  - `test/test_release_point_solver.cpp` — flies a simulated orbit and asserts capture
    **happens** around `loiter_center` and **never** around `entry_point`, across many
    wind directions; plus a mismatched-radius test.
  - `test/test_geo_utils.cpp`.
  - `ballistic_model.hpp` / `geo_utils.hpp` are unchanged (already on `main`).
- **Rewrite** `src/airdrop_planner_node.cpp`:
  - `rclcpp_action::Server<TargetAirdrop>` with `handleGoal` / `handleCancel` /
    `handleAccepted`; a `GoalHandle` member; `finish()` maps result codes to
    `succeed`/`abort`/`canceled`.
  - MAVROS subs (position, rel_alt, `velocity_local`, wind) — velocity read from
    `velocity_local` (ENU), **not** `odom`.
  - Clients: `mavros_msgs/CommandInt`, `mavros_msgs/SetMode`,
    `interfaces/DropPayload`; a `DropAck` sub-state (`NONE/PENDING/OK/FAILED`).
  - The folded FSM (Section 5), timer-driven.
- `config/airdrop_params.yaml`:
  - `approach.release_alt_agl` 50 → **100** (KRTI 100 m AGL); add
    `approach.overshoot_distance`.
  - `release`: add `heading_tolerance`, `speed_reduction`, `gps_latency`,
    `drop_ack_timeout` (keep `proximity_radius`, `mechanism_latency`,
    `entry_capture_radius`).
  - `mission`: add `max_replans`, `num_bays`.
  - `topics`: replace `odom` with `velocity`; add `command_int`, `set_mode`,
    `drop_service`.
- `package.xml` / `CMakeLists.txt`: add `rclcpp_action`, `mavros_msgs`.

## 8. Robustness (repo's own lessons)

- Every `rclcpp::Time` member is assigned from `now()` in the constructor (mixed
  time-source subtraction throws and kills `spin()` silently — `CLAUDE.md`, 2026-07-23).
- Every timer/callback body is wrapped in try/catch so a fault becomes a log line, not a
  silent corpse (0-byte log).
- `DO_REPOSITION` rejects are logged and retried (common cause: not in GUIDED, or
  destination outside geofence). `DropPayload` timeout → clean abort.

## 9. Testing

- ROS-free gtest (`colcon test --packages-select airdrop_planning`):
  `ballistic_model`, `geo_utils`, and `release_point_solver` (capture must fire around
  `s`, never around `p`).
- Full goal → drop cycle verified in SITL later (deferred, per the geofence/authorization
  open items).

## 10. Data-flow summary

```
[AUTO mission waypoints] ── corridor · loiter 2× · mapping ──▶ (plane in AUTO)
target_recognition: lock target (≈75 frames) ── execute_airdrop goal(target, bay_index) ─▶
airdrop_planner_node:
   PLANNING     → DO_REPOSITION(s, r, CW, CHANGE_MODE)         [enter GUIDED]
   TRANSIT/LOITER → orbit s, entryCaptured() at p
   FINAL_APPROACH → solveAlongHeading(actual ground speed)
                  → DO_REPOSITION(overshoot beyond release)
                  → proximity circle → DropPayload.srv(bay_index)
   RELEASED     → set_mode(AUTO) → succeed(drop_successful=true)
   (MISSED → replan ≤ max_replans; any abort → resume AUTO)
   feedback AirdropStatus @10Hz throughout
```
