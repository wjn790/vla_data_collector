# SVT Long-Task Collection And Execution

## Paired B0-S1-B1-S2 Collection

Use `supervisor/paired_collection_controller.py` for the first paired dataset.
It creates one continuous source episode covering S1, concurrent B1, and S2
under one immutable `parent_attempt_id`. The camera, ROS, and hand capture
processes start once after B0 and stop once after S2. `paired_segments.json`
records the non-overlapping S1 and S2 timestamp/frame ranges for later curation;
the continuous source must not be converted as one atomic skill. B0 is not
recorded. B1 remains an audited base-only prior and never enters the VLA action
label.
The right-hand controller mapping is fixed to:

- right A (buttons[11]): run B0, wait for zero feedback, then start continuous
  S1; press right A again after the handle is secure to write the grasp marker
  without stopping S1;
- right B (buttons[12]): run B1 while the single collector keeps recording;
  after B1 reaches zero feedback the controller writes the S2 boundary without
  closing any camera; press right B again to stop the continuous collector;
- right C (buttons[13]): latched base emergency stop and pair abort;
- right D (buttons[14]): reserved and ignored.

The paired controller starts with F710 active and keeps its own
`/svtrobot_cmd` publisher silent. On the first right-A press it disables F710
through `/f710/enable`, waits for the all-stop, and temporarily stops
`svtrobo-f710.service`. It keeps exclusive base control through B0, S1, B1, and
S2. After the second right-B press completes S2, it sends the final zero segment,
restores F710, and resets to `IDLE`; the operator can drive back to the start and
press right A for the next pair without restarting the script. A right-C
emergency or safety fault keeps F710 disabled until script exit. Any unknown
base publisher still blocks execution. The qnbot exoskeleton bridge,
retargeting, and websocket processes remain running. In paired mode, the hand button configuration also
disables the old right-C Wuji toggle and the generic left-A/B collector.
The paired controller embeds the existing `HandButtonController`, acquires its
exclusive dual-hand command lease, and writes the normal hand action log. Do not
start `o6_button_controller.py` separately. Left C (`buttons[7]`) toggles O6
action 4; the right-hand A/B/C/D mapping above remains owned by the paired
controller.

The default command is a pure state-machine/configuration dry run and never
creates a ROS command publisher:

```bash
cd /home/svt/lingbot_data_collector
/home/svt/miniconda3/bin/python supervisor/paired_collection_controller.py
```

Real execution is explicit and remains blocked until the active B0/B1 entries
are calibrated and checksummed, and B1 has a maximum retreat-distance limit:

```bash
cd /home/svt/lingbot_data_collector
/home/svt/miniconda3/bin/python supervisor/paired_collection_controller.py \
  --execute --acknowledge-motion SVT_PAIRED_COLLECTION_V1
```

S1 must record for at least five seconds before the second right-A marker. The
controller uses `--no-offline-align`, and there is no recorder restart at the
S1-to-B1 or B1-to-S2 boundary. S2 must record for at least five seconds after
its boundary before the final right-B press is accepted. After the pair is
complete, read `source_episode` and `segments_file` from
`paired_attempts/PARENT_ID.json`. Run offline alignment on the continuous source
before materializing or converting the two ranges. The source manifest is
marked `training_eligible_as_atomic_episode: false` so it cannot be mistaken for
a single S1 demonstration.

B0 monitors joint-feedback freshness but does not apply an arm-drift limit. B1
allows intentional left-arm motion and load without an arm-pose or joint-effort
abort. Right C, stale feedback, a base-prior timeout or distance violation, or
another base publisher immediately latches zero base velocity and aborts the
pair.

## Atomic Collection

Each recording contains one versioned skill, at least one second of stable
frames at each edge, and at least 75 aligned frames. The collector aborts an
atomic attempt if it observes a nonzero base command, except S1 v2 when invoked
with `--allow-base-motion`; that exception is bound to its declared B1 prior.

```bash
cd /home/svt/lingbot_data_collector
/home/svt/miniconda3/bin/python collect_episode.py \
  --skill-id S1 --attempt-outcome unreviewed \
  --perturbation standard --hand-control-mode button_primitive \
  --allow-base-motion
```

S2 starts with the base stopped and the left hand still grasping the handle.
The demonstration releases the handle, keeps the fingers open, uses the back
of the left hand and left-arm motion to push the door slightly farther open,
unloads contact, and ends with the left arm clear before B2 can move the base.
Do not mix palm, fingertip, knuckle, or forearm pushes into S2 recordings.

S5 must use the index-only glove mode. It latches the left O6 command and 19 of
20 Wuji joints; only raw Wuji index 4 is driven by the glove:

```bash
cd /home/svt/glove_control
/home/svt/miniconda3/bin/python mixed_glove_teleop.py \
  --control-mode s5_index_only
```

Then record with `--skill-id S5 --hand-control-mode s5_index_only`. Button,
full-glove, S5, and supervisor modes share one exclusive command lease. A
second hand publisher fails before enabling hardware.

## Calibrate Base Priors

The candidate streams extracted from
`/home/svt/action_records/2026-07-22/105826_demo_112/base_stages_15hz_v1`
are registered under each prior's `candidate` field. Candidate registration is
not replay approval: the active `stream` and `sha256` remain empty and
`calibrated` remains false until all listed requirements are complete. B3 also
exceeds its current 25 second duration limit and must not be promoted as-is.

During B0/B1 collection transitions, the exoskeleton remains the active arm/hand
command source; this paired collector does not latch arm or hand commands. B0
does not apply an arm-drift limit. During B1, S1 remains open and records the
left-arm/left-hand command corrections. There is no drift-from-initial-pose
limit because motion is intentional, and neither pose change nor excessive load
aborts B1. Stale joint feedback still aborts the pair and latches zero base
velocity. Keep
holding the S1 grasp through B1; only start the S2 release and back-of-hand push
after the controller reports that the S2 segment has started in the same
collector. That transition does not reopen ZED or either RealSense camera.

Record a new action record, select the exact base-only window, and extract only
`/svtrobot_cmd`. This does not replay the rosbag or move the robot.

```bash
/home/svt/miniconda3/bin/python supervisor/extract_base_prior.py \
  /home/svt/action_records/YYYY-MM-DD/SESSION/rosbag \
  /home/svt/lingbot_data_collector/base_priors/B0.jsonl \
  --start-sec START --end-sec END
```

Copy the reported SHA-256 into `config/base_priors.yaml`, set a version and
`calibrated: true`, and complete the 20-trial repeatability record. B1 also
requires a maximum-retreat threshold. Empty or
unchecksummed priors can never run in real mode.

## Supervisor

### Policy Deployment Contract

Real-robot inference must use a WJN deployment contract that binds the exact
checkpoint, robot config, normalization file, and training config. The matching
values are recorded under `policy` in the selected inference YAML. On connect,
SVT verifies the server's contract ID, exact model path, normalization SHA-256,
robot-config SHA-256, and training-config SHA-256 before executing a reset or
publishing a policy action. A missing or mismatched value is a hard startup
failure.

Never change only `model_path` or only `robot_config`. When selecting a new
checkpoint, first create its WJN contract, then update all policy contract
fields in the SVT inference YAML together. Do not bypass this check for a
real-robot test.

Config validation and the state-machine simulation never publish commands:

```bash
/home/svt/miniconda3/bin/python supervisor/long_task_supervisor.py
```

Real execution is deliberately explicit and remains blocked until all four
priors are calibrated:

```bash
/home/svt/miniconda3/bin/python supervisor/long_task_supervisor.py \
  --execute --acknowledge-motion SVT_LONG_TASK_V1
```

The supervisor checks command-topic ownership, holds arm/hand targets in every
base state, keeps base velocity zero in every skill state, executes three of
50 action frames at 15 Hz, enforces executor masks and arm limits, and enters a
latched manual-recovery hold on a 500 ms policy timeout or safety fault. It also
keeps the final right-hand grasp active until the operator exits.

## Evaluation

Store one JSON object per independent trial and summarize it with:

```bash
/home/svt/miniconda3/bin/python supervisor/summarize_evaluation.py results.jsonl
```

Every long-chain record must include outcomes for S1 through S6. The report
enforces per-skill thresholds, 30 standard plus 20 perturbation chain trials,
at least 70 percent overall chain success, and zero safety violations.

## S4-S6 continuous collection (s456_collection_controller)

One take records S4, S5, and S6 continuously with a single collector; the
controller never loads a base prior, so position the robot manually before
each take. The base is latched at zero and F710 is disabled from the first
right A until the take ends. Both hands are commanded by the mixed glove
teleop process (it owns the dual-hand lease); start it once per session:

```bash
/home/svt/miniconda3/bin/python /home/svt/glove_control/mixed_glove_teleop.py \
  --control-mode full_glove
```

The controller switches the teleop hand mode mid-take by writing
`/tmp/svt_hand_control_mode` (the teleop watches it every cycle and prints a
`[mixed] control mode switch` line on every change).

Button sequence (right hand gamepad):

1. Right A: start the take; wait for `[S456 READY]`, keep 1 s stable start.
2. Teach S4 (right glove grasps the spray; left glove keeps holding the box).
3. Right B at the stable S4 end: marks the S4/S5 boundary and switches the
   teleop to `s5_index_only` (left hand and all Wuji joints except raw index
   4 latch at the current pose).
4. Teach S5: aim, one full index press and release, right hand still.
5. Right B again: marks S5/S6 and restores `full_glove`.
6. Teach S6 (left glove places the box; right glove keeps the spray).
7. Right A: stops, finalizes `paired_segments.json` with the S4/S5/S6 frame
   ranges, and releases the base.

Right C is the latched emergency stop; right D rewrites the expected hand
mode file if a switch was missed. Each segment must reach at least
`minimum_skill_recording_sec` (5 s); shorter segments abort finalization.
Reset the scene between takes: open the left glove, replace the box, close
the left glove onto it, return the spray, then press right A again.

Dry run (no ROS, no motion):

```bash
/home/svt/miniconda3/bin/python supervisor/s456_collection_controller.py
```

Real operation requires the motion acknowledgement because the controller
publishes zero Twist and manages F710:

```bash
/home/svt/miniconda3/bin/python supervisor/s456_collection_controller.py \
  --execute --acknowledge-motion SVT_S456_COLLECTION_V1
```

Do not run the standalone `o6_button_controller.py` during S456 takes; the
teleop process holds the hand command lease and both buttons would conflict.
