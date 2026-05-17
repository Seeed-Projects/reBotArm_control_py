# Gemini ER 1.6 In-Context Prompt: Seeed B601 Purple Block Pickup

Dataset:

```text
https://huggingface.co/datasets/andlyu/rebot_hackathon_v4
```

Use this file as an in-context prompt for `gemini-robotics-er-1.6-preview`.

The goal is to teach the model the Seeed reBot B601 action space from teleoperated examples. The model must output absolute B601 joint-position targets in degrees.

## Task

```text
Pick up the purple block.
```

## Robot

```text
seeed_b601_dm_follower
```

## Joint Order

```json
[
  "shoulder_pan.pos",
  "shoulder_lift.pos",
  "elbow_flex.pos",
  "wrist_flex.pos",
  "wrist_yaw.pos",
  "wrist_roll.pos",
  "gripper.pos"
]
```

All values are in degrees.

## Action Semantics

`action_chunk_target_deg` is a short sequence of absolute follower joint targets.

Each row:

```text
[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper]
```

Gripper convention:

```text
gripper.pos near -270 = open
gripper.pos near 0 = closed
```

Never output positive gripper values.

## Visual Context

If using images, attach either:

```text
contact_sheet.jpg
```

or paired frame examples from:

```text
frames/frame_0000_front.jpg
frames/frame_0000_wrist.jpg
frames/frame_0180_front.jpg
frames/frame_0180_wrist.jpg
frames/frame_0300_front.jpg
frames/frame_0300_wrist.jpg
frames/frame_0510_front.jpg
frames/frame_0510_wrist.jpg
frames/frame_0750_front.jpg
frames/frame_0750_wrist.jpg
```

The front/environment view gives global table layout. The wrist/mounted view gives local gripper alignment.

## In-Context Examples

### Example: Start / Ready

Input current_state_deg:

```json
[-4.011, 1.934, 0.361, 1.585, 2.153, -1.519, -1.279]
```

Desired action_chunk_target_deg:

```json
[
  [-2.9, -0.0, 0.0, 0.1, 0.4, -3.6, -2.4],
  [-2.9, -0.0, 0.0, 0.1, 0.4, -3.6, -1.8],
  [-2.9, -0.0, 0.0, 0.1, 0.4, -3.6, -2.4],
  [-2.9, -0.0, 0.0, 0.1, 0.4, -3.6, -2.4]
]
```

### Example: Approach

Input current_state_deg:

```json
[-2.896, -0.011, -3.967, 1.869, -6.874, -4.71, -3.005]
```

Desired action_chunk_target_deg:

```json
[
  [-2.8, -0.2, -15.5, 5.4, -11.1, -6.1, -3.0],
  [-2.7, -0.4, -16.3, 6.0, -11.4, -6.2, -3.0],
  [-2.8, -0.8, -17.2, 6.3, -11.7, -6.2, -3.0],
  [-2.8, -1.4, -18.0, 6.4, -12.0, -6.2, -3.0]
]
```

### Example: Near Object / Gripper Open

Input current_state_deg:

```json
[2.809, -99.963, -100.706, 49.123, -27.179, -16.273, -161.403]
```

Desired action_chunk_target_deg:

```json
[
  [11.1, -105.7, -100.4, 48.4, -27.3, -14.2, -162.0],
  [11.1, -105.9, -99.8, 47.8, -27.6, -13.7, -162.0],
  [11.1, -105.9, -99.2, 47.3, -27.6, -13.2, -162.0],
  [11.0, -106.0, -98.9, 46.9, -27.6, -12.8, -162.0]
]
```

### Example: Grasp / Hold

Input current_state_deg:

```json
[10.765, -120.268, -108.837, 36.949, -35.878, -3.071, -219.258]
```

Desired action_chunk_target_deg:

```json
[
  [10.8, -117.4, -105.7, 33.9, -43.5, 1.2, -214.8],
  [10.8, -117.3, -105.7, 33.9, -43.7, 1.3, -214.2],
  [10.8, -117.4, -105.8, 33.8, -44.3, 1.5, -214.2],
  [10.8, -117.7, -105.7, 33.6, -45.2, 1.7, -214.2]
]
```

### Example: Release / Finish

Input current_state_deg:

```json
[-7.683, -122.322, -128.027, 53.801, -37.124, -15.136, -173.205]
```

Desired action_chunk_target_deg:

```json
[
  [-18.9, -114.9, -130.8, 51.7, -34.0, -10.5, -106.8],
  [-20.8, -114.3, -130.6, 51.5, -34.0, -10.0, -93.6],
  [-21.7, -114.2, -130.4, 50.7, -33.9, -9.2, -76.8],
  [-22.2, -114.1, -129.8, 49.7, -33.8, -8.3, -60.6]
]
```

## Query Template

Current task:

```text
Pick up the purple block.
```

Current robot state in the required joint order:

```json
<CURRENT_STATE_DEG>
```

Return only valid JSON matching this schema:

```json
{
  "task": "pick up purple block",
  "joint_order": [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_yaw.pos",
    "wrist_roll.pos",
    "gripper.pos"
  ],
  "action_chunk_target_deg": [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  ],
  "confidence": 0.0,
  "phase": "start | approach | align | grasp | lift | place | release | finish",
  "reason": "one short sentence"
}
```

Rules:

- Output only JSON.
- Use absolute B601 joint targets in degrees.
- Each action row must contain exactly 7 numbers.
- Use the exact joint order above.
- Keep gripper values inside `[-270, 0]`.
- Do not output positive gripper values.
- Do not output Cartesian motion.
- Do not output deltas.
- Do not output Franka, DROID, SO101, or normalized actions.
