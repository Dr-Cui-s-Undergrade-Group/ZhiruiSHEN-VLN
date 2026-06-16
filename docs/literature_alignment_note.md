# Literature Alignment Note

Date: 2026-06-15

## Sources Read

The project guidance in `/home/bluepoisons/Downloads/README.md` points to the standard VLN and continuous-navigation baselines:

- Room-to-Room / R2R: Anderson et al., CVPR 2018.
- VLN-CE: Krantz et al., ECCV 2020.
- RxR: Ku et al., EMNLP 2020.
- ROS 2 Nav2 for the robot execution layer.

## Relevance To This Project

This project is closer to VLN-CE than classic discrete R2R because the robot runs in Isaac Sim with continuous motion, odometry, camera viewpoints, Nav2 planning, and recovery behavior. The system therefore has failure modes that discrete nav-graph VLN hides:

- Language grounding can map the instruction to the wrong semantic class.
- Visual confirmation can fail even after physical arrival if the final camera pose is poor.
- Nav2 can fail or get stuck despite the semantic target being correct.
- The robot can physically arrive while strict visual confirmation still fails.

## Metric Implications

For poster/paper reporting, use separated metrics rather than a single success number:

| Metric | Why it matters |
|---|---|
| Navigation Error / final error | Captures continuous final-position accuracy. |
| Success Rate / physical arrival | Measures whether the robot reached the target radius. |
| Visual confirmation rate | Measures whether the embodied camera can verify the target. |
| Strict task success | Requires both arrival and visual confirmation. |
| Path length / SPL-style efficiency | Needed for fair comparison if repeated trajectories are recorded. |
| Failure taxonomy | Required to distinguish perception, grounding, planning, and execution errors. |

## Design Implication

The Node 7 clean online rerun showed 13/15 physical arrivals but only 8/15 strict visual task successes. This is a classic continuous-navigation issue: reaching a coordinate is not the same as ending with an informative camera view.

The next optimization should therefore not inflate the metric by accepting all arrived trials as successful. Instead, plant/chair semantic goals should navigate to an observation pose that is free in the occupancy map and oriented toward the target object. This keeps the evaluation honest while addressing the actual failure mechanism.

## Current Implementation Response

The converter now maps plant/chair instructions to safe observation poses:

| Semantic target | Object center | Navigation observation pose |
|---|---|---|
| plant | `(-0.43, -2.92, 0.00)` | `(0.35, -2.92, 3.14)` |
| chair | `(-0.54, -0.69, 1.57)` | `(0.20, -0.69, 3.14)` |
| shelf/package area | `(-6.78, 10.96, 0.00)` | unchanged |

The observation-pose change should be evaluated in a new online rerun before claiming improved task success.
