# nav_supervisor_dora_node

Navigation state supervisor for the Dora navigation prototype.

Inputs:
- `CorrectedPose`: current estimated robot pose.
- `LocalPlannerStatus`: local planner mode and obstacle status.
- `tick`: periodic supervision tick.

Output:
- `NavigationStatus`: current navigation state.

States:
- `RUNNING`
- `REACHED`
- `BLOCKED`
- `TIMEOUT`
- `POSE_TIMEOUT`

This node does not command the chassis yet. It is a supervision/status layer for
debugging, demonstrations, and future replanning logic.
