# Goal Tolerance Implementation Plan

> **For agentic workers:** Execute this single configuration task inline and do
> not create a Git commit.

**Goal:** Set consistent 0.05 m position and 0.035 rad yaw arrival tolerances.

**Architecture:** The three production Dora flow templates inject identical
values into both MPPI and the navigation supervisor. No compiled planner logic
changes.

**Tech Stack:** Dora YAML configuration, shell validation, existing project
build scripts.

## Global Constraints

- Do not start the robot or command chassis motion.
- Do not change the 0.04 m obstacle safety margin.
- Do not create a Git commit.

---

### Task 1: Synchronize production arrival tolerances

**Files:**
- Modify: `apps/adora_nano_navigation/adora_nano_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_localization_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_slam_navigation.yml`

**Interfaces:**
- Consumes: `GOAL_TOLERANCE` and `YAW_GOAL_TOLERANCE` environment variables.
- Produces: identical arrival criteria for MPPI and navigation supervisor.

- [ ] Change every production `GOAL_TOLERANCE` from `0.08` to `0.05`.
- [ ] Change every production `YAW_GOAL_TOLERANCE` from `0.12` to `0.035`.
- [ ] Verify exactly six occurrences of each key and identical values.
- [ ] Generate the position1 navigation configuration in dry-run mode and
      confirm the generated MPPI and supervisor environments.
- [ ] Run `./scripts/build_all.sh` and require exit status 0.
- [ ] Confirm no Dora flow or navigation process was started.
