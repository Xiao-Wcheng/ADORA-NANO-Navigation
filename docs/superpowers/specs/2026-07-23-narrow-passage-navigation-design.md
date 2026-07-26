# Narrow-Passage Navigation Design

## Goal

Allow the robot to plan farther ahead around a newly placed obstacle while
stopping safely when valid MPPI commands do not produce meaningful progress.

## Design

The three production navigation templates use a 0.21 m dynamic-obstacle
inflation radius and a 40-step MPPI horizon. At the existing 0.1 s control
period and 0.045 m/s maximum translation speed, this increases the theoretical
look-ahead from 0.09 m to 0.18 m without changing the robot footprint or 0.04 m
safety margin.

The navigation supervisor owns an independent goal-progress watchdog. It stores
the best distance reached for the active goal. A reduction of at least 0.02 m
resets the watchdog. Ten seconds without that reduction produces
`BLOCKED/stalled_no_progress`, sends a zero command before any local command can
be forwarded, and participates in the existing bounded replan mechanism.
Receiving a changed goal resets the watchdog.

## Safety and Verification

The robot radius remains 0.17 m and the safety margin remains 0.04 m. Unit tests
cover accumulated small progress, genuine progress, timeout, and reset. Full
build, tests, generated-flow inspection, and a no-running-flow audit are
required. No physical navigation starts during implementation.
