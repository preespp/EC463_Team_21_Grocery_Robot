# Robot Autonomy Frontier P5 Acceptance Checklist

Date: 2026-03-05
Scope: End-to-end acceptance for `frontier-mission` through P5 runtime flow, including semantic auto-alignment.
Note: Automatic regression monitor is intentionally out of scope.

## Test Inputs (Fill Before Run)

- `MAP_OUTPUT_DIR`: `<absolute_output_dir>`
- `MAP_NAME_PREFIX`: `<run_prefix>`
- `SHELVES_FILE`: `<absolute_shelves_yaml>`
- `REFERENCE_MAP_YAML`: `<absolute_reference_map_yaml>`

## Acceptance Checklist

| ID | Action / Command | Expected Output (line-level) | Acceptance Criteria | Result |
|---|---|---|---|---|
| AC-01 | `colcon build --packages-select robot_interfaces robot_navigation` | Lines include `Finished <<< robot_interfaces`, `Finished <<< robot_navigation`, and summary with 2 finished packages. | Command exits with code `0`; both packages build successfully. | `[ ] Pass / [ ] Fail` |
| AC-02 | `source install/setup.bash` then `ros2 pkg executables robot_navigation` | Output contains `frontier_explorer`, `mission_orchestrator`, `semantic_overlay`, `nav_assistant`. | All required executables are listed. | `[ ] Pass / [ ] Fail` |
| AC-03 | `ros2 run robot_navigation nav_assistant frontier-mission --explore-profile slow --dry-run` | Printed launch command contains `nav2_params_file:=.../nav2_params_explore_slow.yaml`. | Slow profile resolves to the slow Nav2 params file. | `[ ] Pass / [ ] Fail` |
| AC-04 | `ros2 run robot_navigation nav_assistant frontier-mission --explore-profile task --dry-run` | Printed launch command contains `nav2_params_file:=.../nav2_params_task_run.yaml`. | Task profile resolves to the task Nav2 params file. | `[ ] Pass / [ ] Fail` |
| AC-05 | Start mission: `ros2 run robot_navigation nav_assistant frontier-mission --with-semantic-overlay true --shelves-file <SHELVES_FILE> --semantic-reference-map-yaml <REFERENCE_MAP_YAML> --semantic-auto-align-on-start true --map-output-dir <MAP_OUTPUT_DIR> --map-name-prefix <MAP_NAME_PREFIX> --explore-profile slow` | Logs include `mission_orchestrator ready (autostart=True, mapping_mode=frontier`, `frontier_explorer ready`, `semantic_overlay ready`. | All three nodes are up without immediate error. | `[ ] Pass / [ ] Fail` |
| AC-06 | Observe `mission_orchestrator` logs | Lines include `State -> BOOT` then `State -> AUTO_EXPLORE` (in this order). | State transition reaches `AUTO_EXPLORE` from `BOOT`. | `[ ] Pass / [ ] Fail` |
| AC-07 | `ros2 topic echo /frontier_explorer/state --once` (repeat if needed) | Output line `data: RUNNING` during exploration. | Frontier FSM enters running state. | `[ ] Pass / [ ] Fail` |
| AC-08 | `ros2 topic echo /frontier_explorer/current_goal` (observe for ~20s) | At least one `PoseStamped` message is published with map-frame goal coordinates. | Explorer generates autonomous goals continuously. | `[ ] Pass / [ ] Fail` |
| AC-09 | Observe `semantic_overlay` logs at startup | Line includes `Loaded reference map pattern: <N> occupied samples`. | `<N> > 0`; reference pattern loaded successfully. | `[ ] Pass / [ ] Fail` |
| AC-10 | Observe `semantic_overlay` logs after map is available | Line includes `auto_align success: score=...`. | Auto-align succeeds and score is not below configured minimum (`auto_align_min_score`, default `0.45`). | `[ ] Pass / [ ] Fail` |
| AC-11 | `ros2 service call /semantic_overlay/auto_align std_srvs/srv/Trigger "{}"` | Response includes `success: true` and message contains `auto_align success`. | Manual re-align succeeds on demand. | `[ ] Pass / [ ] Fail` |
| AC-12 | `ros2 service call /semantic_overlay/query_shelf_pose robot_interfaces/srv/QueryShelfPose "{shelf_id: 'shelf_A1'}"` | Response includes `found: true` and `pose.header.frame_id: map`. | Semantic layer provides queryable shelf poses in map frame. | `[ ] Pass / [ ] Fail` |
| AC-13 | `ros2 service call /semantic_overlay/reload std_srvs/srv/Trigger "{}"` | Response includes `success: true` and `loaded <count> shelves, reference_points=<count>`. | Reload path succeeds with no service error. | `[ ] Pass / [ ] Fail` |
| AC-14 | Visual check in RViz after shelf list change + reload | Old labels/markers are removed; only current shelf list remains. | No stale semantic markers remain after reload. | `[ ] Pass / [ ] Fail` |
| AC-15 | `ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: true}"` | Logs include `State -> PAUSED` in mission and frontier (and active goal cancellation if one exists). | Manual override pauses autonomy and robot motion promptly. | `[ ] Pass / [ ] Fail` |
| AC-16 | `ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: false}"` | Logs return to active exploration (`State -> AUTO_EXPLORE` and frontier `State -> RUNNING`). | Resume works without deadlock/restart. | `[ ] Pass / [ ] Fail` |
| AC-17 | Observe transition out of exploration | Logs include one of: `AUTO_EXPLORE finished by frontier explorer done signal`, `AUTO_EXPLORE reached explore_timeout_sec`, or `AUTO_EXPLORE completed by finish_mapping service`, followed by `State -> RETURN_HOME`. | Mission leaves exploration and enters return-home stage. | `[ ] Pass / [ ] Fail` |
| AC-18 | Observe return-home stage logs | Line includes `RETURN_HOME succeeded`. | Return-home action succeeds without entering `ERROR`. | `[ ] Pass / [ ] Fail` |
| AC-19 | Observe save/export stage logs | Lines include `SAVE_EXPORT pbstream saved: ...` and `SAVE_EXPORT map export finished: ...`. | Both pbstream save and map export complete successfully. | `[ ] Pass / [ ] Fail` |
| AC-20 | Verify output files in `<MAP_OUTPUT_DIR>` | Files exist: `<map_name>.pbstream`, `<map_name>.yaml`, `<map_name>.pgm` (non-zero size). | All expected artifacts are produced and usable for localization. | `[ ] Pass / [ ] Fail` |
| AC-21 | Observe final mission state | Logs include `Mission reached LOCALIZE_READY` and `State -> LOCALIZE_READY`. | End-to-end mission completes to `LOCALIZE_READY`. | `[ ] Pass / [ ] Fail` |
| AC-22 | Entire run error scan | No line `State -> ERROR` in mission logs. | Run is accepted only if no mission error state occurs. | `[ ] Pass / [ ] Fail` |

## Overall Pass Rule

- Critical items must pass: `AC-01`, `AC-05`, `AC-06`, `AC-10`, `AC-17`, `AC-18`, `AC-19`, `AC-21`, `AC-22`.
- Full acceptance requires all checklist items to pass.
