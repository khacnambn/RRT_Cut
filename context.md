# Context: Danh gia RRT-Cut va path evaluator

## Van de ban dau

`src/core/path_evaluator` co hai metric:

- `SmoothnessMetric`: trung binh discrete curvature `|d_theta / ds|`.
- `TrackingErrorMetric`: RMSE khoang cach tu moi diem trajectory den segment gan nhat cua reference path.

Ket qua co the trai voi truc quan: RRT-Cut co path nhin thang va muot, trong khi mot so planner khac re/doi huong nhieu nhung lai co metric tot hon.

## Nguyen nhan da xac dinh

1. Curvature cu phu thuoc vao mat do waypoint, dac biet voi corner. Trung binh theo so diem co the lam loang cac lan re, va `d_theta / ds` se tang manh khi segment ngan.
2. Tracking RMSE cu chi do khoang cach gan nhat den tap segment; khong phat quay dau, di nguoc huong, loop, hay tien do sai thu tu.
3. `path_trace_node` truoc day chi giu plan dau tien, nhung robot co the chay theo cac plan replan sau do. So sanh trajectory ca hanh trinh voi plan dau la khong cong bang.
4. RRT-Cut co the tra path den `jump_node`/subgoal. Wrapper cu tung append final goal truc tiep vao path, tao segment khong duoc planner/collision check sinh ra. Duong nay co the nhin dep nhung khong phai route hop le robot theo.
5. Curvature cua trajectory odometry raw co the bung no do cac dich chuyen rat nho va nhieu odometry noise.

## Nguyen tac danh gia cong bang

Evaluator khong duoc doc hay re nhanh theo `planner_name`.

- Moi `nav_msgs/Path` publish tao mot `plan_episode`.
- Odom sau do duoc gan cho plan active moi nhat.
- Tat ca plan/trajectory duoc chuyen cung frame va resample theo cung khoang cach 0.05 m truoc khi do curvature.
- Khong append goal hay noi suy rieng cho bat ky planner nao.
- Bao cao hai tang:
  - Tung episode: length, detour ratio, turn density, curvature, tracking error, progress/backtrack.
  - Toan hanh trinh: success, time, executed length, replan/update count, active-plan tracking.

## Thay doi da trien khai

### `src/custom_node/path_trace_node/src/path_trace_node.cpp`

- Them `PlanEpisode` gom raw plan, normalized/resampled plan, va trajectory khi plan do active.
- `planCallback` luu moi plan, thay vi chi luu plan dau.
- `odomCallback` gan diem odometry vao active episode.
- Them resampling arc-length 0.05 m, `pathLength`, `turnDensity`, va projection theo arc-length de tinh backtrack.
- YAML output moi gom:
  - `evaluation_spacing_m`
  - `plan_updates`, `plan_evaluated`
  - `plan_mean_detour_ratio`
  - `plan_mean_turn_density_rad_per_m`
  - `plan_mean_curvature_1_per_m`, `plan_max_curvature_1_per_m`
  - `active_tracking_rmse_m`, `active_tracking_mean_error_m`, `active_tracking_max_error_m`
  - `active_tracking_backtrack_m`
  - `plan_episodes`: chi tiet moi plan update.
- Reset `total_points` va `previous_seq` moi khi nhan goal moi.

### `src/core/global_planner/sample_planner/src/sample_planner.cpp`

- O hai nhanh RRT-Cut, da bo viec append final goal vao sau path den subgoal.
- Endpoint publish bay gio la endpoint thuc te planner tra ve. Neu la subgoal, he thong can replan de tiep tuc den final goal.

## Cach chay

Build trong Docker/workspace ROS Noetic:

```bash
cd /root/ros_motion_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Sau khi launch simulation/navigation, chay evaluator o terminal khac.

Sample planners (RRT-Cut, RRT, RRT*, Informed RRT...):

```bash
rosrun path_trace_node path_trace_node \
  _plan_topic:=/move_base/SamplePlanner/plan \
  _output_file:=/tmp/rrt_cut_run_01.yaml
```

Graph planners (A*, JPS, Theta*...):

```bash
rosrun path_trace_node path_trace_node \
  _plan_topic:=/move_base/GraphPlanner/plan \
  _output_file:=/tmp/a_star_run_01.yaml
```

Khi benchmark, giu nguyen map, start/final-goal, local planner, costmap, timeout va so trial. Chay nhieu lan (RRT la ngau nhien), sau do bao cao median, IQR/do lech chuan va success rate.

## Luu y xac nhan

Moi thay doi da duoc kiem tra `git diff --check`. Moi truong Codex hien tai khong cai ROS Noetic, nen chua the chay `catkin_make`; can build va test runtime trong Docker cua project.
