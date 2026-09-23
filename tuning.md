# Tuning tránh vật cản động

Tài liệu này ghi lại các vị trí có thể điều chỉnh khi chạy `rrt_cut` trong môi trường có pedestrian động.

## 1. Tốc độ và gia tốc của xe

File chính:

`src/sim_env/config/planner/dwa_planner_params.yaml`

```yaml
DWAPlanner:
  max_vel_x: 0.46       # vận tốc tiến tối đa theo trục x
  max_vel_trans: 0.26  # giới hạn vận tốc tịnh tiến thực tế
  acc_lim_x: 2.5       # gia tốc tiến tối đa
  max_vel_theta: 1.82  # vận tốc quay tối đa
  acc_lim_theta: 3.2   # gia tốc quay tối đa
```

`max_vel_trans` và `max_vel_x` đều giới hạn vận tốc. Vận tốc tiến thực tế không vượt quá giá trị nhỏ hơn trong hai tham số này.

Giá trị thử nghiệm thận trọng:

```yaml
max_vel_x: 0.60
max_vel_trans: 0.35
acc_lim_x: 3.0
```

Tăng từng bước, đo khoảng cách phanh và kiểm tra va chạm trước khi tăng tiếp.

## 2. Giới hạn động lực trong Gazebo

File:

`src/sim_env/urdf/turtlebot3_waffle/turtlebot3_waffle.gazebo.xacro`

```xml
<wheelAcceleration>1</wheelAcceleration>
<wheelTorque>10</wheelTorque>
```

Nếu đã tăng DWA nhưng xe vẫn không đạt vận tốc, có thể thử:

```xml
<wheelAcceleration>2</wheelAcceleration>
<wheelTorque>15</wheelTorque>
```

Không tăng các giá trị này cùng lúc với vận tốc quá lớn; lực kéo và thời gian phanh cần được kiểm tra lại.

## 3. Tần số lidar

File:

`src/sim_env/urdf/turtlebot3_waffle/turtlebot3_waffle.gazebo.xacro`

```xml
<sensor type="ray" name="lds_lfcd_sensor">
  <update_rate>10</update_rate>
</sensor>
```

`/scan` phải có tần số thực tế tương ứng. Kiểm tra trong container:

```bash
rostopic hz /scan
```

Tần số cảm biến thấp sẽ giới hạn khả năng phát hiện pedestrian, dù costmap được đặt tần số cao hơn.

## 4. Global costmap

File:

`src/sim_env/config/costmap/global_costmap_params.yaml`

```yaml
global_costmap:
  update_frequency: 1.0
  publish_frequency: 1.0
```

Giá trị thử nghiệm:

```yaml
update_frequency: 3.0
publish_frequency: 3.0
```

Có thể thử `5.0` nếu CPU vẫn đủ. `update_frequency` cập nhật costmap; `publish_frequency` chỉ điều khiển tần số xuất bản dữ liệu cho RViz và node khác.

Các tham số cảm biến liên quan cũng nằm trong file này:

```yaml
obstacle_range: 2.5
raytrace_range: 3.0
expected_update_rate: 1.5
observation_persistence: 0.0
```

- Tăng `obstacle_range` nếu cần phát hiện xa hơn, nhưng không vượt quá tầm lidar.
- `raytrace_range` dùng để xóa vùng đã quét là không còn vật cản.
- `observation_persistence: 0.0` chỉ giữ observation mới nhất, phù hợp với vật cản động.
- `expected_update_rate` nên phù hợp với tần số `/scan`; với lidar 10 Hz có thể đặt khoảng `0.1` hoặc `0.2` giây nếu muốn cảnh báo dữ liệu cũ chặt hơn.

## 5. Local costmap

File:

`src/sim_env/config/costmap/local_costmap_params.yaml`

```yaml
local_costmap:
  update_frequency: 10.0
  publish_frequency: 10.0
  width: 3
  height: 3
  resolution: 0.05
```

Local costmap 10 Hz hiện đã cao hơn global costmap. Nếu tăng tốc độ xe, có thể tăng vùng nhìn theo hướng chuyển động bằng cách tăng `width` hoặc `height`, nhưng chi phí tính toán cũng tăng.

## 6. Tần số global replanning và điều khiển

File:

`src/sim_env/config/move_base_params.yaml`

```yaml
planner_frequency: 2.0
controller_frequency: 10.0
planner_patience: 5.0
controller_patience: 15.0
```

Giá trị thử nghiệm:

```yaml
planner_frequency: 3.0
controller_frequency: 10.0
```

Có thể thử `planner_frequency: 5.0` chỉ khi một lần chạy `rrt_cut` hoàn thành nhanh hơn `0.2` giây. Nếu planner chạy lâu hơn chu kỳ yêu cầu, tăng tần số sẽ không tạo ra replanning thật sự và có thể làm CPU quá tải.

`controller_frequency` nên giữ ít nhất 10 Hz khi xe chạy nhanh hơn để lệnh vận tốc được cập nhật đều.

## 7. Tham số RRT-Cut

File:

`src/sim_env/config/planner/sample_planner_params.yaml`

```yaml
SamplePlanner:
  sample_points: 10000
  sample_max_d: 5.0
  number_node_k: 5
  obstacle_factor: 0.5
  outline_map: true
```

- `sample_points` càng lớn thì khả năng tìm đường tăng nhưng thời gian lập plan cũng tăng. Khi cần replanning nhanh, thử giảm dần xuống `5000` hoặc `3000`.
- `sample_max_d` ảnh hưởng bước mở rộng của cây; giảm quá thấp có thể làm planner chậm, tăng quá cao có thể làm đường kém linh hoạt.
- `obstacle_factor` được dùng khi RRT-Cut kiểm tra chi phí ô costmap.
- `outline_map: true` giúp giữ cây trong biên costmap.

## 8. Tốc độ pedestrian

File:

`src/user_config/human_config.yaml`

```yaml
pedestrians:
  update_rate: 10
  ped_property:
    - name: human_1
      velocity: 0.6
```

Giảm `velocity` để tạo kịch bản kiểm thử dễ hơn, ví dụ:

```yaml
velocity: 0.2
```

`src/user_config/user_config.yaml` phải bật plugin:

```yaml
plugins:
  pedestrians: "human_config.yaml"
```

Chạy lại `main.sh` sau khi sửa để generator tạo lại world có pedestrian.

## 9. Thứ tự tuning khuyến nghị

1. Kiểm tra `/scan` có nhìn thấy pedestrian: `rostopic hz /scan` và RViz.
2. Đặt lidar 10 Hz, local costmap 10 Hz và global costmap 3 Hz.
3. Đặt `planner_frequency` ở 3 Hz và đo thời gian một lần lập plan.
4. Tăng `max_vel_trans` từ `0.26` lên `0.35` m/s.
5. Nếu plan chậm, giảm `sample_points` trước khi tăng `planner_frequency` tiếp.
6. Chỉ tăng `wheelAcceleration` và `wheelTorque` nếu Gazebo không đạt vận tốc DWA yêu cầu.

## 10. Điều kiện an toàn cần kiểm tra

RRT-Cut lập đường dựa trên costmap tại thời điểm lập kế hoạch. Costmap phải nhận được observation mới trước khi planner chạy lại.

Trong nhánh escape, `sample_planner.cpp` hiện thêm goal cuối vào plan sau khi lập đường tới sub-goal. Đoạn nối này cần được kiểm tra collision nếu muốn bảo đảm toàn bộ plan tránh được pedestrian.

