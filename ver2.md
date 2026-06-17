# ver2 多车安全决策层说明

## 修改文件

- `main.py`：在原纯跟踪输出之后新增多车安全决策层，集成邻车分类、TTC 风险判断、状态机、跟车、变道绕行、边界检查和调试输出。
- `exp_routes/ccw_right_bottom_loop_closed.json`：当前默认加载的闭环实验路径。

## 核心新增函数

- 邻车感知：`get_neighbor_vehicles()`、`transform_to_ego_frame()`、`classify_neighbor_vehicle()`。
- 风险判断：`compute_ttc()`、`has_collision_risk()`、`emergency_stop_if_needed()`。
- 状态机入口：`multi_vehicle_safety_layer(v_ref, w_ref)`。
- 前车/对向车处理：`handle_front_vehicle()`、`handle_oncoming_vehicle()`。
- 候选轨迹：`generate_lane_change_candidates()`、`generate_offset_path()`、`select_safe_candidate_path()`。
- 车辆预测：`predict_vehicle_position()`、`check_vehicle_collision_for_path()`。
- 地图边界：`load_map_boundaries()`、`point_in_polygon()`、`segment_intersects_segment()`、`segment_intersects_polygon()`、`distance_point_to_segment()`、`check_boundary_collision()`。
- 通用工具：`normalize_angle()`、`clamp()`、`get_time_now()`。

## 状态机工作方式

主流程保持为先纯跟踪、再安全层：

```python
v_ref, w_ref = self.calc_pure_pursuit(...)
v_cmd, w_cmd = self.multi_vehicle_safety_layer(v_ref, w_ref)
```

安全层按优先级处理：紧急停车和边界风险最高，其次是车辆碰撞风险；正在执行的变道/绕行轨迹会持续做边界和车辆预测检查；无风险时回到 `NORMAL_PATH_FOLLOW`，继续使用原纯跟踪命令。

## 前车跟车和超车

前方同向车通过自车坐标系下的 `x_rel`、`y_rel` 和航向差分类。若间距小于 `min_gap + ego_speed * time_headway` 或 TTC 小于阈值，进入 `FRONT_CAR_FOLLOW`，用安全间距误差调节速度。前车低速或停车持续超过 `blocking_time_threshold` 后，生成左右平滑偏移轨迹，只有车辆预测和边界检查都通过才进入 `TRY_LANE_CHANGE`。

## 对向来车让行和绕行

对向车风险用双方速度和距离计算 TTC。存在风险时先进入 `ONCOMING_YIELD`，按 `comfort_decel` 平滑减速到 0。停车超过 `stop_wait_time` 后进入 `TRY_BYPASS`，生成左右绕行候选轨迹；若都不安全则继续停车，不强行绕行。

## 地图边界检查

`load_map_boundaries()` 支持读取包含 `boundaries`、`boundary`、`points`、`vertices` 等常见字段的 JSON。当前默认假设边界 JSON 与路径 JSON 坐标一致，都是 x 向右、y 向上的地图坐标；如果后续边界来自图像左上角原点，可通过 `boundary_flip_y`、`boundary_image_height`、`boundary_scale` 和偏移量统一坐标。

候选轨迹会检查三类边界风险：轨迹点是否在禁入闭合区域内，轨迹线段是否穿越边界线，轨迹点到边界线段距离是否小于 `boundary_safe_margin`。

## 局限性和可调参数

- 其他车辆预测使用短时匀速直线模型，复杂机动场景需要更强预测模型。
- 候选轨迹是基于原路径法向的平滑横向偏移，不做全局规划。
- 当前仓库没有实际边界 JSON，默认启动时若找不到边界文件则边界约束为空。
- 可重点调参：`min_gap`、`time_headway`、`ttc_threshold`、`oncoming_ttc_threshold`、`lane_change_offset`、`lane_change_length`、`bypass_length`、`vehicle_safe_radius`、`boundary_safe_margin`。

## 验证

已执行：

```bash
python -m py_compile main.py
```

语法检查通过。
