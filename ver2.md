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

## 邻车检测范围

直道检测使用自车坐标系前向窄走廊：前方同向车检测前方 35m、横向半宽 2m；对向车检测前方 45m；后车检测后方 25m、横向半宽 2m；近旁车检测 `|x_rel| < 4m` 且 `|y_rel| < 6m`。对向车风险额外要求车辆落在当前路线中心线附近的同车道冲突走廊内，默认半宽为 `lane_width * 0.45 = 1.8m`。左侧相邻车道中航向差大于 150 度、距离小于 45m 的车辆归类为 `ADJACENT_LANE_ONCOMING`，正常路径跟随时忽略，不触发对向让行或紧急停车；但候选变道/绕行轨迹仍会把它作为预测障碍检查，避免左变道进入对向车道。当路径曲率大于 `0.035` 时，前方同向车和对向车仍使用前向扇形做前方范围判断，但对向车必须同时满足同车道冲突走廊条件。

## 速度和预瞄

当前最高巡航速度为 `cruise_speed = 15.0`。直道上使用更大的动态预瞄距离提升高速稳定性；弯道中通过路径曲率压缩预瞄距离，并继续使用横向加速度限制降低实际输出速度。为减少弯道入口慢半拍和左右摆动，控制器会把前方路径的带符号曲率作为转向前馈，并用路径曲率提前限制弯道速度。

## 前车跟车和超车

前方同向车通过自车坐标系下的 `x_rel`、`y_rel` 和航向差分类。若间距小于 `min_gap + ego_speed * time_headway` 或 TTC 小于阈值，进入 `FRONT_CAR_FOLLOW`，用安全间距误差调节速度。前车低速或停车持续超过 `blocking_time_threshold` 后，生成左右平滑偏移轨迹，只有车辆预测和边界检查都通过才进入 `TRY_LANE_CHANGE`。

## 对向来车让行

对向车风险用双方速度和距离计算 TTC。存在风险时进入 `ONCOMING_YIELD`，按 `comfort_decel` 平滑减速到 0 并保持停车等待；风险解除后回到 `NORMAL_PATH_FOLLOW`。已删除停车超过固定时间后进入 `TRY_BYPASS` 的逻辑，不再尝试对向车绕行重规划。

## 地图边界检查

`load_map_boundaries()` 支持读取包含 `boundaries`、`boundary`、`points`、`vertices` 等常见字段的 JSON。当前默认假设边界 JSON 与路径 JSON 坐标一致，都是 x 向右、y 向上的地图坐标；如果后续边界来自图像左上角原点，可通过 `boundary_flip_y`、`boundary_image_height`、`boundary_scale` 和偏移量统一坐标。

候选轨迹会检查三类边界风险：轨迹点是否在禁入闭合区域内，轨迹线段是否穿越边界线，轨迹点到边界线段距离是否小于 `boundary_safe_margin`。

## 局限性和可调参数

- 其他车辆预测使用短时匀速直线模型，复杂机动场景需要更强预测模型。
- 候选轨迹是基于原路径法向的平滑横向偏移，不做全局规划。
- 当前仓库没有实际边界 JSON，默认启动时若找不到边界文件则边界约束为空。
- 可重点调参：`min_gap`、`time_headway`、`ttc_threshold`、`oncoming_ttc_threshold`、`lane_change_offset`、`lane_change_length`、`vehicle_safe_radius`、`boundary_safe_margin`。

## 验证

已执行：

```bash
python -m py_compile main.py
```

语法检查通过。
