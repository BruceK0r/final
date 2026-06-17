# ver1 工程化纯跟踪改进说明

## 修改函数

- `Control.__init__()`：新增动态预瞄、转角限幅、转角变化率限制、弯道降速和闭环路径状态参数。
- `Control.run_step()`：保留原有控制流程调用方式，将单步控制逻辑从 `control_node()` 中拆出，接口仍发送 `(v, w)` UDP 控制命令。
- `Control.load_route()`：读取路径后判断是否为闭环路径，并初始化路径索引。
- `Control.calc_pure_pursuit()`：补全 Pure Pursuit 公式，并加入目标点过近、目标点在车后、转角限幅、转角变化率限制和弯道降速保护。
- `Control.search_target_pos()`：替换原来的车头投影点全局最近搜索，改为从车辆当前位置路径索引开始按路径弧长向前累计预瞄距离。
- `Control.update_vehpos_index()`、`Control.search_vehicle_initial_index()`、`Control.find_nearest_point_index()`：增加空路径、闭环路径和索引边界保护。
- 新增 `normalize_angle()`、`clamp()`、`calc_lookahead_distance()`、`estimate_path_curvature()`、`limit_steering_angle()` 等辅助函数。

## 原目标点搜索的问题

原方式先沿车辆朝向投影一个点，再在全局路径点中找最近点。车辆横摆、航向角有误差、路径存在回环或交叉时，投影点可能落到非当前行驶方向的近距离路径段上，目标点会在不同路径段之间跳变。目标点跳变会导致航向误差和曲率突然变化，表现为直道左右摇摆、弯道内切和角速度命令突变。

## 动态预瞄距离

新版本使用 `calc_lookahead_distance()` 根据速度和前方路径曲率调整预瞄距离。速度越高，预瞄距离越大，直道上控制更平顺，不会频繁追逐近处路径点。曲率越大，预瞄距离越小，弯道中目标点更贴近前方路径，减少大预瞄导致的切弯和内切。预瞄距离同时受 `min_lookahead` 和 `max_lookahead` 限制，避免过小或过大。

## 控制输出限制

- 转角限幅：限制最大前轮转角，避免目标点异常或曲率突变时输出不现实的大转向。
- 转角变化率限制：根据 `control_rate` 限制每一帧转角变化量，使角速度命令连续，减少左右摇摆和控制抖动。
- 弯道降速：根据实际转角换算曲率，并用最大横向加速度限制速度上限，降低高速入弯导致的内切和震荡风险。
