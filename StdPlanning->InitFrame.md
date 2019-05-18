# StdPlanning::InitFrame

---------------
initialize Frame class with multiple information
-

input:
current cycle_index, planning start point, current timestamp current vehicle state, ADCtrajectory (as protobuf message):
```cpp
Status StdPlanning::InitFrame(const uint32_t sequence_num,
                              const TrajectoryPoint& planning_start_point,
                              const double start_time,
                              const VehicleState& vehicle_state,
                              ADCTrajectory* output_trajectory)
```
|
|
|
update `reference_lines` and `segments` with `ReferenceLineProvider::reference_lines_` and `ReferenceLineProvider::route_segments_` or with `ReferenceLineProvider::reference_line_history_` and `ReferenceLineProvider::route_segments_history_` if `ReferenceLineProvider::reference_lines_` and `ReferenceLineProvider::route_segments_` is empty:
```cpp
std::list<ReferenceLine> reference_lines;
std::list<hdmap::RouteSegments> segments;
if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                 &segments)) {
  std::string msg = "Failed to create reference line";
  return Status(ErrorCode::PLANNING_ERROR, msg);
}
```

```cpp
if (!reference_lines_.empty()) {
  reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
  segments->assign(route_segments_.begin(), route_segments_.end());
  return true;
} else {
  AWARN << "Reference line is NOT ready.";
  if (reference_line_history_.empty()) {
    return false;
  }
  reference_lines->assign(reference_line_history_.back().begin(),
                          reference_line_history_.back().end());
  segments->assign(route_segments_history_.back().begin(),
                   route_segments_history_.back().end());
}
```
|
|
|
decide to use  `FLAGS_look_forward_long_distance` or `FLAGS_look_forward_short_distance` as forward calculate limit by `velocity * FLAGS_look_forward_time_sec`
```cpp
auto forword_limit =
    hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());
```
```cpp
double PncMap::LookForwardDistance(double velocity) {
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  if (forward_distance > FLAGS_look_forward_short_distance) {
    return FLAGS_look_forward_long_distance;
  }

  return FLAGS_look_forward_short_distance;
}
```
|
|
|
create project window by erasing points from reference lines according to forward limit:
```cpp
ref_line.Shrink(Vec2d(vehicle_state.x(), 
                vehicle_state.y()),
                FLAGS_look_backward_distance, forword_limit)

seg.Shrink(Vec2d(vehicle_state.x(),
                 vehicle_state.y()),
                 FLAGS_look_backward_distance, forword_limit)
```
|
|
|

initialize Frame in `Frame::Init()` with projected reference_lines and segments,align time to prediction, check collision; `reference_lines` and `segments` are used for generating `reference_line_info` in `Frame::CreateReferenceLineInfo()`:
```cpp
frame_->Init(reference_lines, 
             segments,
             reference_line_provider_->FutureRouteWaypoints())
```
```cpp
Status Frame::Init(
  const std::list<ReferenceLine> &reference_lines,
  const std::list<hdmap::RouteSegments> &segments,
  const std::vector<routing::LaneWaypoint> &future_route_waypoints) {
  auto status = InitFrameData();
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  if (!CreateReferenceLineInfo(reference_lines, segments)) {
    const std::string msg = "Failed to init reference line info.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  future_route_waypoints_ = future_route_waypoints;

  return Status::OK();
}
```
```cpp
// in Frame::InitFrameData():

...
  auto prediction = *(local_view_.prediction_obstacles);
  AlignPredictionTime(vehicle_state_.timestamp(), &prediction);
  local_view_.prediction_obstacles->CopyFrom(prediction);
  
...
for (auto &ptr : 
     Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
       AddObstacle(*ptr);
  }
...
const auto *collision_obstacle = FindCollisionObstacle();
if (collision_obstacle != nullptr) {
...
  return Status(ErrorCode::PLANNING_ERROR, err_str);
...
}
return Status::OK();
```
```cpp
// in Frame::CreateReferenceLineInfo():

...
auto ref_line_iter = reference_lines.begin();
auto segments_iter = segments.begin();
while (ref_line_iter != reference_lines.end()) {
  ......
  reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                    *ref_line_iter, *segments_iter);
  ++ref_line_iter;
  ++segments_iter;
}
...

bool has_valid_reference_line = false;
for (auto &ref_info : reference_line_info_) {
  if (!ref_info.Init(obstacles())) {
    AERROR << "Failed to init reference line";
    continue;
  } else {
    has_valid_reference_line = true;
  }
}
return has_valid_reference_line;
```
```cpp
// in ReferenceLineInfo::Init
bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  ...
  ...
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center((param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
                      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), param.length(), param.width());
  ...
  ...
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());
  Vec2d vehicle_center(vehicle_position +
                       vec_to_center.rotate(vehicle_state_.heading()));
  Box2d vehicle_box(vehicle_center, vehicle_state_.heading(), param.length(),
                    param.width());
                    
  if (!reference_line_.GetSLBoundary(vehicle_box,
                                     &sl_boundary_info_.vehicle_sl_boundary_)) {
    ...
    ...
  }

  if (!reference_line_.GetSLBoundary(box,
                                     &sl_boundary_info_.adc_sl_boundary_)) {
    ...
    ...
  }
  ...
  ...
  const auto& map_path = reference_line_.map_path();
  for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
    // -1 and + 1.0 are added to make sure it can be sampled.
    reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0,
                                  speed_bump.end_s + 1.0,
                                  FLAGS_speed_bump_speed_limit);
  }
  // set lattice planning target speed limit;
  ...
  ...
}
```


