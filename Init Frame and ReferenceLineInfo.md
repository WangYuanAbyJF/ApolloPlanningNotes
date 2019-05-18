# Init Frame and ReferenceLineInfo
-------------------------------------------
* #### in `StdPlanning::RunOnce()` dispatch `StdPlanning::InitFrame()`, use stitching_trajector and vehicle_state as Input, meanwhile in class `StdPlanning` there are `frame_` and `local_view_` as variable, for data storage:
```cpp
status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,
                   vehicle_state, trajectory_pb)
```

* #### in `StdPlanning::InitFrame` there is `reference_line_provider_` variable, dispatch `ReferenceLineProvider::GetReferenceLines()` function to generate `reference_lines` and `segments` variables of `StdPlanning` class:
```
reference_line_provider_->GetReferenceLines(&reference_lines,&segments)
```

* #### in `StdPlanning::InitFrame()` dispatch `Frame::Init()` to initialize `frame_` variable of `StdPlanning` class, takes `reference_lines` and `segements` variable as input: 
```cpp
frame_->Init(reference_lines, segments, reference_line_provider_->FutureRouteWaypoints()
```
* #### in `Frame::Init()` dispatch `Frame::InitFrameData()` to initialize data in frame:
```cpp
auto status = InitFrameData()
```
* #### in `Frame::InitFrameData()` add obstacle informations to `Frame::obstacles_` from `Frame::local_view_`:
```cpp
for (auto &ptr :
     Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
  AddObstacle(*ptr);
}

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}
```
* #### in `Frame::Init()` dispatch `Frame::CreateReferenceLineInfo()` to initialize `reference_line_info_` variable in `Frame` class, takes `reference_lines` and `segments` as input:
```cpp
if (!CreateReferenceLineInfo(reference_lines, segments)) {
  const std::string msg = "Failed to init reference line info.";
  AERROR << msg;
  return Status(ErrorCode::PLANNING_ERROR, msg);
}
```
* #### in `Frame::CreateReferenceLineInfo()` use `reference_lines` and `segments` to initialize `reference_line_info_` variable of `Frame` class, and dispatch `ReferenceLineInfo::Init()` to add obstacle information to `reference_line_info_`, take obstacles' information as input:
```cpp
bool Frame::CreateReferenceLineInfo(
  const std::list<ReferenceLine> &reference_lines,
  const std::list<hdmap::RouteSegments> &segments) {
  ...
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      ...
      ...
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }
  ...
  ...
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles())) {
    ...
    ...
    }
  }
  return has_valid_reference_line;
}
```
* #### `ReferenceLineInfo::Init()` initialized sl_boundries, dispatch `ReferenceLineInfo::AddSpeedLimit()` to add on speed limit informations, and dispatch `ReferenceLineInfo::AddObstacles()` to add on obstacle informations:
```cpp
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

