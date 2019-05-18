#  RunOnce Cycle
------------------
RunOnce input: 
`&LocalView, *ADCTrajectory(protobuf message)`
|
|
|
update vehicleState:
`VehicleStateProvider::Update(*local_view_.localization_estimate, *local_view_.chassis)`
input: LocalizationEstimate(protobuf message), Chassis(protobuf message)
`vehicle_state_.set_timestamp` #with timestamp from chasis if timestamp from localization is invalid
`vehicle_state_.set_gear` # set GEAR_NONE if chassis channel has no gear_location
`vehicle_state_.set_linear_velocity` # -vehicle_state_.linear_velocity() if GEAR_REVERSE
|
|
|
estimate (x, y) only if the current time and vehicle state timestamp differs only a small amount (20ms):
`future_xy =  VehicleStateProvider::estimateFuturePosition(time_gap)` 
`vehicle_state.set_x(future_xy.x());`
`vehicle_state.set_y(future_xy.y());`
|
|
|
check if status is ok or vehicle state is valid. if not, set 'not_ready', save status, fill trajectory with initialized teajectory, finish the cycle:
`    not_ready->set_reason(msg);`
`status.Save(trajectory_pb->mutable_header()->mutable_status());`
`FillPlanningPb(start_timestamp, trajectory_pb);`
`GenerateStopTrajectory(trajectory_pb);`
`return;`
|
|
|
check if routing has changed, if so, update RoutingResponse:
```cpp
  if (IsDifferentRouting(last_routing_, *local_view_.routing)) {
    last_routing_ = *local_view_.routing;
    PlanningContext::MutablePlanningStatus()->Clear();
    reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
  }
```
|
|
|
Update reference line provider and reset pull over if necessary, with reference_line_provider whose trajectory information has already updated:
`reference_line_provider_->UpdateVehicleState(vehicle_state);`
|
|
|
compute stitching_trajectory
`stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(vehicle_state,start_timestamp,planning_cycle_time,last_publishable_trajectory_.get(), &replan_reason)`
调用堆栈见 TrajectoryStitcher::ComputeStitchingTrajectory
|
|
|
update ego_info with stiching trajectory and vehicle_state:
```cpp
bool update_ego_info =
      EgoInfo::Instance()->Update(stitching_trajectory.back(), vehicle_state);
```
|
|
|

initialize frame:
```cpp
InitFrame(frame_num, 
  stitching_trajectory.back(), 
  start_timestamp,
  vehicle_state, trajectory_pb)
```````
调用堆栈见 StdPlanning::InitFrame
|
|
|
calculate obs-free distance at front:
```cpp
EgoInfo::Instance()->CalculateFrontObstacleClearDistance(  
                     frame_->obstacles())  //algorithm inside, results at EgoInfo::front_clear_distance_
```
|
|
|
record time:
```cpp
trajectory_pb->
mutable_latency_stats()->
set_init_frame_time_ms(Clock::NowInSeconds() - start_timestamp)
```
|
|
|
apply traffic rules on each reference_line_info:
```cpp
for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
  TrafficDecider traffic_decider;
  traffic_decider.Init(traffic_rule_configs_);
  auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
  if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
    ref_line_info.SetDrivable(false);
    AWARN << "Reference line " << ref_line_info.Lanes().Id()
          << " traffic decider failed";
    continue;
  }
}
```
调用堆栈见 “Traffic Decider Pipeline”
|
|
|
apply plan:
```cpp
status = Plan(start_timestamp, stitching_trajectory, trajectory_pb);
```
调用堆栈见StdPlanning::Plan()



