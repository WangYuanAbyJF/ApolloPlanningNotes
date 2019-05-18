# Pipeline of TrajectoryStitcher::ComputeStitchingTrajectory

```cpp
std::vector<TrajectoryPoint>TrajectoryStitcher::ComputeStitchingTrajectory(
const VehicleState& vehicle_state, 
const double current_timestamp, 
const double planning_cycle_time, 
const PublishableTrajectory* prev_trajectory, 
std::string* replan_reason)
```
|
|
|
check if stitch is disabled by gflag：
```cpp
if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
```
|
|
|
check if previous trajectory exists:
```cpp
if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
```
|
|
|
check if current mode is autonomous driving:
```cpp
if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
```
|
|
|
check if size of previous trajectory > 0,if not reinitialize stiching trajectory with vehicle state:
```cpp
if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  
std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_s(0.0);
  init_point.mutable_path_point()->set_x(vehicle_state.x());
  init_point.mutable_path_point()->set_y(vehicle_state.y());
  init_point.mutable_path_point()->set_z(vehicle_state.z());
  init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);

  return std::vector<TrajectoryPoint>(1, init_point);
}
```
|
|
|
....................
annoying math algorithm



