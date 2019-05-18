# StdPlanning::Plan
----------------
after the reference_line_info list has been processed by traffic_rules, use StdPlanning::Plan() to apply plan strategies.

StdPlanning::Plan() takes stiching_trajectory as input
```cpp
Status StdPlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const trajectory_pb) {
    ...
    ...
}
```
Apollo framework has mutiple different planners, all inherit from `Planner`. In StdPlanning::Plan(), Apollo choose PublicRoadPlanner to be the most frequently used planner. In StdPlanning::Plan() PublicRoadPlanner is dispatched as below, takes Frame and stitching_trajectory as input
```cpp
auto status = planner_->Plan(stitching_trajectory.back(), frame_.get())
```

PublicRoadPlanner::Plan() shows as below
```cpp
Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame) {
  DCHECK_NOTNULL(frame);
  scenario_manager_.Update(planning_start_point, *frame);
  scenario_ = scenario_manager_.mutable_scenario();
  auto result = scenario_->Process(planning_start_point, frame);

  if (FLAGS_enable_record_debug) {
    if (frame->output_trajectory()) {
      auto scenario_debug = frame->output_trajectory()
                                ->mutable_debug()
                                ->mutable_planning_data()
                                ->mutable_scenario();
      scenario_debug->set_scenario_type(scenario_->scenario_type());
      scenario_debug->set_stage_type(scenario_->GetStage());
      scenario_debug->set_msg(scenario_->GetMsg());
    }
  }

  if (result == scenario::Scenario::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, *frame);
  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
    return Status(common::PLANNING_ERROR, "scenario returned unknown");
  }
  return Status::OK();
}
```
## 1. Scenario Layer

in PublicRoadPlanner::Plan() update scenario by using Planner::scenario_manager_, ScenarioManager::Update() is dispatched, take planning_start_point and frame as input 
```cpp
scenario_manager_.Update(planning_start_point, *frame);
scenario_ = scenario_manager_.mutable_scenario();
```

in ScenarioManager::Update(),ScenarioManager::Observe() is dispatched to get information of scenario
```cpp
Observe(frame);
```

in ScenarioManager::Observe(), PlanningContext::GetScenarioInfo() is dispatched, information of scenario is taken from Frame and stored in PlanningContext::ScenarioInfo. PlanningContext::ScenarioInfo is a data struct with PathOverLap type of data.
```cpp
void ScenarioManager::Observe(const Frame& frame) {
  // init
  PlanningContext::GetScenarioInfo()->next_stop_sign_overlap = PathOverlap();
  PlanningContext::GetScenarioInfo()->next_traffic_light_overlap =
      PathOverlap();
  PlanningContext::GetScenarioInfo()->next_crosswalk_overlap = PathOverlap();
  ...
  ...
  for (... : ...) {
    if (...) {
      ...
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap =
          stop_sign_overlap;
    }
  }
  ...
  ...
  for (... : ...) {
    if (...) {
      ...
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap =
          traffic_light_overlap;
    }
  }
  ...
  ...
  for (... : ...) {
    if (...) {
      ...
      PlanningContext::GetScenarioInfo()->next_crosswalk_overlap =
          crosswalk_overlap;
    }
  }
  ...
}


// in planing_context.h:
struct ScenarioInfo {
  apollo::hdmap::PathOverlap next_stop_sign_overlap;
  apollo::hdmap::PathOverlap next_traffic_light_overlap;
  apollo::hdmap::PathOverlap next_crosswalk_overlap;
  std::string stop_done_overlap_id;
  ProceedWithCautionSpeedParam proceed_with_caution_speed;
  std::vector<std::string> stop_sign_wait_for_obstacles;
  std::vector<std::string> crosswalk_wait_for_obstacles;
};

// in path.h
struct PathOverlap {
  PathOverlap() = default;
  PathOverlap(std::string object_id, const double start_s, const double end_s)
      : object_id(std::move(object_id)), start_s(start_s), end_s(end_s) {}
  std::string object_id;
  double start_s = 0.0;
  double end_s = 0.0;
  std::string DebugString() const;
};
```

after ScenarioManager::Observe() is dispatched, ScenarioManager::SelectScenario() is dispatched in ScenarioManager::Update() 
```cpp
if (SelectScenario(scenario, ego_point, frame)) {
      AINFO << "select transferable scenario: "
            << ScenarioConfig::ScenarioType_Name(scenario);
      return;
    }
```

in ScenarioManager::SelectScenario(), ScenarioManager::CreateScenario() is dispatched to create Scenario type object scenario, after checking validation of it, current_scenario_ would point at it as a pointer.
```cpp
bool ScenarioManager::SelectScenario(const ScenarioConfig::ScenarioType type,
                                     const common::TrajectoryPoint& ego_point,
                                     const Frame& frame) {
  if (current_scenario_->scenario_type() == type) {
    return true;
  } else {
    auto scenario = CreateScenario(type);
    if (scenario->IsTransferable(*current_scenario_, ego_point, frame)) {
      AINFO << "switch to scenario: " << scenario->Name();
      current_scenario_ = std::move(scenario);
      return true;
    }
  }
  return false;
}


std::unique_ptr<Scenario> current_scenario_;
```

in ScenarioManager::CreateScenario(), a scenario object would be created according conf files, the method return a pointer at it.
```cpp
std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioConfig::ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;

  switch (scenario_type) {
    case ScenarioConfig::LANE_FOLLOW:
      ptr.reset(new lane_follow::LaneFollowScenario(config_map_[scenario_type],
                                                    &scenario_context_));
      break;
    ...
    ...
    ...
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}
```


## 2. Stage Layer

in PublicRoadPlanner::Plan(), Scenario::Process() is dispatched after scenario is updated. It takes frame and the start point of planning as input. In Scenario::Process(), Process() method is dispatched from different stages.
```cpp
auto ret = current_stage_->Process(planning_init_point, frame)
```

take lane_follow_stage.cc in lane_follow_scenario as example, LaneFollowStage::Process() takes planning_start_pointand frame as input. Incide this method LaneFollowStage::PlanOnReferenceLine() method is dispatched. 
```cpp
Stage::StageStatus LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
    ...
    ...
    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);
    ...
    ...
  }
  return ...;
}
```

## 3. Task Layer

LaneFollowStage::PlanOnReferenceLine() takes planning_start_point, frame, reference_line_info as input. After initialized the speed profile, it apply tasks on reference_line_info by dispatching Execute() method of each optimizer, with information from reference_line_info and frame
```cpp
Status LaneFollowStage::PlanOnReferenceLine(
        const TrajectoryPoint& planning_start_point, Frame* frame,
        ReferenceLineInfo* reference_line_info) {
  ...
  if (speed_profile.empty()) {
    speed_profile =
        SpeedProfileGenerator::GenerateSpeedHotStart(planning_start_point);
    ...
  }
  *heuristic_speed_data = SpeedData(speed_profile);
  ...
  for (auto* optimizer : task_list_) {
    ...
    ret = optimizer->Execute(frame, reference_line_info);
    ...
  }
  ...
}
```

LaneFollowStage is inherited from Stage class, task_list_ variable is also inherited from Stage class. task_list_ is initialized in Stage::Stage(), it reads conf files and produce task object with TaskFactory::CreateTask().
```cpp
Stage::Stage(const ScenarioConfig::StageConfig& config) : config_(config) {
  ...
  for (int i = 0; i < config_.task_type_size(); ++i) {
    auto task_type = config_.task_type(i);
    ...
    auto ptr = TaskFactory::CreateTask(*config_map[task_type]);
    task_list_.push_back(ptr.get());
    ...
  }
}
```

Lane follow scenario configures its task sequence like this:
```cpp
scenario_type: LANE_FOLLOW
stage_type: LANE_FOLLOW_DEFAULT_STAGE
stage_config: {
  stage_type: LANE_FOLLOW_DEFAULT_STAGE
  enabled: true
  task_type: DECIDER_RULE_BASED_STOP
  task_type: DP_POLY_PATH_OPTIMIZER
# task_type: QP_PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_DECIDER
  task_type: DP_ST_SPEED_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: QP_SPLINE_ST_SPEED_OPTIMIZER
  task_type: DECIDER_RSS
  ...
```
take dp_st_speed_optimizer as example. 
```cpp
DpStSpeedOptimizer::Process
```


All optimizers used in stage of speed decision making are inherited from class SpeedOptimizer, the SpeedOptimizer::Execute() method dispatches SpeedOptimizer::Process() to make planning, which takes input as below:
```cpp
virtual apollo::common::Status Process(
      const SLBoundary& adc_sl_boundary, const PathData& path_data,
      const common::TrajectoryPoint& init_point,
      const ReferenceLine& reference_line,
      const SpeedData& reference_speed_data, PathDecision* const path_decision,
      SpeedData* const speed_data) = 0
```

in DpStSpeedOptimizer::Process(), the StBoundayMapper step is the core step of speed planning.
