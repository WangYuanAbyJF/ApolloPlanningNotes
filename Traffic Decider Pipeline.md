# Traffic Decider Pipeline
--------------------
* #### in `RunOnce` Cycle, after `Frame`, `ReferenceLineInfo` and `egoInfo` have been initialized, the `traffic_decider` variable of `StdPlanning` class would be initialized and apply registed traffic rules on each referenceLine of `reference_line_info_` variable of `frame_` variable of `StdPlanning` class:
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

* #### in codeblock above, the `traffic_decider` is initialized by `StdPlanning::traffic_rule_configs_` variable, in `TrafficDecider::Init()`, config files are transmitted to `TrafficDecider::rule_configs_`
```cpp
bool TrafficDecider::Init(const TrafficRuleConfigs &config) {
  if (s_rule_factory.Empty()) {
    RegisterRules();
  }
  rule_configs_ = config;
  return true;
}
```
* #### after `StdPlanning::traffic_decider` has been initialized, it would dispatch `TrafficDecider::Execute()` function to process each `reference_line_info`, it takes Frame and referenceLineInfo as input:
```
auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info)
```
* #### in `TrafficDecider::Execute()`, rules are produced from `TrafficDecider::s_rule_factory` from `TrafficDecider::rule_configs_`, then `rule->ApplyRule()` would be dispatched, which takes `frame` and `reference_line_info` as input.
```cpp
Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  for (const auto &rule_config : rule_configs_.config()) {
    if (!rule_config.enabled()) {
      ADEBUG << "Rule " << rule_config.rule_id() << " not enabled";
      continue;
    }
    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config);
    if (!rule) {
      AERROR << "Could not find rule " << rule_config.DebugString();
      continue;
    }
    rule->ApplyRule(frame, reference_line_info);
    ADEBUG << "Applied rule "
           << TrafficRuleConfig::RuleId_Name(rule_config.rule_id());
  }

  BuildPlanningTarget(reference_line_info);
  return Status::OK();
}
```

## different rule instances has different `ApplyRule()` methods, which would be demonstrated in "Traffic Rule Details", take `"backside_vehicle.cc"` as example:

* #### in `BacksideVehicle::ApplyRule()` dispatch `BacksideVehicle::MakeLaneKeepingObstacleDecision()`, takes return of `referenceLineInfo::path_decision()` and `referenceLineInfo::AdcSlBoundary()` as input
```cpp
Status BacksideVehicle::ApplyRule(
  Frame* const, ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  if (reference_line_info->Lanes().IsOnSegment()) {  // The lane keeping reference line.
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
  }
  return Status::OK();
}
```

* #### in `BacksideVehicle::MakeLaneKeepingObstacleDecision()` iterate over `PathDecision::obstacles`, make decisions using designed strategies. 
```cpp
void BacksideVehicle::MakeLaneKeepingObstacleDecision(const SLBoundary& adc_sl_boundary,
                                                      PathDecision* path_decision) {
  ObjectDecisionType ignore;
  ...
  ...
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle->PerceptionSLBoundary().end_s() >=
        adc_sl_boundary.end_s()) {  // don't ignore such vehicles.
      continue;
    }

    if (obstacle->reference_line_st_boundary().IsEmpty()) {
      path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/no-st-region",
                                        obstacle->Id(), ignore);
      continue;
    }
    // Ignore the car comes from back of ADC
    if (obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {
      path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                        obstacle->Id(), ignore);
      continue;
    }

    const double lane_boundary =
        config_.backside_vehicle().backside_lane_width();
  }
}
```

* #### For example, lateral decisions are added by dispatching `PathDecision::AddLateralDecision()`, in which `Obstacle::AddLateralDecision()` would be dispatched. 
```cpp
bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  obstacle->AddLateralDecision(tag, decision);
  return true;
}
```

* #### in `Obstacle::AddLateralDecision()`, `Obstacle::MergeLateralDecision()` is dispatched in order to decide if old decision is need to overwritten by new decision. Decision is stored in `PathDecision::obstacles`, in `Obstacle::lateral_decision_` and `Obstacle::longitudal_decision_`
```cpp
void Obstacle::AddLateralDecision(const std::string& decider_tag,
                                  const ObjectDecisionType& decision) {
  ...
  ...
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  ...
  ...
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}
```

