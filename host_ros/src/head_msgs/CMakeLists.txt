cmake_minimum_required(VERSION 3.8)
project(head_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTargets.msg"
  "msg/ServoState.msg"
  "msg/HeadState.msg"
  "msg/HeadDiagnostics.msg"
  "msg/ProprioceptionEvent.msg"
  "msg/ProprioceptionSessionState.msg"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  "srv/Enable.srv"
  "srv/StartProprioception.srv"
  "srv/StartRouting.srv"
  "srv/RenewControl.srv"
  "srv/Disable.srv"
  "srv/ClearFault.srv"
  "srv/SetFanOverride.srv"
  "srv/GetConfiguration.srv"
  "srv/ConfirmCalibration.srv"
  "srv/PrepareProprioceptionSession.srv"
  "srv/SetProprioceptionRecording.srv"
  "srv/AbortProprioceptionSession.srv"
  "action/HomeHead.action"
  "action/MaintenanceCalibrate.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
