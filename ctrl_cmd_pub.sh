# ctrl_cmd_pub.sh — publish a single Control message to /control/cmd/control_cmd
# Usage: ./publish_control.sh <velocity> <steering_angle>

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 VELOCITY STEERING_ANGLE"
  echo "  VELOCITY       — desired forward speed (m/s)"
  echo "  STEERING_ANGLE — desired steering tire angle (rad)"
  exit 1
fi

VEL="$1"
STR="$2"

# bound velocity between -1.5 and 1.5 m/s
if (( $(echo "$VEL < -1.5" | bc -l) )); then
  VEL=-1.5
elif (( $(echo "$VEL > 1.5" | bc -l) )); then
  VEL=1.5
fi

# bound steering angle between -0.3 and 0.3 rad
if (( $(echo "$STR < -0.5" | bc -l) )); then
  STR=-0.3
elif (( $(echo "$STR > 0.5" | bc -l) )); then
  STR=0.3
fi

ros2 topic pub /control/command/control_cmd autoware_control_msgs/msg/Control \
"{stamp: {sec: 0, nanosec: 0}, control_time: {sec: 0, nanosec: 0}, \
 lateral: {stamp: {sec: 0, nanosec: 0}, control_time: {sec: 0, nanosec: 0}, \
 steering_tire_angle: $STR, steering_tire_rotation_rate: 0.0, \
 is_defined_steering_tire_rotation_rate: false}, \
 longitudinal: {stamp: {sec: 0, nanosec: 0}, control_time: {sec: 0, nanosec: 0}, \
 velocity: $VEL, acceleration: 0.0, jerk: 0.0, \
 is_defined_acceleration: false, is_defined_jerk: false}}"
