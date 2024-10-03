# the ID is mutched when all components are all matched

# the inclusion relation is
# module : module is top
# - joint : modules contain joints
#   - motor: joints contain motors
# - connector : modules contian connectors
#   - motor: connectors contain motors
# - wheel:
# - gripper:
#   - motor: grippers contain motors
# - camera:
# - depthcamera:
# - imu:
# - lidar:
# - battery:
# - solar_panel:

# Note: the type and subtype might be not needed.
#       please give the empty value if it is not needed.

# - type: the type of mudular: body, leg, endeffector, sensor, ... please name a good name
# - name: the subtype of mudular: body_version1, body_version2, ... please name a good name
# - num:   the serial number of mudular.

string type
string subtype
int64 num