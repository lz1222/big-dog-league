# D1 原始 App 显示值；当前单位未获得界面证据，不能直接用作 JointState.position。
std_msgs/Header header
bool feedback_valid
bool feedback_stale
bool sources_consistent
float64[7] app_values
int32 enable_status
int32 power_status
int32 error_status
float32 feedback_age_sec
string value_unit
string reason
