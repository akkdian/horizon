# camera_streamer/msg/SharedImage.msg
string shm_name           # 共享内存名称
uint32 width              # 图像宽度
uint32 height             # 图像高度
string encoding           # 图像编码格式
builtin_interfaces/Time timestamp  # 时间戳