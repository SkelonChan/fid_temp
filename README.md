nmea_navsat_driver是 RTK的官方驱动
encoder3是编码器节点的底层控制
ZED2 见博客https://blog.csdn.net/xiaojinger_123/article/details/121142167
razor。。。是IMU的包
# fid_temp
fid2 是舵机和电机 的arduino控制底层文件
transf 是编码器到里程计的变换
encoder 4 超出arduino uno的内存，放弃
# nmea_...为ros官方提供的包，整体放到同一个工作空间下有肯能会引发错误
对于rtk中的nmea--包需要单独放到一个工作空间
