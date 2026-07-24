# tests

按四层建立测试：接口/消息、离线路径规划、轨迹与控制、全流程任务和故障注入。

`test_day5_run_tools.py` 覆盖 Day5 顺序启动顺序、latest-only QoS、
轨迹时长自适应看门狗，以及 `/tf_static` transient-local 录包约定。
