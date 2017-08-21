# 多机器人协作物流快件自动分拣系统
# 机器人模拟类Robot Simulation

系统整体分为4个模块：

AStar类：改进的A*算法寻找K条候选路径；

ExperimentExecutor类：实验执行器，修改参数的接口；

ResEntityr类：路径资源实体，负责管理通行时间窗；

Robot类：机器人实体，负责管理机器人动作，资源等信息；

VirtualEnvironment类：实验运行模拟环境，负责管理环境拓扑，冲突错误等检测；
