# DroneX控制器仿真工程

## 仿真系统结构

### 接口数据结构定义
    按照信号类型分为位姿信号、控制信号、执行信号、力矩信号
    按照逻辑类型分为真值(物理世界理想信号)、采样值(考虑传感器噪声)、估计值(考虑建模误差与随机行为)

    一共有以下几种数据结构：
    POS_truth{//位姿真实值
        disp_xyz:三轴质心位移(世界参考系)
        att_quat:姿态四元数(世界参考系->机体参考系)
        vel_xyz:三轴质心线速度(世界参考系)
        acc_xyz:三轴质心加速度(世界参考系)
        angv_xyz:三轴角速度(机体参考系)
        anga_xyz:三轴角加速度(机体参考系)
    }


    Sensor_truth{//传感器系位姿真实值
        angv_xyz:三轴角速度(传感器参考系)
        acc_xyz:三轴加速度(传感器参考系)
    }
    //以上是Dynamics模块

    Sensor_obs{//传感器观测到的值，Sensor_truth带噪声
        angv_xyz:三轴角速度(传感器参考系)
        acc_xyz:三轴加速度(传感器参考系)
    }

    //以上是Sensor模块

    POS_est{//预测的机体位姿
        disp_xyz:三轴质心位移(世界参考系)
        att_quat:姿态四元数(世界参考系->机体参考系)
        vel_xyz:三轴质心线速度(世界参考系)
        acc_xyz:三轴质心加速度(世界参考系)
        angv_xyz:三轴角速度(机体参考系)
        anga_xyz:三轴角加速度(机体参考系)
    }

    //待定：预测算法的建模误差

    //以上是Estimator模块

    Ctrl_output{//控制器的输出
        thrust_xyz:目标推力矢量(机体系)
        torque_xyz:目标旋转扭矩(机体系)
    }

    //以上是Controller模块

    Ctrl_cmd{直接输出给执行器的控制信号
        Servo_x:使矢量单元沿x轴旋转的舵机占空比(在限幅内归一化)
        Servo_y:使矢量单元沿y轴旋转的舵机占空比(在限幅内归一化)
        Motor_P:正桨电机占空比
        Motor_N:反桨电机占空比
    }

    //以上是Allocator模块

    Actor_output{执行器输出的真实力矩
        Force_xyz:执行器作用在机体质心上的三轴力
        Torque_xyz:执行器作用在机体上、绕机体系三轴旋转的力矩
    }

    //以上是Actuator模块

    送入Dynamics积分


### Sensor(传感器模块)

模拟实机传感器模块的工作

读入真实位姿数据POS_truth,
    Sensor_err{//传感器安装误差
        disp_xyz:传感器参考系原点与机体参考系原点(质心)的位移误差
        disp_err:建模误差比例
        att_quat:机体参考系到传感器参考系的旋转姿态差
        att_err:建模误差比例
    }
以此为依据计算传感器系的真实位姿
为以上位姿添加噪声，模拟传感器噪声，传出Sensor_obs

### Estimator(位姿估计模块)

从传感器数据估计机体位姿

读入传感器系带噪声读数Sensor_obs
矫正、逆变换得到机体系位姿
位姿估计算法得到POS_est,传出

### Controller(控制器)

将位姿与目标位姿比较，输出控制矢量

读入估计位姿POS_est,更新控制器,输出Ctrl_output

### Allocator(执行器映射层)

根据目标控制矢量调整各执行器控制信号

读入Ctrl_output，通过执行器建模，计算执行器控制信号Ctrl_cmd

### Actuator(执行器模块)

根据控制信号，模拟执行器行为

读入Ctrl_cmd，根据执行器建模，加入噪声、随机行为、滞后、限幅等，输出模拟的实际输出物理量Actor_output

### Dynamics(物理仿真模块)

根据真实位姿和执行器实际输出更新真实位姿

读入Actor_output，按照运动学模型更新真实位姿，输出POS_truth

## 仿真系统设计

