# 说明  ----已放弃------

此ros包实现joy消息向控制车辆的底层电机 的pwm信号以及舵机信号的转发

## 硬件

F710

## ubuntu 20.04

### 依赖包

sudo apt install ros-neotic-joy

roscore 

rosrun joy joy_node

rostopic echo joy



## STOP 紧急按钮 LB/RB ，按下则全车油门归零，舵机归零

#### 			#buttons[4] == 1 ||buttons[5] == 1 则 油门pwm = 1500,舵机为90 

## LT 纯前进按钮 1表示pwm 1500, -1 表示pwm 1700

### 		#axes[2] (-1至1 )map to (1500至 1700)

## RT 纯后退按钮 1 表示pwm 1500, -1 表示pwm 1300

#### 		##axes[5] (-1至1 )map to (1500至 1300)

### ##若出现LT&RT不同时为1 ，则同时碰触两个按钮，不做任何动作

# 左右摇杆功能相同

​		左摇杆 

​								axes[1] (0至1) 

​							map to (1500 至1700)

​		axes[0] (0至1)								axes[0] (0至-1)

​	map to (90 至 45)							map to (90 至 135)

​								axes[1] (0至-1) 

​							 map to (1500 至1300)



​		右摇杆 

​								axes[4] (0至1) 

​							map to (1500 至1700)

​		axes[3] (0至1)								axes[3] (0至-1)

​	map to (90 至 45)							map to (90 至 135)

​								axes[4] (0至-1) 

​							 map to (1500 至1300)





## LT->axes[2] (1至-1)     RT-> aexs[5] (1至-1) 

LB :  buttons[4] (0变1)

RB：buttons[5] (0变1)

BACK： buttons[6]  (0变1)

START： buttons[7]  (0变1)

## 左摇杆 

​						axes[1] (0至1)

axes[0] (0至1)								axes[0] (0至-1)

​						axes[1] (0至-1)

### 右摇杆

​						axes[4] (0至1)

axes[3] (0至1)								axes[3] (0至-1)

​						axes[4] (0至-1)

### 左按键

​						axes[7] (0至1)

axes[6] (0至1)								axes[6] (0至-1)

​						axes[7] (0至-1)

### 右按键

​						Y :     buttons[3] (0变1)

X:      buttons[2] (0变1)            B :    buttons[1] (0变1)

​						A ： buttons[0] (0变1)



