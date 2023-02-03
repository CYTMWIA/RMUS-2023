# RMUS-2023
RoboMaster 2023 机甲大师高校 Sim2Real 挑战赛

## 常用命令
```bash
# 构建 Docker 镜像
scripts/build_image.sh
# 启动测试镜像
scripts/launch_debug.sh
# 终止容器
scripts/halt.sh
# 重新摆放方块与重置机器人位置
scripts/reset.sh
# 键盘控制机器人
scripts/kbd_control.sh
```

**提交**  

```bash
# sim 镜像（仿真器调试阶段）
CLIENT_IMAGE=docker.discover-lab.com:55555/ironspirit/client:sim-$(date +"%Y%m%d") bash scripts/build_image.sh
docker push docker.discover-lab.com:55555/ironspirit/client:sim-$(date +"%Y%m%d")
# real 镜像（Sim2Real 调试阶段）
CLIENT_IMAGE=docker.discover-lab.com:55555/ironspirit/client:real-$(date +"%Y%m%d") bash scripts/build_image.sh
docker push docker.discover-lab.com:55555/ironspirit/client:real-$(date +"%Y%m%d")
```

## 待办事项/问题（按优先级排序）

- （功能、参数）导航偶尔会在接近障碍物的地方“假死”（导致机器人在未到达目的地的情况下执行下一步动作）
- （功能）最后对齐方块改用纯视觉方案，不使用深度相机
- （功能）在兑换后恢复并进入下一观测点的过程中，会重复检测先前区域
- （功能）在非寻找方块过程中对看到的其他目标方块予以记录，之后免去寻找方块的过程
- （参数）对准方块时，在最后的接近部分耗时较长，且少有实际移动（类似“抽搐“）

## 相关链接
[ICRA-RM-Sim2Real-Baseline](https://github.com/AIR-DISCOVER/ICRA-RM-Sim2real-Baseline)  
[ICRA-RM-Sim2Real-Baseline/reference.md](https://github.com/AIR-DISCOVER/ICRA-RM-Sim2Real-Baseline/blob/master/reference.md)  
[ICRA-RM-Sim2Real-Baseline/routines.md](https://github.com/AIR-DISCOVER/ICRA-RM-Sim2Real-Baseline/blob/master/routines.md)  
