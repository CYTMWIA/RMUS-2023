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

## 相关链接
[ICRA-RM-Sim2Real-Baseline](https://github.com/AIR-DISCOVER/ICRA-RM-Sim2real-Baseline)  
[ICRA-RM-Sim2Real-Baseline/reference.md](https://github.com/AIR-DISCOVER/ICRA-RM-Sim2Real-Baseline/blob/master/reference.md)  
[ICRA-RM-Sim2Real-Baseline/routines.md](https://github.com/AIR-DISCOVER/ICRA-RM-Sim2Real-Baseline/blob/master/routines.md)  
