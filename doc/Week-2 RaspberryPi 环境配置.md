# Week-2 RaspberryPi 环境配置

[[2026-04-04]] 

搭建树莓派基础设施，材料：
1. Raspberry Pi 4 Model B
2. microSD 卡 32G，USB 读卡器
3. 网线，type-C 数据线

### 1. 在 microSD 卡烧录 Raspberry Pi OS

采用官方的 Raspberry Pi Imager 即可，记录：

```
Hostname: xzmeng
Username: xzmeng
Password: raspberry_pi
```

系统选择推荐的 64 bit 系统

Wifi 没有进行配置，因为运行时的 Wifi 环境不确定，并且不同地方分配给树莓派的 IP 地址不同。目前方案是用我的电脑进行网络共享

SSH 配置：目前已经将我的公钥上传至树莓派

Raspberry Pi Connect：目前已经将我的账号与这台硬件进行关联

现在操作系统已经全部烧录完成

### 2. 树莓派的网络连接 & SSH

将我的电脑跟树莓派的网口连接，进行网络共享

#### Mac 端网络设置

1. 在系统设置中导航至 通用 -> 共享
2. 在 互联网共享 中配置如下：
   共享以下来源的连接：选择 Wi-Fi（即 Mac 联网用的网卡）
   给使用以下端口的电脑：勾选对应的网口名称（如 AX88179B）
3. 打开 互联网共享 开关

#### SSH 配置

在终端输入：

```shell
arp -a
```

寻找接口名为 bridge100 下方的 IP 地址（我的是 192.168.2.2），然后使用：

```shell
ssh user_id@192.168.x.x
```

### 3. 项目环境配置迁移

目前在 Mac 上使用 anaconda 进行环境管理，但是树莓派的系统太小，不适宜继续使用 anaconda 管理。直接选择 Python 原生的 venv

当前项目的主要依赖如下：

```
mediapipe
opencv-contrib-python
pyserial
```

对于 CSI 摄像头（Camera Module v1/v2/v3），项目通过系统工具 `rpicam-vid`（基于 libcamera）采集图像，不需要 Python 层面的 `picamera2` 包。安装 `rpicam-vid`：

```bash
sudo apt update
sudo apt install rpicam-apps
```

#### Python 降级

由于 `mediapipe` 支持的 python 版本最多到 3.12，并且对于 3.11 的支持比较稳定，而 Raspberry Pi OS 自带的 python 版本是 3.13，所以需要对其进行降级

而操作系统并没有保留 python 3.11 的包，所以需要通过 `pyenv` 下载并管理

更新系统依赖：

```bash
sudo apt update
sudo apt install -y
```

安装 `pyenv`：

```bash
curl https://pyenv.run | bash
```

修改配置文件 `.bashrc`，添加如下配置：

```bash
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"
```

`source ~/.bashrc`

编译并使用 python 3.11.9，这步需要较长的时间，我等了快 15 分钟

```bash
pyenv install 3.11.9
pyenv global 3.11.9
```

#### 安装各种包

建立虚拟环境：

```bash
python -m venv my_robot_env
source my_robot_env/bin/activate
```

开始安装：

```bash
pip install mediapipe opencv-contrib-python pyserial
```

其他的包比如 `matplotlib` 之类的是上述三个包的依赖，会自动安装。但是`sounddevice` 依赖的底层 C 语言库 `PortAudio` 没有在系统中安装，需要执行：

```bash
sudo apt update
sudo apt install libportaudio2 libasound2-dev
```

给用户授权串口权限和音频、视频权限：

```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G audio $USER
sudo usermod -a -G video $USER
```

### 3. 拉取代码

```bash
git clone https://github.com/xiangzhen-meng/Robotics-from-Scratch.git
```

### 4. 运行方式

CSI 摄像头（Camera Module）使用 rpicam 后端：

```bash
source ~/my_robot_env/bin/activate
cd ~/Robotics-from-Scratch
python -m raspberry_pi.main \
    --port /dev/serial0 \
    --baudrate 115200 \
    --camera-backend rpicam \
    --frame-width 640 \
    --frame-height 480 \
    --camera-fps 15 \
    --num-poses 1
```

USB 摄像头使用 opencv 后端（Mac 环境或 USB 摄像头）：

```bash
python -m raspberry_pi.main \
    --port /dev/ttyACM0 \
    --baudrate 115200 \
    --camera-backend opencv \
    --camera-id 0
```

注意：不要尝试在 pyenv 虚拟环境中 `pip install picamera2`，它依赖系统 apt 包的 libcamera Python 绑定，与 pyenv Python 不兼容。CSI 摄像头应始终使用 `--camera-backend rpicam`。






