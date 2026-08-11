# Lixel XBIN 数据提取工具

本目录用于检查和解析 XGRIDS/Lixel `.xbin` 录制文件，可以导出标定参数、
三路相机图片、原始激光点云以及 RTK/GNSS 数据。解析过程以只读方式打开原始
录制文件，不会修改 `.xbin`，也不会修改已安装的 LixelStudio 程序文件。

当前代码已在 **LixelStudio 4.0.1.4** 上完成验证，默认安装目录为：

```text
C:\Program Files (x86)\LixelStudio
```

## 一、完整数据导出

```powershell
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR
```

例如：

```powershell
python C:\Jiajia\python\xbin_tools\extract_lixel_dataset.py `
  C:\数据目录\recording.xbin `
  C:\数据目录\recording_export `
  --jobs 4
```

默认会导出以下内容：

- `calib/config/*.yaml`
  - 三个相机的内参、畸变参数和相机模型；
  - Camera–LiDAR 外参；
  - IMU–LiDAR 外参；
  - RTK 天线杆臂参数；
  - IMU、LiDAR 型号及标定参数。
- `images/camera_0/*.jpg`：左相机图片；
- `images/camera_1/*.jpg`：中相机图片；
- `images/camera_2/*.jpg`：右相机图片；
- `lidar_points/*.pcd`：原始单帧点云；
- `gnss.csv`：带时间戳的经纬高、定位状态、DOP、位置方差和卫星数；
- `raw_gnss_log.txt`：GNSS 接收机原始文本消息；
- `raw_ntrip_log.data`：原始 RTCM/NTRIP 差分数据；
- `raw_ppk_log.data`：原始 PPK 接收机数据；
- 各图片及点云目录下的 `timestamps.csv`；
- 输出根目录下的 `extraction_manifest.json`，记录数据源、导出参数和数量统计。

图片和点云文件均使用原始采集时间戳命名。PCD 中保留以下字段：

```text
x y z normal_x normal_y normal_z intensity timestamp ring
```

其中，PCD 文件名表示单帧时间戳，每个点的 `timestamp` 字段保存逐点时间戳。

## 二、选择需要导出的数据

可通过 `--components` 只导出部分数据：

```powershell
# 只导出标定参数
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --components config

# 只导出三路相机
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --components cameras

# 只导出点云
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --components lidar

# 只导出 RTK/GNSS
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --components rtk

# 同时导出多个类别
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --components config lidar rtk
```

可用类别如下：

| 参数 | 内容 |
|---|---|
| `all` | 全部数据，默认值 |
| `config` | 内参、外参和传感器配置 |
| `cameras` | 左、中、右三路相机图片 |
| `lidar` | 原始带逐点时间戳的单帧点云 |
| `rtk` | GNSS结果、接收机日志、NTRIP和PPK数据 |

## 三、指定时间范围

`--start-us` 是绝对 Unix 时间戳，单位为整数微秒；`--duration-s` 是导出时长，
单位为秒。`--duration-s 0` 表示从起始位置一直导出到录制结束。

```powershell
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --start-us 1785746830000000 `
  --duration-s 20
```

不指定时，默认从录制开头导出全部数据。

## 四、并行加速与断点续跑

完整导出默认并行处理最多 4 个传感器主题：

```powershell
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --jobs 4
```

- SSD 建议使用 `--jobs 3` 或 `--jobs 4`；
- 机械硬盘建议使用 `--jobs 1` 或 `--jobs 2`，避免并发写入导致速度下降。

任务中断后，可使用相同的数据源、时间范围和输出目录继续执行：

```powershell
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --jobs 4 `
  --resume
```

已成功完成的主题会根据 `OUTPUT_DIR\.extract_state` 中的状态文件自动跳过。

## 五、Binary PCD

LiDAR 默认直接写成标准的非压缩 Binary PCD：

```text
DATA binary
```

Binary PCD 完整保留所有字段和 double 精度逐点时间戳，同时避免厂商解码器中
耗时较大的 ASCII 数字格式化过程。需要兼容只能读取 ASCII PCD 的旧软件时，
可以使用：

```powershell
python .\extract_lixel_dataset.py INPUT.xbin OUTPUT_DIR `
  --pcd-format ascii
```

在 d3 数据的一段 205 帧测试中：

| 格式 | 导出时间 | 数据量 |
|---|---:|---:|
| ASCII PCD | 7.44 秒 | 323.3 MiB |
| Binary PCD | 2.61 秒 | 146.1 MiB |

Binary PCD 在该测试中提速约 **2.85 倍**，代表帧体积减少约 **54.8%**。
首帧、中间帧和末帧均完成逐点一致性验证，XYZ、强度、时间戳和 ring 与 ASCII
结果一致；Binary 中的时间戳精度高于保留六位小数的 ASCII 文件。

直接 Binary 写出功能仅对当前验证过的 LixelStudio 4.0.1.4 DLL 启用。代码会
检查 DLL 的 SHA256 和关键机器码；如果 DLL 版本不匹配，将在开始点云导出前
安全退出。所有修改只在当前 LiDAR 导出子进程的内存中生效，不会修改磁盘上的
厂商 DLL。

## 六、单主题底层导出

底层脚本使用 LixelStudio 自带的官方 `lixel_bag.dll`：

```powershell
python .\extract_lixel_xbin.py INPUT.xbin OUTPUT_DIR `
  --topic /lidar `
  --duration-s 1
```

直接输出 Binary PCD：

```powershell
python .\extract_lixel_xbin.py INPUT.xbin OUTPUT_DIR `
  --topic /lidar `
  --duration-s 1 `
  --binary-pcd
```

其他常用主题包括：

```text
/config_file
/camera_left/h264
/camera_center/h264
/camera_right/h264
/gnss_data
/raw_gnss_log
/raw_ntrip_log
/raw_ppk_log
```

## 七、原始数据保留说明

厂商解码器会输出零值无效回波，以及三个值为零的法向占位字段。工具不会主动
过滤或修改这些点，以保证导出结果仍然是帧级原始点云。如需过滤无效点，建议在
完成原始数据归档后另行处理。
