# 数据说明
## 数据格式
- 此数据为2d激光雷达录制数据包，格式参考如下:

```
<DataFrame1>
<DataFrame2>
...
<DataFrameN>
```
- 每条DataFrame格式如下:
```
<TimeStamp> <PointCount(M)> <Angle1> <Distance1> <Confidence1> <Angle2> <Distance2> <Confidence2>...<AngleM> <DistanceM> <ConfidenceM>
```
数据类型如下
- TimeStamp    类型uint64_t   单位ns
- PointCount   类型int
- Angle        类型float      单位角度
- Distance     类型float      单位米 量程12
- Confidence   类型uint8_t    范围0-255

## 数据场景
- lidar_data001.txt--雷达前进约0.55m
- lidar_data002.txt--雷达后退约0.61m
- lidar_data003.txt--雷达逆时针旋转约18度
- lidar_data004.txt--雷达顺时针旋转约18度
- lidar_data005.txt--雷达先前进约4.65m，后退至原点，再后退约4.85m，前进至原点
- lidar_data006.txt--雷达先顺时针旋转约1圈，再逆时针旋转约一圈
- lidar_data007.txt--雷达在地图中央8字运动
- lidar_data008.txt--雷达绕地图大范围8字运动
>注意上述距离及角度为估计值，且雷达带有噪声，测量有些许误差为正常现象