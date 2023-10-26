# 异构标定

## 传感器安装示意图

<img src="/home/lh/.config/Typora/typora-user-images/image-20231024104639827.png" alt="image-20231024104639827" style="zoom: 25%;" />

## 烟雾序列数据示意

红外-可见光-激光-毫米波

![image-20231024134210488](/home/lh/.config/Typora/typora-user-images/image-20231024134210488.png)

![image-20231024134232081](/home/lh/.config/Typora/typora-user-images/image-20231024134232081.png)

![image-20231024134252704](/home/lh/.config/Typora/typora-user-images/image-20231024134252704.png)

![image-20231024134307114](/home/lh/.config/Typora/typora-user-images/image-20231024134307114.png)

## 红外内参标定

两个版本：分别用matlab标定了有黑边1920\*1080和去除黑边640\*480图像。

需要用太阳光均匀照大标定板。

```matlab
cameraParams.ImageSize
1080    1920

cameraParams.IntrinsicMatrix
1110.82549521097	0	0
0	1109.54028549222	0
912.993567282656	452.657733048226	1

cameraParams.TangentialDistortion
0	0

cameraParams.RadialDistortion
-0.230149580219241	0.150581510284111
```

```matlab
cameraParams.ImageSize
480    640

cameraParams.IntrinsicMatrix
1104.50195815164	0	0
0	1104.80247345753	0
281.815052848494	166.229103132276	1

cameraParams.TangentialDistortion
0	0

cameraParams.RadialDistortion
-0.200600349900097   -0.045799082965466
```

<img src="/media/lh/lh1/dataset/thermal_radar/calib_data/thermal_image_test/1698042392675241357.png" alt="1698042392675241357" style="zoom: 33%;" />

<img src="/media/lh/lh1/dataset/thermal_radar/calib_data/crop_test/1698042393171473196.png" alt="1698042393171473196" style="zoom:50%;" />

## 激光-红外标定

点云转化关系：$P_c = R * P_l + t$，然后将$P_c$通过内参投影至像素。

https://github.com/icameling/lidar_camera_calibration/tree/manual_calib

原代码有万向节死锁问题，修改旋转顺序ZYX为YXZ。更换库函数setRPY()为setRPY_YXZ()函数。同理也可替换为XYZ等顺序。

根据订阅的图像是否为compressed需要修改代码中的变量类型。

| roll  | pitch | yaw   | x    | y     | z     |
| ----- | ----- | ----- | ---- | ----- | ----- |
| 84.70 | 84.00 | -2.20 | 0.00 | -0.18 | -0.18 |

```cpp
R lidar_to_cam:
  0.0664371   -0.992042 -0.00982091
  0.0976066   0.0218043   -0.988996
   0.987278   0.0590989    0.104451
P lidar_to_cam:
    0 -0.18 -0.18
```

![image-20231023210311298](/home/lh/.config/Typora/typora-user-images/image-20231023210311298.png)

![image-20231023210331428](/home/lh/.config/Typora/typora-user-images/image-20231023210331428.png)

![image-20231023210416170](/home/lh/.config/Typora/typora-user-images/image-20231023210416170.png)



## 激光-毫米波标定

根据原lidar_cam_calib的手动激光-毫米波标定。

点云转换关系：$P_{r}=R*P_{l}+t$

| roll | pitch | yaw  | x    | y    | z     |
| ---- | ----- | ---- | ---- | ---- | ----- |
| 0    | 0     | 5    | 0.22 | 0    | -0.28 |

```cpp
R lidar_to_radar:
  0.996195 -0.0871557          0
 0.0871557   0.996195          0
         0          0          1
P lidar_to_radar:
 0.22     0 -0.28
```

![image-20231024103208396](/home/lh/.config/Typora/typora-user-images/image-20231024103208396.png)

![image-20231024104039592](/home/lh/.config/Typora/typora-user-images/image-20231024104039592.png)

![image-20231024111740379](/home/lh/.config/Typora/typora-user-images/image-20231024111740379.png)

