### TODO
+ [Frame] 改特征点法为光流法，加速匹配。
+ [VO] MapPoint三角化。
+ [VO] 将pnp 改成　icp 求解

## 5.20
 
 
## 5.19

* PROBLEM: solvepnp  Assertion failed

- DONE: 解决了局内点数量都不足的问题，原因在于手贱修改了getPositionCV()中的返回值。
- DONE: 经过RANSAC提高了匹配精度后，01序列可以正常跑到180+帧。

## 5.18 
* PROBLEM: 代码误删...心态崩了
* PROBLEM: 代码恢复后，不知为何每次计算PnP时得到的局内点数量都不足

- DONE: 恢复到5.16的代码

## 5.17
* PROBLEM: 想使用ORB_SLAM2的 ORBextractor 来提取ORB特征点，代码移植失败

- DONE: 加入RANSAC算法，提高了匹配精度。从原来的近2000个特征点选出1500个匹配，改进到目前稳定到300-500个精确匹配点，几乎没有误匹配

## 5.16
* PROBLEM: 记录的轨迹只有KeyFrame的姿态，轨迹对比中要提取groundtruth中对应的帧的姿态跟其进行对比。

- DONE: 解决了KeyFrame的姿态对比问题。

## 5.15
* PROBLEM: unordered_map格式的map_points_在erase()时会造成内存溢出错误。
* PROBLEM: 计算到100帧左右累计误差过大，MapPoint没有更新，要想办法降低累计误差。

- DONE: 解决了unordered_map内存溢出的错误，不要在迭代器的值变动后进行访问，否则会出现越界的情况。
- DONE: 解决了程序执行一半过程中内存溢出的错误，将Frame类中特征点kpsLeft_和描述子descLeft_的尺寸与ind_kp_dep_一致。
- DONE: 初步完成了一个利用ICP对比KITTI轨迹的程序。

## 5.14
* PROBLEM: 计算到90帧左右累计误差过大，程序结束，要想办法降低累计误差。
* PROBLEM: 前90帧内也有个别帧误差很大，经过后面帧的优化后，后面帧的位姿可以拉回来，但前面帧估计的位姿无法拉回。

- DONE: 完成了深度计算的纠正。
- DONE: 把特征点计算和匹配改到Frame类里进行。
- DONE: 写了一个简单的记录轨迹的代码。

### 5.10
* PROBLEM: 第二帧后的MapPoints没有更新,关键帧检测失败？
* PROBLEM: 视差图计算不对，待纠正。

- DONE: 完成了双目的深度读取。

### 5.09
* PROBLEM: 计算深度部分的代码待修改。

- DONE: 采用slambook的project框架（RGB-D），初步完成几个类的改造。


