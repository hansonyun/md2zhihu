#  简介
最近在深入研究 msckf 及其变种.其中最经典的当数 MingYang Li的 Msckf 2.0. 这篇文章就详细介绍了 msckf 2.0的各个细节. 重点关注msckf 2.0是如何发现msckf的不一致性, 理论分析是如何推导的以及最后是如何解决的.
# 详细理论分析
## 定义
MSCKF 2.0主要估计位姿, 提到位姿就需要对坐标系进行一下定义了. 在文章中定义了两个坐标系, Global Coordinate Frame, 简称$G$系, IMU Coordinate Frame, 简称$I$系. 同时，论文中也对各个变量的写法进行了规定。

- 坐标系表示：使用前上表表示变量表达在哪个坐标系下，例如$^Gp_{\ell}$表示$\ell$时刻$G$系下的位置。
- 旋转表示：$^A_BR$表示旋转矩阵，可以把$B$系下的矢量旋转到$A$系。$^A_B\overline{q}$表示单位四元数，其与$^A_BR$对应，表示的意义是一样的。
- $\lfloor c_\times\rfloor$代表反对称矩阵。$0,1$分别代表零矩阵以及单位矩阵。
- $\hat{a},\widetilde{a}$分别待估计量$a$的估计值以及误差值。$\hat{a}_{i|j}$代表使用$j$及$j$时刻之前的观测得到的估计值。

首先定义待估计状态，定义$\ell$时刻的 IMU 状态为 $16\times 1$的矢量,记为$x_{I_\ell}$,且$x_{I_\ell} = \begin{bmatrix}{}^{I_{\ell}}_G\overline{q}^T&^Gp^T_{\ell} &^Gv^T_{\ell} &b_{g_\ell}^T & b_{a_\ell}^T\end{bmatrix}^T$。$^{I_{\ell}}_G\overline{q}^T$代表从$G$系旋转到$I_\ell$系的单位四元数，$^Gp^T_{\ell} ,^Gv^T_{\ell}$代表IMU在$G$系下的位置及速度，$b_{g_\ell}^T , b_{a_\ell}^T$代表陀螺仪及加速度偏置。
其次定义IMU误差状态，对于$^Gp^T_{\ell} ,^Gv^T_{\ell}，b_{g_\ell}^T , b_{a_\ell}^T$，使用加性误差定义，即$^Gp^T = ^G\widetilde{p}^T+^G\hat{p}^T$。而对于旋转误差，则不能使用加性误差定义，因为是李群，不满足加法定义，定义旋转误差（旋转误差为小量）的四元数表示形式为$\delta{q} \simeq \begin{bmatrix} \frac{1}{2}^G\widetilde{\theta} & 1 \end{bmatrix}^T$，且满足$^{I}_G\overline{q}^T = ^{I}_G\hat{\overline{q}}^T\otimes{\delta{q}}$。
> 这里说一下，定义右扰可以使得可观测矩阵的扰动项中的部分项为0。参考..

## EKF-SLAM 及 MSCKF 介绍
### EKF-SLAM介绍
EKF-SLAM算是基于滤波的SLAM算法的鼻祖了，基本上MingYang Li以及他的导师都是以这个作为参考去评价算法的改进，同时其也比较有利于可观测性的分析。在EKF-SLAM算法中，$\ell$时刻的待估计变量包括IMU状态$x_{I_{\ell}}$以及若干特征点，可以表示为$x_{\ell} = \begin{bmatrix} x_{I_{\ell}} & f_1^T&f_2^T&...&f_{n_{\ell}}^T\end{bmatrix}^T$，其中$f_i,i=1,....,n_{\ell}$代表$\ell$时刻的待估计特征点变量。
$f_i$有很多表示形式，其中XYZ表示形式，逆深度表示形式以及锚定齐次形式（anchored homogenous）是最常见的三种表示形式，后面两种表示形式能够增加滤波器的一致性（consistency）和准确性（accuracy）。
> 这里其实对**一致性（consistency）**的定义我是挺模糊的，但是看了Jing胖大神的[如何理解一致性](https://www.zhihu.com/question/59784440/answer/170886408)后豁然开朗。引用里面一段最经典的表达。
> a state estimator is consistent if the estimation errors (i) are zero-mean, and (ii) have covariance matrix smaller or equal to the one calculated by the filter.
> 1.状态估计结果的误差是零均值的（无偏？）；2.状态估计结果的误差的协方差，比estimator估计的协方差要更小或相等（也就是说**估计的协方差**不能比**真实误差的协方差**更小）。
> 这里也补充一下无偏估计的理解，说实话，我对这个还是有点懵，参考[如何证明卡尔曼滤波无偏特性](https://www.zhihu.com/question/331568328/answer/735287556)。

EKF-SLAM的流程和基于EKF的状态估计流程基本上一致，分为状态传播（propagate）以及状态更新（update）两步，状态传播利用IMU测量进行，如下，其中$\Phi_I$代表IMU误差状态传播矩阵（error-state transition matrix）；$w_{d_{\ell}}$代表高斯噪声，其协方差矩阵为$Q_{d_{\ell}}$；$P_{II_{\ell|\ell}}$代表IMU状态的协方差矩阵；$P_{FF_{\ell|\ell}}$代表特征状态的协方差矩阵；$P_{IF_{\ell|\ell}}$代表IMU状态和特征状态的协方差矩阵。
$\begin{align}
\widetilde{x}_{I_{\ell+1|\ell}} &= \Phi_{I_\ell}\widetilde{x}_{I_{\ell|\ell}} +w_{d_\ell}\\
P_{I_{\ell+1|\ell}} &= \begin{bmatrix} \Phi_{I_\ell}P_{II_{{\ell|\ell}} }\Phi_{I_\ell}^T +Q_{d_{\ell}}&  \Phi_{I_\ell}P_{IF_{\ell|\ell}} \\ P_{IF_{\ell|\ell}}^T\Phi_{I_\ell}^T & P_{FF_{\ell|\ell}}\end{bmatrix}\tag{1}
\end{align}$
然后更新部分主要使用图像上特征点的观测进行。假定一个标定好的透视相机在$\ell$时刻观测到特征点$i$，观测可以表达为公式（2）。其中${}^C_I R, {}^Cp_I$代表camera和IMU之间的外参；$n_{i\ell}$代表零均值高斯噪声，其协方差矩阵为$\sigma^2I_2$;$H_{i\ell} = \begin{bmatrix}H_{I_{i\ell}}&0&...&H_{f_{i\ell}}&...&0 \end{bmatrix}$为$h$的雅克比矩阵，其计算点为$\hat{x}_{\ell|\ell-1}$。
$\begin{align}
{}^C_{\ell}p_{f_{\ell}}  &= \begin{bmatrix}x_{f_{\ell}} & y_{f_{\ell}} & z_{f_{\ell}} \end{bmatrix}^T = {}^C_IR \  {}^{I_{\ell}}_GR({}^{G}p_{f_{\ell}} -{}^Gp_{I_{\ell}})+{}^Cp_I\\
z_{i\ell} &= h(x_{I_{\ell}},f_i)+n_{i\ell} = \begin{bmatrix}\frac{x_{f_{\ell}}}{z_{f_{\ell}}} & \frac{y_{f_{\ell}}}{z_{f_{\ell}}} \end{bmatrix}^T + n_{i\ell}\\
r_{i\ell} &= z_{i\ell}-h(\hat{x}_{I_{\ell}},\hat{f}_i)\approx H_{i\ell}\widetilde{x}_{\ell|\ell-1}+n_{i\ell}

\end{align}\tag{2}$
一旦$H_{i\ell}$和$r_{i\ell}$计算出来，就就可以进行 Mahalanobis gating test, 用于剔除不正确的观测, 避免由于错误的观测进入到更新导致错误的估计产生. 同时, 特征点不同的形式将会产生不同的雅可比矩阵.
最后一步是状态的删除, 其主要目的是为了算法效率, 毕竟状态越大, 矩阵的运算越耗时, 为了保持实时性,必须剔除部分特征点. 那么删除哪些特征点呢, 在 EKF-SLAM中,主要是删除那些当前已经跟踪不到了的特征点, 或者说特征点已经在当前相机的视野之外了. 
### MSCKF介绍
与 EKF-SLAM不同的是, MSCKF在它的状态空间中维持了一个位姿的滑窗, 并且使用特征观测在位姿之间施加概率约束. msckf在$\ell$时刻的状态为$x_{\ell} = \begin{bmatrix}x_{I_{\ell}}& \pi^T_{\ell -1} & \pi^T_{\ell -2} & ...&\pi^T_{\ell -N} \end{bmatrix}^T$,其中$\pi_i=\begin{bmatrix}{}^{I_i}_G\overline{q}^T& {}^Gp_i^T \end{bmatrix},i\in \ell-N,...,
\ell-1$为过去$N-1$帧相机的位姿.
与 EKF-SLAM 一致的地方是, 在状态传播的过程中, IMU的测量用于传播 IMU 状态及其协方差. 与 EKF-SLAM不同的是在观测的使用上, 每当接受到一张图片时, 首先进行状态增广, 也就是将当前 IMU状态中的位姿及其协方差复制一份, 同时对图像的特征点进行跟踪以及匹配. 每个特征点直到所有的特征点都可用的时候再进行状态更新, 这样状态更新在执行时就可以使用特征点的所有测量了.
对于 MSCKF 的更新, 首先假定在$\ell$时刻特征点$f_i$被$N$帧相机观测到了, 然后利用$N$帧相机位姿及其特征点的观测通过 Gauss-Netween 的方法将$f_i$的坐标计算出来. 得到了坐标点后我们就可以计算残差, 如公式(3)所示. 其中$H_{\pi_{ij}},H_{f_i}$分别代表残差$r_{ij}$对位姿及特征点误差$\tilde{\pi},{}^G\tilde{p}_{f_i}$的雅可比矩阵
$\begin{align}
r_i &= z_i - h(\hat{\pi}_{\ell|\ell -1},{}^G\hat{p}_{f_i}) \\
&\simeq H_{\pi_i}(\hat{x}_{\ell|\ell-1},{}^G\hat{p}_{f_i})\tilde{x}_{\ell|\ell-1}+H_{f_i}(\hat{x}_{\ell|\ell-1},{}^G\hat{p}_{f_i}){}^G\tilde{p}_{f_i}+n_i \\
\begin{bmatrix}r_{i1}\\ r_{i2}\\.\\.\\.\\r_{iN} \end{bmatrix} &=\begin{bmatrix} 
H_{\pi_{i1}}(\hat{\pi}_{1|\ell-1},{}^G\hat{p}_{f_i})&0&....&H_{f_{i1}}(\hat{\pi}_{1|\ell-1},{}^G\hat{p}_{f_i})\\
0&H_{\pi_{i2}}(\hat{\pi}_{2|\ell-1},{}^G\hat{p}_{f_i})&....&H_{f_{i2}}(\hat{\pi}_{2|\ell-1},{}^G\hat{p}_{f_i})\\
0&...&H_{\pi_{iN}}(\hat{\pi}_{N|\ell-1},{}^G\hat{p}_{f_i})&H_{f_{iN}}(\hat{\pi}_{N|\ell-1},{}^G\hat{p}_{f_i})
\end{bmatrix}\begin{bmatrix}\tilde{\pi}_{i1}\\\tilde{\pi}_{i1}\\.\\.\\.\\\tilde{\pi}_{iN}\\{}^G\tilde{p}_{f_i} \end{bmatrix} +n_i
\end{align} \tag{3}$
上面这种形式是没有办法更新的, 因为对于 EKF 更新来说, 需要$r = H\tilde{x} + n$这种形式且$n$和$\tilde{x}$要相互独立, 但是msckf 得状态里面又没有${}^G\tilde{p}_{f_i}$且${}^G\tilde{p}_{f_i}$与$\tilde{x}_\ell$之间又不是独立的(因为特征点是由位姿以及特征点观测计算处理的). 为了能够获取用于更新的形式, 采用左零空间的方式消除$H_{f_i}(\hat{x}_{\ell|\ell-1},{}^G\hat{p}_{f_i}){}^G\tilde{p}_{f_i}$这一项. 定义矩阵$V_i$,其列向量构成了$H_{f_i}$矩阵的左零空间. 用$V_i^T$分别乘在公式(3)的左右两侧,可以得到
$\begin{align}
r_i^o &= H_i^o(\hat{x}_{\ell|\ell-1},{}^G\hat{p}_{f_i})\tilde{x}_{\ell|\ell-1} + n_i^o\\
V_i^Tr_i&=V_i^TH_{\pi_i}(\hat{x}_{\ell|\ell-1},{}^G\hat{p}_{f_i})\tilde{x}_{\ell|\ell-1} + V_i^Tn_i
\end{align}\tag{4}$
一旦$H_{i}^o$和$r_{i}^o$计算出来，就就可以进行 Mahalanobis gating test, 用于剔除不正确的观测, 避免由于错误的观测进入到更新导致错误的估计产生. 这里 msck 论文给出了 Mahalanbis gating test的详细计算, $\gamma_i = (r^o_i)^T(H_i^oP_{\ell|\ell-1}(H_i^o)^T+\sigma^2I)^{-1}r^o_i$, $\gamma$也称为卡方距离, 论文中以自由度为$2N-3$的卡方分布 95%的值为阈值进行校验.
最后一步是删除部分状态节约算力.论文中通过移除滑窗中的位姿来进行, 移除哪些位姿呢? 如果该位姿所有观测到的特征点都被用于更新后,则移除该位姿.
![image.png](https://cdn.nlark.com/yuque/0/2023/png/27781308/1681031161863-ced9d8ca-9471-4f19-ba8b-f27a35bd51c7.png#averageHue=%23fbf9f6&clientId=uae1081cf-a63f-4&from=paste&height=255&id=u918458be&originHeight=281&originWidth=755&originalType=binary&ratio=1.100000023841858&rotation=0&showTitle=false&size=76677&status=done&style=none&taskId=u95423530-3b67-46e9-a69a-e780374743c&title=&width=686.3636214871055)
### EKF-SLAM与 MSCKF测试对比
论文第3节的最后一部分是对比EKF-SLAM以及MSCKF。这里我没有细研究MingYangLI是如何构建的仿真对比过程，只先提供解决结论吧。论文中主要通过extensive Monte-Carlo simulations来构建仿真数据，对比两个算法的优劣，指标分别是NEES(averge normalized estimation error squared)以及RMSE(root mean squared error)此处需要补充NEES以及RMSE的计算公式，前一个指标能够反应算法的一致性，后一个指标能够反应算法估计位姿的准确性。在仿真测试中，不管是准确性（smaller RMSE）还是一致性（NEES closre to six）上 MSCKF 都比 EKF-SLAM要强，论文中将其归结为两个原因

- 首先，每次迭代前假设位姿与特征点的误差是联合高斯分布。但是，由于相机测量模型的非线性特性，联合高斯分布并不是一个良好的近似。同时，不同的特征点表达方式具有不同的近似程度，AHD和IDP近似程度较好，XYZ近似程度最差。相比较而言，MSCKF并没有把特征点放在待估计状态中，没有假设特征点的概率分布，因此避免掉了影响准确性的主要方面。
- 其次，在EKF-SLAM中，特征点观测线性化以及处理在每一步都进行。相比而言，MSCKF采用了“延迟线性化（delay linearization）”方法，每个特征点仅在其所有的观测都变得可用的情况下进行处理。这就意味着在估计雅克比矩阵时，其路标点的位置估计较好，使得卡尔曼增益以及状态校正计算的精度也就更高。

同样，在第3节的末尾，也对算力进行了测试。实际测量结果为 MSCKF 0.93msec/update, XYZ-EKF-SLAM 1.54msec/update, IDP-EKF-SLAM 3.28msec/update, AHP-EKF-SLAM 4.45msec/update. 实际测量结果与理论测量结果一致，即MSCKF的算例消耗与特征点的数量成线性关系，EKF-SLAM则成三次方关系。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/27781308/1681031175074-5c009c63-226a-499b-a676-bb0090fc02d0.png#averageHue=%23f9f6f6&clientId=uae1081cf-a63f-4&from=paste&height=582&id=u95d4145e&originHeight=640&originWidth=864&originalType=binary&ratio=1.100000023841858&rotation=0&showTitle=false&size=108616&status=done&style=none&taskId=u14677a61-1ffb-4935-a650-c98436c323b&title=&width=785.454528430277)
![image.png](https://cdn.nlark.com/yuque/0/2023/png/27781308/1681031187934-9cd6aa3c-dc44-4ebc-8a5b-d88caea5cd04.png#averageHue=%23fdfbf9&clientId=uae1081cf-a63f-4&from=paste&height=210&id=LQAIr&originHeight=231&originWidth=795&originalType=binary&ratio=1.100000023841858&rotation=0&showTitle=false&size=55292&status=done&style=none&taskId=ube8d0630-0386-49fb-b8f4-b291d3cc42e&title=&width=722.7272570625813)

## 如何发现 MSCKF 的不一致性
 	从2.2.3的测试结果上可以看出，MSCKF的NEES为7.741，且最后100s的NEES的平均值为10.6，其理论值应该是小于等于6的，这说明滤波器变得不一致了，且不一致性在长轨迹中逐渐变得明显。接下来在论文的第4，5章节，首先简单介绍可观性矩阵， 然后介绍误差状态转移矩阵的计算，方便理论分析。
### 可观性矩阵简要介绍
 考虑一个非线性方程公式（4）描述的物理系统，其中$x$为系统状态，$u$是控制输入，$z$为观测，$w,n$是噪声。
$\begin{align}\dot{x} &= f(x,u) + w\\
z&=h(x) +n \end{align}\tag{4}$
为了能在电脑上估计$x$,必须对非线性模型进行离散化。同时，当使用EKF进行状态估计的时候，滤波方程（协方差传播和更新，增益计算）依赖离散模型的版本，其通用的形式如公式（5）所示。其中$\Phi_{\ell},H_{\ell}$分别代表状态转移矩阵（error-state transition matrix）以及测量雅克比矩阵(measurement jacobian matrix)。
$\begin{align}\tilde{x}_{\ell} &= \Phi_{\ell}\tilde{x}_{\ell-1} + w_{\ell} \\ 
\tilde{r}_{\ell}&=H_\ell\tilde{x}_{\ell} + n_{\ell}
\end{align} \tag{5}$
理想情况下，公式（5）的可观特性应该和公式（4）的可观特性一致，即如果状态中的任何一个在实际的非线性系统中不可观，那么在线性系统中，其误差也不可被观测，否则在EKF估计状态的过程中，关于该状态的虚假信息将被将累计，造成不一致。
当一个camera+imu相机系统在已知重力不知道特征点位置的环境下移动时，visual-inertial navigation有4个自由度不可观测。（i）全局位置。（ii）航向角。VIO系统在$\begin{bmatrix} k,k+m\end{bmatrix}$时间间隔内的可观测性可以通过公式（5）的可观测矩阵得到，如公式（6）所示。理想情况下，$O$的零空间描述了观测对状态空间中的哪些方向没有提供信息，即不可观测方向。同时，非线性系统与线性系统应该有类似的可观测特性，也就是说，公式（6）的零空间应该有4个维度，同时这4个维度对应这全局位置以及航向角。但是，实际的VIO系统中（EKF-SLAM,MSCKF）航向角在线性系统中变得可观测了。
$O\triangleq\begin{bmatrix}
H_k\\
H_{k+1}\Phi_k\\
.\\.\\.\\
H_{k+m}\Phi_{k+m-1}...\Phi_{k}
\end{bmatrix}\tag{6}$
### 计算状态转移矩阵
从公式（6）的形式可以看出，要想计算可观测矩阵，必须先要计算状态转移矩阵$\Phi$以及测量雅克比矩阵$H$。状态转移矩阵的计算有多种形式，比如使用龙格-库塔积分，一步欧拉积分（$\Phi_{I} = I + F\Delta{t}$）等等。但是这些方法都过于数值化了，不利于理论分析（这段话我理解就是每种数值积分方法都有自己的形式）。更重要的是，如果状态转移矩阵计算的太数值化或者近似，不能保证其具有可观测特性（我理解这里应该是全局位姿和航向角不可观测特性）。为此，MingYangLI提出了一种状态转移矩阵的闭式形式，也就是状态转移矩阵可以表示为状态估计值的函数，表达形式就是$\Phi = f(\hat{x})$。
#### IMU测量提供了什么？
在VIO系统中，状态转移矩阵主要是靠IMU的测量驱动的。IMU主要有陀螺仪，加速度计组成，有的还提供磁力计的测量。其中陀螺仪提供IMU坐标系下角速度的测量，记为$\omega_m(t)$；加速度计提供IMU坐标系加速度测量，记为$a_m(t)$。其表达形式如公式（7）所示，其中${}^I\omega(t),{}^Ga(t)$分别代表角速度以及线性加速度的真实值（true state），$n_r(t),n_a(t)$代表高斯白噪声，$g$代表全局坐标系下的重力矢量。
$\begin{align}\omega_m(t) &= {}^I\omega(t) + b_g(t) + n_r(t)\\ a_m(t) &= {}^I_GR(t)({}^Ga(t) -g) + b_a(t)+ n_a(t)\end{align}\tag{7}$
假定在$\begin{bmatrix}t_{\ell}&t_{\ell+1} \end{bmatrix}$时间内$\omega_m(t),a_m(t)$任意时间的值均可以获取，也就是说是连续时间信号。那么对于陀螺仪的角速度测量来说，可以获取$\begin{bmatrix}t_{\ell}&t_{\ell+1} \end{bmatrix}$时间内的相对旋转，即${}^{I_{\ell+1}}_{I_{\ell}}\hat{\overline{q}}$。定义$t$时刻的旋转角速度估计值为$\hat{\omega}(t) = \omega_m(t) - \hat{b_g}(t)$，初始姿态${}^{I_{\ell}}_{I_{\ell}}\hat{\overline{q}}= \begin{bmatrix} 0&0&0&1\end{bmatrix}^T$，然后对公式（8）所示的微分方程进行积分即可得到${}^{I_{\ell+1}}_{I_{\ell}}\hat{\overline{q}}$。然后将其与$\ell$时刻的旋转估计值相乘，就可以得到$\ell+1$时刻的旋转估计值，即${}^{I_{\ell+1}}_{I_{G}}\hat{\overline{q}} = {}^{I_{\ell+1}}_{I_{\ell}}\hat{\overline{q}}\odot{{}^{I_{\ell}}_{I_{G}}\hat{\overline{q}}}$。
${}^{I_t}_{I_{\ell}}\dot{\hat{\overline{q}}} = \frac{1}{2}\begin{bmatrix}-\lfloor \hat{\omega(t)}_\times \rfloor & \hat{\omega(t)} \\ -\hat{\omega(t)}^T&0\end{bmatrix}{}^{I_t}_{I_{\ell}}{\hat{\overline{q}}} \tag{8}$
同样，对于加速度计来说，可以获取$\begin{bmatrix}t_{\ell}&t_{\ell+1} \end{bmatrix}$时间内的相对速度以及相对位置。定义$t$时刻的加速度估计值为$\hat{a}(t) = {}^G_{I_t}\hat{R}(a_m(t) -\hat{b_a}(t))+g$。通过对其进行两次积分，可以得到$\ell+1$时刻的速度及位置估计值，如公式（9）所示。
$\begin{align} \Delta{t} &= t_{\ell+1} - t_{\ell}\\
\hat{s_{\ell}}&=\int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}R(a_m(\tau) - \hat{b_a}(\tau))d\tau\\
\hat{y_{\ell}}&=\int_{t_\ell}^{t_{\ell+1}}\int_{t_\ell}^{s}{}^{I_\ell}_{I_\tau}R(a_m(\tau) - \hat{b_a}(\tau))d\tau ds\\
{}^G\hat{v}_{\ell+1}&={}^G\hat{v}_{\ell}+\int_{t_\ell}^{t_{\ell+1}} {}^G\hat{a}(\tau)d\tau\\
&={}^G\hat{v}_{\ell}+\int_{t_\ell}^{t_{\ell+1}}  {}^G_{I_\tau}\hat{R}(a_m(\tau) -\hat{b_a}(\tau)) d\tau + g\Delta{t}\\
&={}^G\hat{v}_{\ell}+{}^G_{I_\ell}\hat{R}\int_{t_\ell}^{t_{\ell+1}}  {}^{I_\ell}_{I_\tau}\hat{R}(a_m(\tau) -\hat{b_a}(\tau)) d\tau + g\Delta{t}\\
&={}^G\hat{v}_{\ell}+{}^G_{I_\ell}\hat{R}\hat{s}_\ell + g\Delta{t}\\
{}^G\hat{p}_{\ell+1}&={}^G\hat{p}_{\ell}+{}^G\hat{v}_{\ell}\Delta{t}+\int_{t_\ell}^{t_{\ell+1}}\int_{t_\ell}^{s} {}^G\hat{a}(\tau)d\tau ds\\
&={}^G\hat{p}_{\ell}+{}^G\hat{v}_{\ell}\Delta{t}+{}^G_{I_\ell}\hat{R}\int_{t_\ell}^{t_{\ell+1}}\int_{t_\ell}^{s}  {}^{I_\ell}_{I_\tau}\hat{R}(a_m(\tau) -\hat{b_a}(\tau)) d\tau ds + \frac{1}{2}g\Delta{t}^2\\
&={}^G\hat{p}_{\ell}+{}^G\hat{v}_{\ell}\Delta{t}+{}^G_{I_\ell}\hat{R}\hat{y}_\ell + \frac{1}{2}g\Delta{t}^2
\end{align}\tag{9}$
#### 离散IMU传播
公式(8,9)假设$\omega_m(t),a_m(t)$在整个积分时间段内可用. 但是实际上,IMU只在离散时间$t_\ell$和$t_{\ell+1}$时刻进行了采样.为了能够进行状态传播,必须进行一定的假设,例如可以假设信号在积分时间段内保持不变(等于$t_\ell$或者 $t_{\ell+1}$时刻的值)或者在整个积分时间段内线性变化. 虽然说这些假设会引入一定近似,但是在imu频率较高时是很小的.
在论文中, MingYangLI将陀螺仪及加速度偏置建模为随机游走, 即$\dot{b_g}(t) = n_{wg}(t),\dot{b_a}(t) = n_{wa}(t)$, $n_{wg}(t),n_{wa}(t)$是零均值高斯白噪声. 因此,在积分时间段内,其$\hat{b_g}_{\ell+1} = \hat{b_g}_{\ell}, \hat{b_a}_{\ell+1} = \hat{b_a}_{\ell}$. 同时,对于 IMU信号,假定其在积分时间段内线性变化. 对于公式(8)的所示的微分方程,使用四阶龙格-库塔方法计算${}^{I_{\ell+1}}_{I_{\ell}}\hat{\overline{q}}$. 同时,对于公式(9)所示的积分方程$\hat{s}_\ell, \hat{y}_\ell$, 使用 Simpson 积分进行计算.
#### 误差状态转移矩阵计算
首先来推导旋转误差状态的转移方程. 四元数形式的旋转误差已经在第 1 节中进行了定义,下面定义一下旋转矩阵形式的旋转误差,定义${}^I_GR\simeq {}^I_G\hat{R}(I_3-\lfloor {}^G\tilde{\theta}_\times\rfloor)$. 同时, 由于 IMU 的角速度测量存在误差, 且在积分过程中存在近似, 所以对于${}^{I_{\ell+1}}_{I_{\ell}}{\overline{q}}$也存在误差,定义${}^{I_{\ell+1}}_{I_{\ell}}{\overline{q}} = {}^{I_{\ell+1}}_{I_{\ell}}\hat{\overline{q}}\otimes \delta{\overline{q}}_{\Delta{t}}$, 表示为旋转矩阵的形式为${}^{I_{\ell+1}}_{I_{\ell}}R = {}^{I_{\ell+1}}_{I_{\ell}}\hat{R}(I-\lfloor \tilde\theta_{\Delta{t}}\rfloor)$. 将上面两个公式带入到${}^{I_{\ell+1}}_GR = {}^{I_{\ell+1}}_{I_{\ell}}R{}^{I_{\ell}}_GR$中,可以得到旋转误差状态的转移方程
${}^G\tilde{\theta}_{\ell+1} \simeq {}^G\tilde{\theta}_{\ell} + \hat{R}^T_\ell\tilde{\theta}_{\Delta{t}}\tag{10}$
随后推导速度误差状态转移方程. 定义$\tilde{s}_\ell = s_\ell - \hat{s}_\ell$, 同时将${}^I_GR\simeq {}^I_G\hat{R}(I_3-\lfloor {}^G\tilde{\theta}_\times\rfloor)$以及方程(9)中的速度估计值部分带入到${}^G{v}_{\ell+1}={}^G{v}_{\ell}+{}^G_{I_\ell}{R}{s}_\ell + g\Delta{t}$中, 可以得到速度状态误差的转移方程为
${}^G\tilde{v}_{\ell+1}\simeq -\lfloor \hat{R}_{\ell}^T\hat{s}_{\ell} {}\times\rfloor {}^G\tilde{\theta}_\ell + {}^G\tilde{v}_{\ell}+\hat{R}_{\ell}^T\tilde{s}_\ell \tag{11}$
同理,定义$\tilde{y}_\ell = y_\ell - \hat{y}_\ell$, 同时将${}^I_GR\simeq {}^I_G\hat{R}(I_3-\lfloor {}^G\tilde{\theta}_\times\rfloor)$以及方程(9)中的位置估计值部分带入到${}^G{p}_{\ell+1}={}^G{p}_{\ell}+{}^G{v}_{\ell}\Delta{t}+{}^G_{I_\ell}{R}{y}_\ell + \frac{1}{2}g\Delta{t}^2$中,可以得到位置状态误差的转移方程为
${}^G\tilde{p}_{\ell+1}\simeq-\lfloor \hat{R}_{\ell}^T\hat{y}_{\ell} {}\times\rfloor {}^G\tilde{\theta}_\ell + {}^G\tilde{p}_{\ell}+{}^G\tilde{v}_{\ell}\Delta{t}+\hat{R}^T_\ell\tilde{y}_\ell  \tag{12}$
结合公式(10), (11) 以及(12), 我们可以写出联合形式
$\underbrace{\begin{bmatrix} 
{}^G\tilde{\theta}_{\ell+1}  \\ {}^G\tilde{p}_{\ell+1} \\ {}^G\tilde{v}_{\ell+1}\end{bmatrix}}_{\tilde{x}_{I_{\ell+1}}} = 

\underbrace{\begin{bmatrix}I_3 & 0_3&0_3\\-\lfloor \hat{R}_{\ell}^T\hat{y}_{\ell} {}\times\rfloor &I_3&\Delta{t}I_3\\-\lfloor \hat{R}_{\ell}^T\hat{s}_{\ell} {}\times\rfloor &0_3&I_3 \end{bmatrix}}_{\Phi_{I_{\ell}}}

\underbrace{\begin{bmatrix} {}^G\tilde{\theta}_{\ell}  \\ {}^G\tilde{p}_{\ell} \\ {}^G\tilde{v}_{\ell}\end{bmatrix}}_{\tilde{x}_{I_{\ell}}}
+
\underbrace{\begin{bmatrix} \hat{R}^T_\ell\tilde{\theta}_{\Delta{t}} \\ \hat{R}^T_\ell\tilde{y}_\ell \\ \hat{R}^T_\ell\tilde{s}_\ell\end{bmatrix}}_{w_{d_\ell}}\tag{13}$
公式(13)中的误差状态转移矩阵$\Phi_{I_\ell}$具有直观的意义. (i) 首先是对角线, 代表上一帧误差状态的累积. (ii) $\ell$时刻的速度误差乘以$\Delta{t}$影响着$\ell+1$时刻的位置误差. (iii) $\ell$时刻的旋转误差乘以"lever-arms"$-\lfloor \hat{R}_{\ell}^T\hat{s}_{\ell} {}\times\rfloor,-\lfloor \hat{R}_{\ell}^T\hat{y}_{\ell} {}\times\rfloor$, 影响着$\ell+1$时刻的位置及速度误差.
为了将误差状态转移矩阵$\Phi_{I_\ell}$写成待估计量的函数. 将$\tilde{y}_\ell,\tilde{s}_\ell$按照公式(9)的形式进行替换,并带入公式(13), 可以得到
$\begin{align}
\Phi_{I_\ell}(\hat{x}_{I_{\ell+1}},\hat{x}_{I_{\ell}}) &= \begin{bmatrix}I_3 & 0_3&0_3\\\Phi_{pq}(\hat{x}_{I_{\ell+1}},\hat{x}_{I_{\ell}})  &I_3&\Delta{t}I_3\\\Phi_{vq}(\hat{x}_{I_{\ell+1}},\hat{x}_{I_{\ell}}) &0_3&I_3 \end{bmatrix} \\
\Phi_{vq}(\hat{x}_{I_{\ell+1}},\hat{x}_{I_{\ell}})&=-\lfloor({}^G\hat{v}_{\ell+1}-{}^G\hat{v}_{\ell}-g\Delta{t})_{\times} \rfloor \\
\Phi_{pq}(\hat{x}_{I_{\ell+1}},\hat{x}_{I_{\ell}})&=-\lfloor({}^G\hat{p}_{\ell+1}-{}^G\hat{p}_{\ell}-{}^G\hat{v}_{\ell}\Delta{t}-\frac{1}{2}g\Delta{t}^2)_{\times} \rfloor 
\end{align}\tag{14}$
从公式(14)可以看出, 误差状态转移矩阵只与状态估计值相关, 与公式(8)(9)的积分形式无关.
## MSCKF 可观性的分析
论文中对可观性的分析主要是通过分析可观性矩阵的零空间来实现的。首先作者先说明了一下为什么在待估计的状态空间中没有偏置，首先最重要的一条是偏置是可观测的，同时带不带偏置其可观测性矩阵的零空间维度都会下降，为了简化分析就不带了。同时，作者采用的是EKF-SLAM的测量雅克比矩阵构建可观性矩阵，主要原因是由于MSCKF的测量方程太复杂（能不复杂嘛，又是组合，又是左零空间的）且EKF-SLAM和MSCKF在给定线性模型时，其测量方程是等效的。
对于EKF-SLAM， 假定其状态空间包含IMU姿态，位置以及速度，同时也包含$\begin{bmatrix}k,k+m \end{bmatrix}$时间间隔内所有的路标点的位置。对于该状态空间，其$\ell$时刻的误差状态转移矩阵如公式（15）所示，其中$M$为路标点的数量。
$\Phi_{\ell} (\hat{x}_{I_{\ell+1|\ell}},\hat{x}_{I_{\ell|\ell}}) = 
\begin{bmatrix}
\Phi_{I_\ell} (\hat{x}_{I_{\ell+1|\ell}},\hat{x}_{I_{\ell|\ell}})  & 0 \\ 0 &I_{3M\times 3M}
\end{bmatrix} \tag{15}$
对于特征点测量，假设特征点$i$在$\alpha_i +1$时刻处理，那么相应的雅克比计算使用“直到$\alpha_i$时刻的测量估计出来的状态”以及“三角化计算出来的特征点位置${}^G\hat{p}_{f_i}$”，可以得到测量雅克比矩阵为
$\begin{align}
H_{i\ell}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &= \begin{bmatrix}H_{I_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &0&...&H_{f_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &...&0 \end{bmatrix}\\
H_{f_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &= J_{i\ell}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}){}^C_IR\hat{R}_{\ell|\alpha_i}\\
H_{I_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &=H_{f_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i})\begin{bmatrix}\lfloor ({}^G\hat{p}_{f_i} - {}^G\hat{p}_{\ell|\alpha_i})_{\times} & -I_3&0_3 \end{bmatrix}\\
\begin{bmatrix} {}^{C_\ell}\hat{x}_{f_i} &{}^{C_\ell}\hat{y}_{f_i}&{}^{C_\ell}\hat{z}_{f_i} \end{bmatrix}^T &= {}^C_IR\hat{R}_{\ell|\alpha_i}( {}^G\hat{p}_{f_i} - {}^G\hat{p}_{\ell|\alpha_i})+{}^Cp_I \\
J_{i\ell}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &= \frac{1}{{}^{C_\ell}\hat{z}_{f_i}}\begin{bmatrix}1&0&  \frac{-{}^{C_\ell}\hat{x}_{f_i}}{{}^{C_\ell}\hat{z}_{f_i}}\\0&1& \frac{-{}^{C_\ell}\hat{y}_{f_i}}{{}^{C_\ell}\hat{z}_{f_i}}\end{bmatrix}
\end{align} \tag{16}$
### 理想的可观性矩阵
论文中首先推导出在理想情况下的可观性矩阵，也就是在估计可观性矩阵使使用的真实状态，使用真实状态分别计算（15）（16），然后将其带入到公式（6）中，可以得到在$\ell$时刻观测到路标点$i$的一行如公式（17）所示，其中$\check{}$代表使用针织计算出来的矩阵，$\Delta{t_\ell}$代表$k$时刻到$\ell$时刻的时间间隔。其中第一段的转换参考4.1
$\begin{align}
\check{O}_{i\ell} &= H_{i\ell}\Phi_{I_\ell-1}\Phi_{I_{\ell-2}}....\Phi_{I_{k}}\\

&= \check{M}_{i\ell}\begin{bmatrix}\check{\Gamma}_{i\ell}&-I_3&-\Delta{t_\ell}&0_3&...&I_3&...&0_3 \end{bmatrix}\\
\check{M}_{i\ell}&=\check{J}_{i\ell}{}^C_IRR_{\ell}\\
\check{\Gamma}_{i\ell}&=\lfloor ({}^Gp_{f_i}-{}^Gp_{k} - {}^Gv_{k}\Delta{t_\ell}-\frac{1}{2}g\Delta{t_\ell}^2)_\times\rfloor
\end{align}\tag{17}$ 
这里，我们定义矩阵$N$
$N = \begin{bmatrix}0_3&R_kg\\I_3&-\lfloor{}^Gp_k\times\rfloor g\\
0_3 &-\lfloor{}^Gv_k\times\rfloor g\\
I_3 &-\lfloor{}^Gp_{f_1}\times\rfloor g\\
I_3 &-\lfloor{}^Gp_{f_2}\times\rfloor g\\
.&.\\
.&.\\
.&.\\
I_3 &-\lfloor{}^Gp_{f_N}\times\rfloor g\\
 \end{bmatrix}\tag{18}$
可以得到$\check{O}_{i\ell} \cdot{N} = 0_{2\times 4}$。因为对任意的$i$以及$\ell$（对可观性矩阵的所有行）该特性都满足，所以我们可以得到$\check{O}\cdot{N}=0$。另外，$N$的四列是线性独立的，这就意味着它们构建了可观测矩阵$O$零空间的四个基。
从上面可以看出，当雅克比矩阵使用真实状态计算时，其零空间的维度是4.同时，零空间的4个基也有对应的物理意义。前三列对应着状态空间中的全局位置，最后一列对应着航向角。因此，如果我们能够在计算雅克比的时候都使用真实状态，线性系统的可观测特性将于非线性系统的可观测特性一致。
### MSCKF的可观性矩阵
与2.4.1节的求解过程一致，我们可以得到使用MSCKF状态的可观测矩阵的一行为
$\begin{align}
{O}_{i\ell} &= {M}_{i\ell}\begin{bmatrix}{\Gamma}_{i\ell}+\Delta{\Gamma}_{i\ell}&-I_3&-\Delta{t_\ell}&0_3&...&I_3&...&0_3 \end{bmatrix}\\
{M}_{i\ell}&={J}_{i\ell}(\hat{x}_{\ell|\alpha_i},{}^Gp_{f_i}){}^C_IR\hat{R}_{\ell|\alpha_i}\\
{\Gamma}_{i\ell}&=\lfloor ({}^G\hat{p}_{f_i}-{}^G\hat{p}_{k|k} - {}^G\hat{v}_{k|k}\Delta{t_\ell}-\frac{1}{2}g\Delta{t_\ell}^2)_\times\rfloor\\
E^j_p&=\lfloor ({}^G\hat{p}_{j|j-1}- {}^G\hat{p}_{j|j})_\times  \rfloor\\
E^j_v&=\lfloor ({}^G\hat{v}_{j|j-1}- {}^G\hat{v}_{j|j})_\times\rfloor\\
\Delta{\Gamma}_{i\ell}&=\lfloor({}^G\hat{p}_{\ell|\ell-1} - {}^G\hat{p}_{\ell|\alpha_i})_{\times} \rfloor + \sum_{j=k+1}^{\ell-1} (E^j_p +\sum_{s=k+1}^{j}E^s_v\Delta{t})
\end{align}\tag{19}$
通过比较公式（17）和（19）可以看出，当使用msckf估计状态时，“扰动项”$\Delta{\Gamma}_{i\ell}$出现了，可以看到扰动项很复杂，且其是由修正项$\lfloor ({}^G\hat{p}_{j|j-1}- {}^G\hat{p}_{j|j})_\times  \rfloor,\lfloor ({}^G\hat{v}_{j|j-1}- {}^G\hat{v}_{j|j})_\times\rfloor$来决定的。因为修正项是随机的，所以可观性矩阵的扰动项也是随机的，并且改扰动项破坏了可观测矩阵的特殊结构。因此$O_{i\ell}\cdot{N}=0$不再成立。
在MingYangLi的另外文章中证明，可观测性矩阵的零空间的维度不在为4，而是变成了3。其零空间仅由公式（18）的前三列张成，这意味着航向角可观测了。在这种情况下，MSCKF低估了航向估计的不确定性。因为航向的不确定性影响着其他状态的不确定性，所以所有的状态的不确定性都将被低估，造成估计器的不一致性。最后造成的结果就是NESS偏大。
最后，作者指出了线性系统的不可观测特性不光影响着MSCKF算法，EKF-SLAM也有这种特性，其可观测矩阵的零空间的维度也是3。但是不同的是，EKF-SLAM的扰动项中包含了对路标点位置估计的修正项。MSCKF没有是因为其路标点位置在计算雅克比中只使用了一次。
## 如何解决不一致性
最后作者给出了如何解决不一致性的方法。按照作者的说法，这种方法非常简单，可以避免增加可观测矩阵子空间的维数（减少可观测性矩阵零空间的维数）。在公式（19）中，由于MSCKF中使用了同一状态的不同估计结果计算雅克比矩阵，导致对航向角注入了虚构的信息，导致在可观性矩阵中出现了扰动项$\Delta{\Gamma}_{i\ell}$，使得可观测矩阵的零空间维度变为3。为了移除扰动项，作者提出了一个简单的方法，即所有雅克比计算的时候对所有的状态仅使用同一个估计结果。即

- 计算IMU误差状态转移矩阵的时候使用$\Phi^*_{I_\ell} (\hat{x}_{I_{\ell+1|\ell}},\hat{x}_{I_{\ell|\ell-1}})$计算，而不是$\Phi_{I_\ell}(\hat{x}_{I_{\ell+1|\ell}},\hat{x}_{I_{\ell|\ell}})$
- 计算观测雅克比矩阵的时候

$\begin{align}
H_{f_{i\ell}}^*(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &= J_{i\ell}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}){}^C_IR\hat{R}_{\ell|\alpha_i}\\H_{I_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &=H_{f_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i})\begin{bmatrix}\lfloor ({}^G\hat{p}_{f_i} - {}^G\hat{p}_{\ell|\ell-1})_{\times} & -I_3&0_3 \end{bmatrix}\\
\end{align}$ 
经过上面的修改，可以看到扰动项已经被移除，可观测矩阵的零空间维度变为了4。因此避免了虚构信息的注入。
# 代码实现及验证
为了验证代码的有效性，这里采用openvins的sim模块生成一段时间的真值数据，imu数据以及特征点数据。最后验证程序输出状态的估计值及协方差的准确性。参考如下代码。
如下所示为估计出来的协方差以及用蒙特卡洛方法计算出来的协方差。可以看到量级基本一致。
![image.png](https://cdn.nlark.com/yuque/0/2023/png/27781308/1684665079350-d70e3eb3-5b0e-4fe9-a7f7-1c516eadf89a.png#averageHue=%232d2a26&clientId=u71187b51-dec1-4&from=paste&height=463&id=gSC6o&originHeight=509&originWidth=1622&originalType=binary&ratio=1.100000023841858&rotation=0&showTitle=false&size=178315&status=done&style=none&taskId=u02fc95cb-221d-4655-8f21-9bdcb624e45&title=&width=1474.5454225855433)

# 附录
## 证明A

## 证明B

- 证明公式（10）

$\begin{align}
{}^{I_{\ell+1}}_GR &= {}^{I_{\ell+1}}_{I_\ell}R\ {}^{I_{\ell}}_GR\\
{}^{I_{\ell+1}}_G\hat{R}(I-{\tilde{\theta}_{\ell+1}}_\times) &\approx {}^{I_{\ell+1}}_{I_\ell}\hat{R}(I-{\tilde{\theta}_{\Delta{t}}}_\times)\ {}^{I_{\ell}}_G\hat{R}(I-{\tilde{\theta}_{\ell}}_\times) \\
(I-{\tilde{\theta}_{\ell+1}}_\times) &\approx {}_{I_{\ell}}^G\hat{R}(I-{\tilde{\theta}_{\Delta{t}}}_\times)\ {}^{I_{\ell}}_G\hat{R}(I-{\tilde{\theta}_{\ell}}_\times) \\

(I-{\tilde{\theta}_{\ell+1}}_\times) &\approx (I-{{}_{I_{\ell}}^G\hat{R}\tilde{\theta}_{\Delta{t}}}_\times)\ (I-{\tilde{\theta}_{\ell}}_\times) \\


{}^G\tilde{\theta}_{\ell+1} &\simeq {}^G\tilde{\theta}_{\ell} + \hat{R}^T_\ell\tilde{\theta}_{\Delta{t}}

\end{align}$

- 证明公式（11）

$\begin{align}
{}^G{v}_{\ell+1}&={}^G{v}_{\ell}+{}^G_{I_\ell}{R}{s}_\ell + g\Delta{t}\\
{}^G\hat{v}_{\ell+1}+{}^G\tilde{v}_{\ell+1} &= {}^G\hat{v}_{\ell}+{}^G\tilde{v}_{\ell} + (I+{\tilde{\theta}_\ell}_\times){}^G_{I_\ell}\hat{R}(\hat{s}_\ell+\tilde{s}_\ell) + g\Delta{t}\\

{}^G\tilde{v}_{\ell+1} &={}^G\tilde{v}_{\ell} +{}^G_{I_\ell}\hat{R}\tilde{s}_\ell+{\tilde{\theta}_\ell}_\times {}^G_{I_\ell}\hat{R}\hat{s}_{\ell}+{\tilde{\theta}_\ell}_\times{}^G_{I_\ell}\hat{R}\tilde{s}_\ell \\
{}^G\tilde{v}_{\ell+1} &\simeq -\lfloor \hat{R}_{\ell}^T\hat{s}_{\ell} {}\times\rfloor {}^G\tilde{\theta}_\ell + {}^G\tilde{v}_{\ell}+\hat{R}_{\ell}^T\tilde{s}_\ell
\end{align}$

- 证明公式（12）

$\begin{align}
{}^G{p}_{\ell+1}&={}^G{p}_{\ell}+{}^G{v}_{\ell}\Delta{t}+{}^G_{I_\ell}{R}{y}_\ell + \frac{1}{2}g\Delta{t}^2\\
{}^G\hat{p}_{\ell+1}+{}^G\tilde{p}_{\ell+1}&={}^G\hat{p}_{\ell}+{}^G\tilde{p}_{\ell}+({}^G\hat{v}_{\ell}+{}^G\tilde{v}_{\ell})\Delta{t}+(I+{\tilde{\theta}_\ell}_\times){}^G_{I_\ell}\hat{R}(\hat{y}_\ell+\tilde{y}_\ell)  + \frac{1}{2}g\Delta{t}^2\\

{}^G\tilde{p}_{\ell+1}&={}^G\tilde{p}_{\ell}+{}^G\tilde{v}_{\ell}\Delta{t}+{\tilde{\theta}_\ell}_\times{}^G_{I_\ell}\hat{R}\hat{y}_\ell+{\tilde{\theta}_\ell}_\times{}^G_{I_\ell}\hat{R}\tilde{y}_\ell+{}^G_{I_\ell}\hat{R}\tilde{y}_\ell\\
{}^G\tilde{p}_{\ell+1} &\simeq-\lfloor \hat{R}_{\ell}^T\hat{y}_{\ell} {}\times\rfloor {}^G\tilde{\theta}_\ell + {}^G\tilde{p}_{\ell}+{}^G\tilde{v}_{\ell}\Delta{t}+\hat{R}^T_\ell\tilde{y}_\ell  
\end{align}$
## 证明C
证明公式（16）
由$z_{i\ell} =\begin{bmatrix}u&v \end{bmatrix}^T = \begin{bmatrix} \frac{{}^{C_\ell}x_{f_i}}{{}^{C_\ell}z_{f_i}} &  \frac{{}^{C_\ell}y_{f_i}}{{}^{C_\ell}z_{f_i}}\end{bmatrix}^T$可以推出 $\frac{\partial{z_{i\ell}}}{\partial{{}^{C_\ell}p_{f_i}}} = 
\begin{bmatrix} 
\frac{1}{{}^{C_\ell}z_{f_i}} &0&-\frac{{}^{C_\ell}x_{f_i}}{{}^{C_\ell}z_{f_i}^2} \\
0&\frac{1}{{}^{C_\ell}z_{f_i}} &-\frac{{}^{C_\ell}y_{f_i}}{{}^{C_\ell}z_{f_i}^2} 
\end{bmatrix}
=\frac{1}{{}^{C_\ell}z_{f_i}} \begin{bmatrix} 
1&0&-\frac{{}^{C_\ell}x_{f_i}}{{}^{C_\ell}z_{f_i}} \\
0&1 &-\frac{{}^{C_\ell}y_{f_i}}{{}^{C_\ell}z_{f_i}} 
\end{bmatrix}$。
同时，由${}^{C_\ell}p_{f_i}=\begin{bmatrix} {}^{C_\ell}\hat{x}_{f_i} &{}^{C_\ell}\hat{y}_{f_i}&{}^{C_\ell}\hat{z}_{f_i} \end{bmatrix}^T = {}^C_IR\hat{R}_{\ell|\alpha_i}( {}^G\hat{p}_{f_i} - {}^G\hat{p}_{\ell|\alpha_i})+{}^Cp_I$可以得到
$\begin{align}
{}^{C_\ell}p_{f_i} &= {}^C_IR{R}_{\ell|\alpha_i}( {}^G{p}_{f_i} - {}^G{p}_{\ell|\alpha_i})+{}^Cp_I\\
&\simeq {}^C_IR\hat{R}_{\ell|\alpha_i}(I - {\tilde{\theta_{\ell}}}_\times)( {}^G\hat{p}_{f_i} + {}^G\tilde{p}_{f_i}  - {}^G\hat{p}_{\ell|\alpha_i} - {}^G\tilde{p}_{\ell})+{}^Cp_I \\
&\simeq \{{}^C_IR\hat{R}_{\ell|\alpha_i}( {}^G\hat{p}_{f_i}  - {}^G\hat{p}_{\ell|\alpha_i})+{}^Cp_I\} 
+{}^C_IR\hat{R}_{\ell|\alpha_i}({}^G\hat{p}_{f_i}  - {}^G\hat{p}_{\ell|\alpha_i})_\times \tilde{\theta}_\ell 
+  {}^C_IR\hat{R}_{\ell|\alpha_i}( {}^G\tilde{p}_{f_i} - {}^G\tilde{p}_{\ell})\\
\end{align}$
可以推出
$\begin{align}
H_{f_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &= J_{i\ell}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}){}^C_IR\hat{R}_{\ell|\alpha_i}\\H_{I_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i}) &=H_{f_{i\ell}}(\hat{\pi}_{\ell|\alpha_i},{}^G\hat{p}_{f_i})\begin{bmatrix}\lfloor ({}^G\hat{p}_{f_i} - {}^G\hat{p}_{\ell|\alpha_i})_{\times} & -I_3&0_3 \end{bmatrix}\\
\end{align}$
## 证明D
15维状态转移方程推导，参考。离线推导参考。这里先对 $n_{wg}(t)$$n_{wa}(t)$$n_r(t)$$n_a(t)$进行定义，因为其为高斯白噪声，所以可以得到如下的定义
$\begin{align}
 &E(n_{r}(t)) = 0, E(n_{r}(t)n_{r}(\tau)^T) = Q_{r} \delta{(t-\tau)} \\
 &E(n_{a}(t)) = 0, E(n_{a}(t)n_{a}(\tau)^T) = Q_{a} \delta{(t-\tau)} \\
&E(n_{wg}(t)) = 0, E(n_{wg}(t)n_{wg}(\tau)^T) = Q_{wg} \delta{(t-\tau)} \\
 &E(n_{wa}(t)) = 0, E(n_{wa}(t)n_{wa}(\tau)^T) = Q_{wa} \delta{(t-\tau)} \\
 


\end{align}$
对高斯白噪声的积分进行离散化，同时定义$\Delta{t} = t_{\ell+1} - t_\ell$。可以得到
$\begin{align}

 &n_{rd}(t_\ell) =\int_{t_{\ell}}^{t_{\ell+1}} n_{rc}(\tau)d\tau,E(n_{rd}(t_\ell)) = 0, E(n_{rd}(t_\ell)n_{rd}(t_\ell)^T) = Q_{r}\Delta{t} \\
 &n_{ad}(t_\ell) =\int_{t_{\ell}}^{t_{\ell+1}} n_{ac}(\tau)d\tau,E(n_{ad}(t_\ell)) = 0, E(n_{ad}(t_\ell)n_{ad}(t_\ell)^T) = Q_{a}\Delta{t} \\
&n_{wgd}(t_\ell) =\int_{t_{\ell}}^{t_{\ell+1}} n_{wg}(\tau)d\tau,E(n_{wgd}(t_\ell)) = 0, E(n_{wgd}(t_\ell)n_{wgd}(t_\ell)^T) = Q_{wg}\Delta{t} \\
 &n_{wad}(t_\ell) =\int_{t_{\ell}}^{t_{\ell+1}} n_{wa}(\tau)d\tau,E(n_{wad}(t_\ell)) = 0, E(n_{wad}(t_\ell)n_{wad}(t_\ell)^T) = Q_{wa}\Delta{t} \\


\end{align}$且$E(\begin{bmatrix}n_{rd}\\n_{ad}\\ n_{wgd} \\ n_{wad} \end{bmatrix}) = 0,E(\begin{bmatrix}n_{rd}\\n_{ad}\\ n_{wgd} \\ n_{wad} \end{bmatrix} *\begin{bmatrix}n_{rd}\\n_{ad}\\ n_{wgd} \\ n_{wad} \end{bmatrix}^T) = \begin{bmatrix} Q_r\Delta{t} & 0& 0 &0\\ 0&Q_a\Delta{t}&0&0\\ 0&0&Q_{wg}\Delta{t}&0\\0&0&0&Q_{wa}\Delta{t}\end{bmatrix}$
### $\tilde{\theta}_{\Delta_\ell}$的推导
$\tilde{\theta}_{\Delta_\ell}$的推导与论文的思路一致，都是从微分方程出发，推导出$\dot{\tilde{\theta}}_{\Delta_\ell}$.首先推导出${}_G^I\dot{R}$，定义$\lim_{\Delta{t}\to{0}} \frac{{}^{I_\ell}\theta}{\Delta{t}} = {}^{I_\ell}\omega$，则 $I+[{}^{I_\ell}\theta_\times] \approx {}_{I_{\ell+\Delta{t}}}^{I_\ell}R$，可以得到如下的导数公式。其中${}^{I_\ell}\theta$代表坐标系$I_{\ell}$坐标系绕$\frac{{}^{I_\ell}\theta}{\|{}^{I_\ell}\theta\|}$轴旋转$\|{}^{I_\ell}\theta\|$角度后与坐标系$I_{\ell+\Delta{t}}$重合。
$\begin{align}
{}_G^{I_\ell}\dot{R} &= \lim_{\Delta{t}\to 0} \frac{{}^{I_{\ell+\Delta{t}}}_GR - {}^{I_\ell}_{G}R}{\Delta{t}} \\
&\approx  \lim_{\Delta{t}\to 0} \frac{{}^{I_{\ell+\Delta{t}}}_{I_\ell}R {}^{I_{\ell}}_GR - {}^{I_\ell}_{G}R}{\Delta{t}}\\
&\approx  \lim_{\Delta{t}\to 0} \frac{(I_3 - [{}^{I_\ell}\theta_\times]) {}^{I_{\ell}}_GR - {}^{I_\ell}_{G}R}{\Delta{t}}\\
&\approx  \lim_{\Delta{t}\to 0} \frac{- [{}^{I_\ell}\theta_\times] {}^{I_{\ell}}_G R}{\Delta{t}}\\
&\approx  - [{}^{I_\ell}\omega_\times] {}^{I_{\ell}}_G R
\end{align}$
可以得到${}^{I_\ell}_G\dot{R} = -[{}^{I_\ell}\omega_\times] {}^{I_\ell}_GR$，$\dot{{}^{I_\ell}_G\hat{R} }= -[{}^{I_\ell}\hat{\omega}_\times] {}^{I_\ell}_G\hat{R}$，然后对${}^{I_\ell}_GR = {}^{I_\ell}_G\hat{R}(I_3 - [{{}^G\tilde{\theta}}_\times])$两面同时求导
$\begin{align}
\frac{\partial{{}^{I_\ell}_GR}}{\partial{t}} &\simeq \frac{\partial{{}^{I_\ell}_G\hat{R}(I_3 - [{{}^G\tilde{\theta}}_\times])}}{\partial{t}} \\
 -[{}^{I_\ell}\omega_\times] {}^{I_\ell}_GR &\simeq -[{}^{I_\ell}\hat{\omega}_\times] {}^{I_\ell}_G\hat{R}(I_3 - [{{}^G\tilde{\theta}}_\times]) -  {}^{I_\ell}_G\hat{R}[{{}^G\dot{\tilde{\theta}}}_\times] \\

 -[{}^{I_\ell}\omega_\times] {}^{I_\ell}_G\hat{R}(I_3 - [{{}^G\tilde{\theta}}_\times]) &\simeq -[{}^{I_\ell}\hat{\omega}_\times] {}^{I_\ell}_G\hat{R}(I_3 - [{{}^G\tilde{\theta}}_\times]) -  {}^{I_\ell}_G\hat{R}[{{}^G\dot{\tilde{\theta}}}_\times] \\

[{}^{I_\ell}\tilde{\omega}_\times] {}^{I_\ell}_G\hat{R}(I_3 - [{{}^G\tilde{\theta}}_\times]) &\simeq {}^{I_\ell}_G\hat{R}[{{}^G\dot{\tilde{\theta}}}_\times] \\

[{{}^G\dot{\tilde{\theta}}}_\times] &\simeq {}^{I_\ell}_G\hat{R}^T[{}^{I_\ell}\tilde{\omega}_\times] {}^{I_\ell}_G\hat{R}(I_3 - [{{}^G\tilde{\theta}}_\times]) \\
[{{}^G\dot{\tilde{\theta}}}_\times] &\simeq [{}^{I_\ell}_G\hat{R}^T{}^{I_\ell}\tilde{\omega}_\times] (I_3 - [{{}^G\tilde{\theta}}_\times]) \\
\end{align}$
去除二阶以上的部分，可以得到$[{{}^G\dot{\tilde{\theta}}}_\times] \simeq [{}^{I_\ell}_G\hat{R}^T{}^{I_\ell}\tilde{\omega}_\times]$，进而得到${{}^G\dot{\tilde{\theta}}} \simeq {}^{I_\ell}_G\hat{R}^T{}^{I_\ell}\tilde{\omega}$。对其进行积分，可以得到${{}^G\tilde{\theta}}_{t_\ell+\Delta{t}} \simeq {{}^G\tilde{\theta}}_{t_\ell} + \int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T{}^{I_{\tau}}\tilde{\omega} d\tau$。同样，由公式（7）我们可以得到${}^I\omega_t  = {}^I\omega_{m_t} - \hat{b_{g_t}} - \tilde{b_{g_t}}- n_{r_t}$，即${}^{I_\tau}\tilde{\omega} = - \tilde{b_{g_t}} - n_{{r_t}}$。同时，由于论文中对偏置建模为随机游走，可以得到$\tilde{b_{g_\tau}} = \tilde{b_{g_{t_\ell}}} + \int_{t_\ell}^{\tau} n_{wg_s} ds$。所以对${{}^G\tilde{\theta}}_{\ell+\Delta{t}}$进行展开，并将${}^{I_\ell}_G\hat{R}$记为$\hat{R}_{\ell|\ell}$可以得到
$\begin{align}
{{}^G\tilde{\theta}}_{t_\ell+\Delta{t}} &\simeq {{}^G\tilde{\theta}}_{t_\ell} + \int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T{}^{I_{\tau}}\tilde{\omega} d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} + \int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T (- \tilde{b_{g_\tau}} - n_{r_\tau}) d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} + \int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T (- (\tilde{b_{g_{t_\ell}}} + \int_{t_\ell}^{\tau} n_{wg_s} ds)- n_{r_\tau}) d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} -\int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^Td\tau \tilde{b_{g_{t_\ell}}}-\int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T  (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_\ell+\Delta{t}}{}_{I_\tau}^{I_{\ell}}\hat{R}d\tau \tilde{b_{g_{t_\ell}}}-\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_\ell+\Delta{t}}{}_{I_\tau}^{I_{\ell}}\hat{R}  (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau \\
\end{align}$
这里可以得到$n_{\theta_{\ell+1}} = -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_{\ell+1}}{}_{I_\tau}^{I_{\ell}}\hat{R}  (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau$. 这里对噪声进行离散化，参考严恭敏老师《惯性仪器测试与数据分析》的第九章。这里保留二次积分项，且假设${}_{I_\tau}^{I_{\ell}}\hat{R} = {}^{I_{\ell}}_{I_{\ell}}\hat{R} = I_3$，可以得到$n_{\theta_{\ell+1}} \simeq -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_{\ell+1}} (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau$。对其进行离散化可以得到$n_{\theta_{\ell+1}} \simeq -\hat{R}_{\ell|\ell}^T(n_{rd} + n_{wgd}\Delta{t})$。
### $\tilde{s}_{\ell}$的推导
由$\hat{s}_\ell = \int_{t_\ell}^{t_{\ell+1}} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})d\tau$可以得到
$\begin{align}
\tilde{s}_\ell &= s_\ell - \hat{s}_\ell\\
\tilde{s}_\ell &= \int_{t_\ell}^{t_{\ell+1}} {}^{I_\ell}_{I_\tau}{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau}- \tilde{b}_{a_\tau}-n_{a_\tau})d\tau - \int_{t_\ell}^{t_{\ell+1}} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})d\tau \\

&= \int_{t_\ell}^{t_{\ell+1}} (I_3 - [{\theta_{\Delta{\tau}}}_\times]){}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau}- \tilde{b}_{a_\tau}-n_{a_\tau})d\tau - \int_{t_\ell}^{t_{\ell+1}} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})d\tau \\

&= \int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}(-\tilde{b}_{a_\tau}-n_{a_\tau}) d\tau -\int_{t_\ell}^{t_{\ell+1}} [{\theta_{\Delta{\tau}}}_\times]{}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau}- \tilde{b}_{a_\tau}-n_{a_\tau})d\tau  \\

&\simeq \int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}(-\tilde{b}_{a_\tau}-n_{a_\tau}) d\tau -\int_{t_\ell}^{t_{\ell+1}} [{\theta_{\Delta{\tau}}}_\times]{}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})d\tau  \\

&= \int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}(-\tilde{b}_{a_\tau}-n_{a_\tau}) d\tau -\int_{t_\ell}^{t_{\ell+1}} [{\theta_{\Delta{\tau}}}_\times]{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}d\tau  \\

&= \int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}(-\tilde{b}_{a_\tau}-n_{a_\tau}) d\tau +\int_{t_\ell}^{t_{\ell+1}} [{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\theta_{\Delta{\tau}}d\tau  \\

&= \int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}(-\tilde{b}_{a_\tau}-n_{a_\tau}) d\tau +\int_{t_\ell}^{t_{\ell+1}} \hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T\theta_{\Delta{\tau}}d\tau  \\
\end{align}$
这里$\theta_{\Delta_\tau}=\int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}ds \cdot \tilde{b}_{g_{t_\ell}} + n_{\theta_\tau}$，$n_{\theta_\tau} = \int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}(\int_{t_\ell}^{s}n_{wgt}dt+n_{rs})ds$,$\tilde{b}_{a_\tau} = \tilde{b}_{a_{t_\ell}} + \int_{t_\ell}^{\tau} n_{wa_s}ds$。将其带入上式,可以得到
$\begin{align}
\tilde{s}_\ell &= \int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}(-\tilde{b}_{a_{t_\ell}} - \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau}) d\tau +\int_{t_\ell}^{t_{\ell+1}} \hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T(\int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}ds \cdot \tilde{b}_{g_{t_\ell}} + n_{\theta_\tau})d\tau\\
&= -\int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}d\tau \tilde{b}_{a_{t_\ell}} + \int_{t_\ell}^{t_{\ell+1}} \left(\hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}ds\right) d\tau \tilde{b}_{g_{t_\ell}} + \int_{t_\ell}^{t_{\ell+1}} \left( \hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T n_{\theta_\tau}+ {}^{I_\ell}_{I_\tau}\hat{R}(- \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau})\right)d\tau
\end{align}$
这里可以得到$n_{v_{\ell+1}} =\hat{R}_{\ell|\ell}^T \left( \int_{t_\ell}^{t_{\ell+1}} \left(\hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T n_{\theta_\tau}+ {}^{I_\ell}_{I_\tau}\hat{R}(- \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau})\right)d\tau\right)$。采用和4.4.1同样的策略，仅保留二次积分项，且且假设${}_{I_\tau}^{I_{\ell}}\hat{R} = {}^{I_{\ell}}_{I_{\ell}}\hat{R} = I_3$，可以得到$n_{v_{\ell+1}} \simeq \hat{R}_{\ell|\ell}^T( \int_{t_\ell}^{t_{\ell+1}}  \left(\int_{t_\ell}^{\tau} [{\hat{a}}_\times]n_{rs}ds+ (- \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau})\right)d\tau)$。所以，可以得到$n_{v_{\ell+1}} \simeq \hat{R}_{\ell|\ell}^T([{}^{I_\ell}\hat{a}_\times]\Delta{t}n_{rd} - n_{ad}-\Delta{t}n_{wa})$。
### $\tilde{y}_{\ell}$的推导
由$\hat{y}_\ell = \int_{t_\ell}^{t_{\ell+1}}  \int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})   d\tau ds$可以推出
$\begin{align}
\tilde{y}_\ell &= y_\ell - \hat{y}_\ell\\

 &= \int_{t_\ell}^{t_{\ell+1}}  \int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau}- \tilde{b}_{a_\tau} - n_{a_\tau})   d\tau ds - \int_{t_\ell}^{t_{\ell+1}}  \int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})   d\tau ds \\

 &= \int_{t_\ell}^{t_{\ell+1}}  (\int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau}- \tilde{b}_{a_\tau} - n_{a_\tau})   d\tau  - \int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})   d\tau )ds \\

 &= \int_{t_\ell}^{t_{\ell+1}} \tilde{s}_s ds \\



\end{align}$
这里可以得到$n_{p_{\ell+1}} = \int_{t_\ell}^{t_{\ell+1}} n_{v_{s}} ds$。同理，省略三次及以上积分项，可以得到$n_{p_{\ell+1}} \simeq -\hat{R}_{\ell|\ell}^T\Delta{t}n_{ad}$.
### 15维状态向量的状态转移方程
$\begin{align}
{{}^G\tilde{\theta}}_{t_\ell+\Delta{t}} &\simeq {{}^G\tilde{\theta}}_{t_\ell} + \int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T{}^{I_{\tau}}\tilde{\omega} d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} + \int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T (- \tilde{b_{g_\tau}} - n_{r_\tau}) d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} + \int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T (- (\tilde{b_{g_{t_\ell}}} + \int_{t_\ell}^{\tau} n_{wg_s} ds)- n_{r_\tau}) d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} -\int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^Td\tau \tilde{b_{g_{t_\ell}}}-\int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_G\hat{R}^T  (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau \\

&\simeq {{}^G\tilde{\theta}}_{t_\ell} -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_{I_{t_\ell}}\hat{R}^Td\tau \tilde{b_{g_{t_\ell}}}-\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_\ell+\Delta{t}}{}^{I_\tau}_{I_{t_\ell}}\hat{R}^T  (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau \\
\end{align}$
$\begin{align}
\tilde{s}_\ell &= \int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}(-\tilde{b}_{a_{t_\ell}} - \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau}) d\tau +\int_{t_\ell}^{t_{\ell+1}} \hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T(\int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}ds \cdot \tilde{b}_{g_{t_\ell}} + n_{\theta_\tau})d\tau\\
&= -\int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}d\tau \tilde{b}_{a_{t_\ell}} + \int_{t_\ell}^{t_{\ell+1}} \left(\hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}ds\right) d\tau \tilde{b}_{g_{t_\ell}} + \int_{t_\ell}^{t_{\ell+1}} \left( \hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T n_{\theta_\tau}+ {}^{I_\ell}_{I_\tau}\hat{R}(- \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau})\right)d\tau
\end{align}$
$\begin{align}
\tilde{y}_\ell &= y_\ell - \hat{y}_\ell\\

 &= \int_{t_\ell}^{t_{\ell+1}}  \int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau}- \tilde{b}_{a_\tau} - n_{a_\tau})   d\tau ds - \int_{t_\ell}^{t_{\ell+1}}  \int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})   d\tau ds \\

 &= \int_{t_\ell}^{t_{\ell+1}}  (\int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau}- \tilde{b}_{a_\tau} - n_{a_\tau})   d\tau  - \int_{t_\ell}^{s} {}^{I_\ell}_{I_\tau}\hat{R}({}^{I_\tau}a_m - \hat{b}_{a_\tau})   d\tau )ds \\

 &= \int_{t_\ell}^{t_{\ell+1}} \tilde{s}_s ds \\



\end{align}$
根据公式 10,11,12 以及 4.4.1-4.4.3章节,可以推出状态转移方程为
$\Phi_{I_k} = \begin{bmatrix} I_3&0_3&0_3&\Phi_{qb_g}&0_3\\ \Phi_{pq}&I_3&\Delta{t}I_3&\Phi_{pb_g}&\Phi_{pb_a}\\
\Phi_{vq}&0_3&I_3&\Phi_{vb_g}&\Phi_{vb_a}\\
0_3&0_3&0_3&I_3&0_3\\
0_3&0_3&0_3&0_3&I_3\\
 \end{bmatrix}$
其中
$\begin{align}
\Phi_{qb_g} & =  -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_{\ell+1}}{}_{I_\tau}^{I_\ell}\hat{R}d\tau\\
\Phi_{pq}&=-\lfloor({}^G\hat{p}_{\ell+1}-{}^G\hat{p}_{\ell}-{}^G\hat{v}_{\ell}\Delta{t}-\frac{1}{2}g\Delta{t}^2)_{\times} \rfloor \\

\Phi_{pb_g} &= \int_{t_\ell}^{t_{\ell+1}}\int_{t_\ell}^{w} \left([\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}ds \right) d\tau dw\\
\Phi_{pb_a} & = -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_{\ell+1}}\int_{t_\ell}^{s}{}^{I_\ell}_{I_\tau}\hat{R}d\tau ds \\
\Phi_{vq}&=-\lfloor({}^G\hat{v}_{\ell+1}-{}^G\hat{v}_{\ell}-g\Delta{t})_{\times} \rfloor \\

\Phi_{vb_g} &= \int_{t_\ell}^{t_{\ell+1}}\left([\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{\tau} {}^{I_\ell}_{I_s}\hat{R}ds \right)d\tau \\
\Phi_{vb_a} &=-\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_{\ell+1}}{}^{I_\ell}_{I_\tau}\hat{R}^Td\tau \\

\end{align}$
### 误差状态转移方程离散实现
使用梯形积分对4.4.4节积分形式的误差状态转移方程进行求解。

### 误差状态转移方程总结
这里可以得到$n_{\theta_{\ell+1}} = -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_{\ell+1}}{}_{I_\tau}^{I_{\ell}}\hat{R}  (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau$. 这里对噪声进行离散化，参考严恭敏老师《惯性仪器测试与数据分析》的第九章。这里保留二次积分项，且假设${}_{I_\tau}^{I_{\ell}}\hat{R} = {}^{I_{\ell}}_{I_{\ell}}\hat{R} = I_3$，可以得到$n_{\theta_{\ell+1}} \simeq -\hat{R}_{\ell|\ell}^T\int_{t_\ell}^{t_{\ell+1}} (\int_{t_\ell}^{\tau} n_{wg_s} ds+ n_{r_\tau}) d\tau$。对其进行离散化可以得到$n_{\theta_{\ell+1}} \simeq -\hat{R}_{\ell|\ell}^T(n_{rd} + n_{wgd}\Delta{t})$。
这里可以得到$n_{p_{\ell+1}} = \int_{t_\ell}^{t_{\ell+1}} n_{v_{s}} ds$。同理，省略三次及以上积分项，可以得到$n_{p_{\ell+1}} \simeq -\hat{R}_{\ell|\ell}^T\Delta{t}n_{ad}$.
这里可以得到$n_{v_{\ell+1}} =\hat{R}_{\ell|\ell}^T \left( \int_{t_\ell}^{t_{\ell+1}} \left(\hat{R}_{\ell|\ell}[\hat{R}_{\ell|\ell}^T{{}^{I_\ell}_{I_\tau}\hat{R}{}^{I_\tau}\hat{a}}_\times]\hat{R}_{\ell|\ell}^T n_{\theta_\tau}+ {}^{I_\ell}_{I_\tau}\hat{R}(- \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau})\right)d\tau\right)$。采用和4.4.1同样的策略，仅保留二次积分项，且且假设${}_{I_\tau}^{I_{\ell}}\hat{R} = {}^{I_{\ell}}_{I_{\ell}}\hat{R} = I_3$，可以得到$n_{v_{\ell+1}} \simeq \hat{R}_{\ell|\ell}^T( \int_{t_\ell}^{t_{\ell+1}}  \left(\int_{t_\ell}^{\tau} [{\hat{a}}_\times]n_{rs}ds+ (- \int_{t_\ell}^{\tau} n_{wa_s}ds-n_{a_\tau})\right)d\tau)$。所以，可以得到$n_{v_{\ell+1}} \simeq \hat{R}_{\ell|\ell}^T([{}^{I_\ell}\hat{a}_\times]\Delta{t}n_{rd} - n_{ad}-\Delta{t}n_{wa})$。

$\underbrace{\begin{bmatrix} 
{}^G\tilde{\theta}_{\ell+1}  \\ {}^G\tilde{p}_{\ell+1} \\ {}^G\tilde{v}_{\ell+1} \\ \tilde{b}_{g_{\ell+1}} \\ \tilde{b}_{a_{\ell+1}}\end{bmatrix}}_{\tilde{x}_{I_{\ell+1}} } = 

\underbrace{\begin{bmatrix} I_3&0_3&0_3&\Phi_{qb_g}&0_3\\ \Phi_{pq}&I_3&\Delta{t}I_3&\Phi_{pb_g}&\Phi_{pb_a}\\
\Phi_{vq}&0_3&I_3&\Phi_{vb_g}&\Phi_{vb_a}\\
0_3&0_3&0_3&I_3&0_3\\
0_3&0_3&0_3&0_3&I_3\\ \end{bmatrix}}_{\Phi_{I_{\ell}}}

\underbrace{\begin{bmatrix} {}^G\tilde{\theta}_{\ell}  \\ {}^G\tilde{p}_{\ell} \\ {}^G\tilde{v}_{\ell} \\ \tilde{b}_{g_{\ell}} \\ \tilde{b}_{a_{\ell}}\end{bmatrix}}_{\tilde{x}_{I_{\ell}}}
+
\underbrace{\begin{bmatrix}n_{\theta_{\ell}} \\ n_{p_{\ell}}  \\ n_{v_{\ell}} \\n_{{b_g}_{\ell}} \\n_{{b_a}_{\ell}} \end{bmatrix}}_{w_{d_\ell}}$
其中$\Phi_{qb_g}$,$\Phi_{pq}$,$\Phi_{pb_g}$,$\Phi_{pb_a}$,$\Phi_{vq}$,$\Phi_{vb_g}$,$\Phi_{vb_a}$由 4.4.5 所定义.$n_{\theta_{\ell}} \  n_{p_{\ell}} \  n_{v_{\ell}} \  n_{{b_g}_{\ell}} \ n_{{b_a}_{\ell}}$为高斯噪声, 其定义如下
$\begin{bmatrix}n_{\theta_{\ell}} \\ n_{p_{\ell}}  \\ n_{v_{\ell}} \\n_{{b_g}_{\ell}} \\n_{{b_a}_{\ell}} \end{bmatrix} = \underbrace{
\begin{bmatrix} 
-\hat{R}_{\ell|\ell}^T & 0& -\hat{R}_{\ell|\ell}\Delta{t}&0 \\
0&-\hat{R}_{\ell|\ell}^T\Delta{t}&0&0\\
\hat{R}_{\ell|\ell}^T[{}^{I_\ell}\hat{a}_\times]\Delta{t}&-\hat{R}_{\ell|\ell}^T&0&-\hat{R}_{\ell|\ell}^T\Delta{t}\\

0&0&I&0\\
0&0&0&I
\end{bmatrix}}_{G_\ell} *\begin{bmatrix}n_{rd}\\n_{ad}\\ n_{wgd} \\ n_{wad} \end{bmatrix}$
