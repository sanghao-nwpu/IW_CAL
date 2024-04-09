# DR算法
记录IMU为$b$系， 车体为$v$系，导航坐标系为$n$系，忽略地球自转，则$n$系与惯性坐标系$i$系以及世界坐标系$w$系重合。
以$b$系为定位中心，以b系在$n$系下的角度$\bm{R}^n_b$,速度$\bm{v}^n_b$,加速度计偏置$\bm{b}_a$为状态变量，则系统状态方程可以写为：
$$
\begin{equation}
\begin{bmatrix} 
\dot{\bm{R}}^n_b \\    
\dot{\bm{v}}^n_b \\
\dot{\bm{b}}_a
\end{bmatrix} 
= 
\begin{bmatrix} 
\bm{R}_b^n \cdot \bm{\omega}^b_{nb} \\    
\bm{R}_b^n (\bm{f^b_{nb}} - \bm{b_a}) + \bm{g}^n \\
\bm{0} 
\end{bmatrix} 
\end{equation}
$$

在公式中，$\bm{R}_b^n$为$b$系在$n$系下的旋转矩阵，$\bm{\omega}^b_{nb}$为$b$系在$n$系下的角速度，$\bm{f^b_{nb}}$为加速度计读数，$\bm{g}^n$为重力加速度。

同时，将上边的公式改为误差形式，参考高博的书《自动驾驶中的SLAM技术》，可以将上边的系统状态方程改写为：

$$
\begin{equation}
\begin{bmatrix} 
\delta \dot{\bm{\theta}} \\    
\delta \dot{\bm{v}} \\
\delta \dot{\bm{b}}_a
\end{bmatrix} 
= 
\begin{bmatrix} 
\bm{R}_b^n \cdot \bm{\omega}^b_{nb} \\    
\bm{R}_b^n (\bm{f^b_{nb}} - \bm{b_a}) + \bm{g}^n \\
\bm{0} 
\end{bmatrix} 
\end{equation}
$$


其中，为$b$系在$n$系下的旋转误差，$\bm{e}_v^n_b$为$b$系在$n$系下的速度误差，$\bm{e}_b_a$为加速度计偏置误差。