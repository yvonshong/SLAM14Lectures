# 深蓝学院SLAM第2讲作业  yvonshong

# Eigen

线性方程 $Ax=b$ , 在 $A$ 为方阵的前提下

## 1. 在什么条件下，$x$ 有解且唯一 

$||A||\neq 0$ 并且 $A$ 为满秩方阵

## 2. 高斯消元法原理

高斯消元法相当于对方程组进行线性组合，组合时某一行消去其他所有的未知数，只剩下一个未知数，便能直接求解，然后反向代入其他方程式，求解其他未知数。

## 3. QR 分解

$[a_1,a_2 ,\cdots  ,a_n]=[q_1, q_2 ,\cdots,q_n] \begin{bmatrix} r_{11} && r_{12} && \cdots && r_{1n} \\   0 && r_{22} && \cdots && r_{2n} \\  \vdots  && \vdots && \ddots && \vdots \\ 0 && 0 && \cdots && r_{nn} \\     \end{bmatrix} $

实际含义是已知矩阵有n个线性独立的行向量，利用QR分解求得此矩阵如何由n个单位正交基变换而来。

### a. Gram-Schimit 正交化

正交化：先求得与x

令 $y_1 =x_1 $

$y_2=x_2 -\frac{y^T_1 x_2}{y_1^T y_1} y_1$ ，即 将 $x_2$ 去除 $x_2$ 正交投影到 $y_1$ 子空间的分量就得到 $y_2 $ 

$y_3 =x_3 -  -\frac{y^T_1 x_3}{y_1^T y_1} y_1 -\frac{y^T_2 x_3}{y_2^T y_2} y_2$ 

如此迭代下去，得到正交的 $\{y_1,y_2 ,\cdots,y_n\}$

反解：

$x_1 =y_1$

$x_2 =\frac{y^T_1 x_2}{y^T_1y_1}y_1+y_2 $

$x_3 =\frac{y^T_1 x_3}{y^T_1y_1}y_1+\frac{y^T_2 x_3}{y^T_2 y_2}y_2+y_3 $

$\cdots$

单位化：$q_i =\frac{y_i}{||y_i||}$

$x_1 =r_{11} q_1$

$x_2 =r_{12}q_1 +r_{22} q_2$

$x_3=r_{13} q_1+r_{23} q_2 +r_{33}q_3$

$\cdots$

$A=[x_1,x_2 ,\cdots  ,x_n]=[q_1, q_2 ,\cdots,q_n] \begin{bmatrix} r_{11} && r_{12} && \cdots && r_{1n} \\   0 && r_{22} && \cdots && r_{2n} \\  \vdots  && \vdots && \ddots && \vdots \\ 0 && 0 && \cdots && r_{nn} \\     \end{bmatrix} $

$=[q_1, q_2 ,\cdots,q_n]  \begin{bmatrix} q_1^T x_1  &&  q_1^T x_2 && \cdots &&  q_1^T x_n \\   0 &&  q_2^T x_2 && \cdots &&  q_2^T x_n \\  \vdots  && \vdots && \ddots && \vdots \\ 0 && 0 && \cdots &&  q_n^T x_n \\     \end{bmatrix} $

$=QR $

### b. Givens 旋转

令 $x$ 逆时针旋转 $\theta $ 弧度，旋转矩阵为

$Q= \begin{bmatrix}  cos(\theta) && -sin(\theta)\\   sin(\theta) && cos(\theta)     \end{bmatrix} = \frac{1}{\sqrt{x_1^2 +x_2^2}} \begin{bmatrix}  x_1 && x_2 \\  -x_2&&  x_1    \end{bmatrix}$

在向量空间中任取两个坐标轴，假设是第i个和第j个轴，$i \neq j$ ，Given 旋转就是在这两个坐标轴所张开的平面上旋转。

$Q_{ji} = \begin{bmatrix} 1 && \cdots&& 0&&\cdots &&0&& \cdots&&0\\           \vdots &&\ddots && \vdots && &&\vdots &&&&\vdots \\             1 && \cdots&& c&&\cdots &&-s&& \cdots&&0\\                           \vdots && && \vdots &&\ddots &&\vdots &&&&\vdots \\              1 && \cdots&& -s&&\cdots &&c&& \cdots&&0\\            \vdots &&  && \vdots && &&\vdots && \ddots &&\vdots \\                              1 && \cdots&& 0&&\cdots &&0&& \cdots&&1                      \end{bmatrix} $

$q_{ii}=c, q_{jj}=c, q_{ij}=-s , q_{i}=s$ 来替代单位矩阵

$c=\frac{x_i}{\sqrt{x_i^2 +x_j^2}}, s=-\frac{x_j}{\sqrt{x_i^2 +x_j^2}}$

$ \begin{bmatrix} 1 && \cdots&& 0&&\cdots &&0&& \cdots&&0\\           \vdots &&\ddots && \vdots && &&\vdots &&&&\vdots \\             1 && \cdots&& c&&\cdots &&-s&& \cdots&&0\\                           \vdots && && \vdots &&\ddots &&\vdots &&&&\vdots \\              1 && \cdots&& -s&&\cdots &&c&& \cdots&&0\\            \vdots &&  && \vdots && &&\vdots && \ddots &&\vdots \\                              1 && \cdots&& 0&&\cdots &&0&& \cdots&&1                      \end{bmatrix}  \begin{bmatrix}x_1 \\   \vdots \\ x_i \\ \vdots \\x_j \\ \vdots \\x_n\\  \end{bmatrix} = \begin{bmatrix}x_1 \\   \vdots \\ \sqrt{x_i^2 +x_j ^2} \\ \vdots \\0 \\ \vdots \\x_n\\  \end{bmatrix} $

设计 $n$ 维下的特殊的 Given 旋转，可以选择消除向量中的任何一个元，施行一连串的 Given 旋转就能消除到只剩一个未知数直接求解。

$Q_{ni} \cdots Q_{i+1,i}  Q_{i-1,i} \cdots Q_{1i} x = ||x||e_i $

$Q_{ni} \cdots Q_{i+1,i}  Q_{i-1,i} \cdots Q_{1i} A = R $

由于是依次消元，所以R是一个上三角矩阵

$A= (Q_{ni}^{-1} \cdots Q_{i+1,i}^{-1}  Q_{i-1,i}^{-1} \cdots Q_{1i} ^{-1})R $

$Q_{ij}$ 旋转矩阵为正交矩阵，多个的积也是正交矩阵

$A=QR$

### c. Householder 变换

Householder 变换矩阵

$H=I-2vv^T $

几何意义为基本反射变换，单位向量 v 是反射超平面的法向量，并且是对称矩阵，正交矩阵

$H^T = (I-2vv^T )^T =I^T -2vv^T =H $

$H^T H=(I-2vv^T )(I-2vv^T )=I-4vv^T +4vv^Tvv^T=I-4vv^T+4vv^T =H^2=I$

设 x 为一n维非零实向量，$\sigma  = ||x||$

法向量 $v= \frac{x\pm \sigma  e_1}{||x\pm \sigma e_1||}, e_1=[1,0,...,0]^T$

$Hx=(I-2vv^T )x=x-2(v^T v)v=x-2\frac{(x\pm \sigma e_1)^T x}{||x\pm \sigma e_1||^2} (x\pm \sigma e_1)$

$x^T x=||x||^2=\sigma ^2$

$(x\pm \sigma e_1 )^T (x\pm \sigma e_2)=x^T x +\sigma ^2 \pm 2\sigma e_1^T x=2x^T x \pm 2\sigma e_1^T x = 2x^T x \pm 2\sigma  e_1^T x=2(x\pm \sigma e_1 )^T x  $



设计的特殊的Householder矩阵将向量x反射到向量空间的第1个轴上，其他维度都是0。

$Hx=x-(x\pm \sigma e_1) = \mp\sigma  e_1 =\mp ||x||e_1 = \begin{bmatrix} \mp ||x|| \\0\\ \vdots \\0  \end{bmatrix} $

$H_1 A=H_1[a_1,a_2,\cdots ,a_n]=[H_1 a_1,H_1 a_2,\cdots ,H_1 a_n]= \begin{bmatrix} r_{11} && r_{12 }&& \cdots &&r_{1n} \\ 0&& * && \cdots &&* \\\vdots &&\vdots  &&\ddots  &&\vdots \\0&&* &&\cdots &&*  \end{bmatrix}$

右下角为A11的余子式

$H_2 H_1 A= \begin{bmatrix} r_{11} &&r_1^T\\ 0 &&\hat{H}_2 A_2  \end{bmatrix}$

如果 $m>n$ :

$H_n \cdots H_2 H_1  A = \begin{bmatrix} R\\0 \end{bmatrix}$

依次消去，所以R是上三角矩阵

$H_{i}$ 是正交矩阵

$A=Q\begin{bmatrix} R\\0 \end{bmatrix}$

如果 $m<n$：

$A=Q[R,S]$

## 4. Cholesky 分解的原理

n 阶实对称矩阵 A

如果 A 可分解为 $A=LDU$ ，L为下三角矩阵，U为上三角矩阵，LU的主对角元素皆为1，D为对角矩阵

$A=LDU=A^T=(LDU)^T=U^T D L^T $

如果D的主对角元素不含0，则LDU分解时唯一的，$U=L^T$

如果A是正定矩阵，则对于所有的非零实向量x

$x^T Ax =x^T LDL^T x=y^TDy =\Sigma_{i=1}^n  d_{ii} y_i^2>0$

$y=(y_1,\cdots ,y_n)^T=L^T x$

L是可逆矩阵，任何一个非零向量y都满足上述不等式，设y为标准单位向量 $e_i$ ，推知每一 $d_ii$ 为正数

令  $D^{1/2}=diag(\sqrt{d_{11}} ,\cdots ,\sqrt{d_{nn}  })$

$A=LDL^T =LD^{1/2}D^{1/2}L^T=(LD^{1/2} )(LD^{1/2})^T$

实对称正定矩阵 $A$ 的 Cholesky 分解为 $A=GG^T, G=LD^{1/2}$ 是包含正主对角元的下三角矩阵，并且由于LDU分解时唯一的，正定矩阵的Cholesky分解也是唯一的。



## 5. 编程实现  A 为 100×100 随机矩阵时，⽤ QR 和 Cholesky 分解求 x 的程序

```cpp
#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#define MATRIX_SIZE 100
using namespace std;

int main( int argc, char** argv )
{
    Eigen::MatrixXd matrix;
    matrix = Eigen::MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
    matrix = matrix.transponse()*matrix;
    Eigen::VectorXd b;
    b = Eigen::MatrixXd::Random( MATRIX_SIZE,1 );


    clock_t time_stt = clock();

    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix.inverse()*b;
    cout<<b<<endl;
    cout <<"time use in normal inverse is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;

    // QR分解
    time_stt = clock();
    x = matrix.colPivHouseholderQr().solve(b);
    cout <<"time use in Qr decomposition is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    // Cholesky 分解
    time_stt = clock();
    x = matrix.ldlt().solve(b);
    cout <<"time use in Qr decomposition is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    return 0;
}
```


# 几何运算练习

小萝卜一号的位姿为 $q_1 =[0.35,0.2,0.3,0.1] , t_1 =[0.3,0.1,0.1]^T$  （$T_{cw}$ 世界到相机的变换关系）

小萝卜二号的位姿为 $q_2 =[-0.5,0.4,-0.1,0.2]  , t_2 =[-0.1,0.5,0.3]^T$

现在小萝卜一号看到某个点在自身坐标系下，坐标为 $p=[0.5,0,0.2]^T$ 

求该点在小萝卜二号下的坐标的代码

```cpp
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

int main( int argc, char** argv )
{
    Eigen::Quaterniond q_1(0.35,0.2,0.3,0.1);
    Eigen::Quaterniond q_2(-0.5,0.4,-0.1,0.2);
    q_1.normalize();
    q_2.normalize();
    Eigen::Vector3d p_1(0.5,0,0.2);
    Eigen::Vector3d p_2;
    Eigen::Vector3d t_1(0.3,0.1,0.1);
    Eigen::Vector3d t_2(-0.1,0.5,0.3);
  
    p_2 = q_2*(t_1+q_1.conjugate()*p_1 -t_2 );
    cout<<p_2<<endl;
    return 0;
}
```

# 罗德里格斯公式的证明

$ R=cos(\theta) I -(1-cos(\theta))nn^T +sin(\theta) n^\land $


## 1. 旋转矩阵

适当的正交矩阵的行列式为1，实正交矩阵的特征值的绝对值等于1，且实矩阵若存在复特征值必然为共轭对。因此，由欧拉旋转定理可推论$3\times 3$阶旋转矩阵R的特征值为$1，e^{i\theta}，e^{-i \theta}$。旋转矩阵R满足$RR^\ast=R^\ast R$，此为正规矩阵，等价条件是R可么正对角化为$R=UDU^\ast$，其中U是酉矩阵，$U^\ast=U^{-1}$，且$D=\text{diag}（1，e^{i\theta}，e^{ -i\theta}）$。令$U=\begin{bmatrix} \mathbf{u}&\mathbf{v}&\mathbf{w} \end{bmatrix}$，其中$\mathbf{u}，\mathbf{v}，\mathbf{w}$组成一个单位正交集。酉对角化蕴含下列性质：

$U^\ast U=I$推得$\mathbf{u}^\ast\mathbf{u}=\mathbf{v}^\ast\mathbf{v}=\mathbf{w}^\ast\mathbf{w}=1$且$\mathbf{u}^\ast\mathbf{v}=\mathbf{v}^\ast\mathbf{w}=\mathbf{w}^\ast\mathbf{u}=0$

$UU^\ast=I$推得$\mathbf{u}\mathbf{u}^\ast+\mathbf{v}\mathbf{v}^\ast+\mathbf{w}\mathbf{w}^\ast=I$

乘开$RU=UD$，$R\mathbf{u}=\mathbf{u}$表明$\mathbf{u}$是一个单位实向量，$R\mathbf{v}=e^{i\theta}\mathbf{v}$和$R\mathbf{w}=e^{-i\theta}\mathbf{w}$表明$\mathbf{v}$和$\mathbf{w}$是单位复向量。

使用这些性质以及欧拉公式$e^{\pm i\theta}=\cos\theta\pm i\sin\theta$，

$  R=\begin{bmatrix} \mathbf{u}&\mathbf{v}&\mathbf{w} \end{bmatrix}\begin{bmatrix} 1&0&0\\ 0&e^{i\theta}&0\\ 0&0&e^{-i\theta} \end{bmatrix}  \begin{bmatrix} \mathbf{u}^\ast\\ \mathbf{v}^\ast\\ \mathbf{w}^\ast \end{bmatrix}$

$=\mathbf{u}\mathbf{u}^\ast+（\cos\theta+i\sin\theta）\mathbf{v}\mathbf{v}^\ast+（\cos\theta-i\sin\theta）\mathbf{w}\mathbf{w}^\ast$

$=\cos\theta（\mathbf{u}\mathbf{u}^\ast+\mathbf{v}\mathbf{v}^\ast+\mathbf{w}\mathbf{w}^\ast）+（1-\ cos\theta）\mathbf{u}\mathbf{u}^\ast+i\sin\theta（\mathbf{v}\mathbf{v}^\ast-\mathbf{w}\mathbf{w}^\ast）$

$ =\cos\theta I+（1-\cos\theta）\mathbf{u}\mathbf{u}^T+i\sin\theta（\mathbf{v}\mathbf{v}^\ast-\mathbf{w}\mathbf{w}^\ast）=\cos\theta I+（1-\cos\theta）\mathbf{u}\mathbf{u}^T+\sin\theta K$

其中$K=i（\mathbf{v}\mathbf{v}^\ast-\mathbf{w}\mathbf{w}^\ast）$。因为R和$\mathbf{u}\mathbf{u}^T$是实矩阵，上式表明K也是一个实矩阵，则

$\displaystyle K^T=K^\ast=-i（\mathbf{v}\mathbf{v}^\ast-\mathbf{w}\mathbf{w}^\ast）^\ast=-i（\mathbf{v}\mathbf{v}^\ast-\mathbf{w}\mathbf{w}^\ast）=-K$

推知K是反对称矩阵。令$K=\left[\begin{array}{rrr} 0&a&b\\ -a&0&c\\ -b&-c&0 \end{array}\right]$，其中$a，b，c\in\mathbb{R}$。对于任一$\theta$，$\mathbf{u}=R\mathbf{u}=\cos\theta\mathbf{u}+（1-\cos\theta）\mathbf{u}+\sin\theta K\mathbf{u}=\mathbf{u}+\sin\theta K\mathbf{u}$，可得

$\displaystyle K\mathbf{u}=\left[ \begin{array}{rrr} 0&a&b\\ -a&0&c\\ -b&-c&0 \end{array} \right]\begin{bmatrix} u_1\\ u_2\\ u_3 \end{bmatrix}=\begin{bmatrix} 0\\ 0\\ 0 \end{bmatrix}$

或改写为

$\displaystyle \left[ \begin{array}{rrr} u_2&u_3&0\\ -u_1&0&u_3\\ 0&-u_1&-u_2 \end{array}\right]\begin{bmatrix} a\\ b\\ c \end{bmatrix}=\begin{bmatrix} 0\\ 0\\ 0 \end{bmatrix}$。

解开上式，

$\displaystyle \begin{bmatrix} a\\ b\\ c \end{bmatrix}=\alpha\left[ \begin{array}{r} -u_3\\ u_2\\ -u_1 \end{array} \right]$，

其中$\alpha$是一个待确定的实数。定义「外积矩阵」

$ \mathbf{u}^\land=\left[\begin{array}{rrr} 0&-u_3&u_2\\ u_3&0&-u_1\\ -u_2&u_1&0 \end{array}\right]$，

即有$K=\alpha\mathbf{u}^\land  $。使用单范正交集$\{\mathbf{u}，\mathbf{v}，\mathbf{w}\}$的基本性质，

$\displaystyle\begin{aligned} K^2&=i^2（\mathbf{v}\mathbf{v}^\ast-\mathbf{w}\mathbf{w}^\ast）（\mathbf{v}\mathbf{v}^\ast-\mathbf{w}\mathbf{w}^\ast）\\ &=-（\mathbf{v}\mathbf{v}^\ast\mathbf{v}\mathbf{v}^\ast-\mathbf{v}\mathbf{v}^\ast\mathbf{w}\mathbf {w}^\ast-\mathbf{w}\mathbf{w}^\ast\mathbf{v}\mathbf{v}^\ast+\mathbf{w}\mathbf{w}^\ast\mathbf{w}\mathbf{w}^\ast）\\ &=-（\mathbf{v}\mathbf{v}^\ast+\mathbf{w}\mathbf{w}^\ast）\\ &=\mathbf{u}\mathbf{u}^T-I\end{aligned}$

再将$K=\alpha\mathbf{u} ^\land$代入上式，立得$\alpha=\pm 1$。我们选择$\alpha=1$，原因是配合惯例使$\theta$代表逆时针转角。令$R_\mathbf{u}（\theta）$代表一个旋转矩阵，其特征值为$1，\cos\theta\pm i\sin\theta$且$\mathbf{u}$为对应特征值1的单位特征向量。综合以上结果，旋转矩阵$R_\mathbf{u}（\theta）$的公式如下：

$\displaystyle\begin{aligned} R_\mathbf{u}（\theta）&=\cos\theta I+（1-\cos\theta）\mathbf{u}\mathbf{u}^T+\sin\theta \mathbf{u} ^\land\\ &=\begin{bmatrix} u_1u_1v\theta+c\theta&u_1u_2v\theta-u_3s\theta&u_1u_3v\theta +u_2s\theta\\ u_1u_2v\theta+u_3s\theta&u_2u_2v\theta+c\theta&u_2u_3v\theta-u_1s\theta\\ u_1u_3v\theta-u_2s\theta&u_2u_3v\theta+u_1s\theta&u_3u_3v\theta+c\theta \end{bmatrix}，\end{aligned}$

## 2. 四元数

若 $q=a+\mathbf{v} $ 为单位四元数，即 $\vert q\vert^2=a^2+\Vert\mathbf{v}\Vert^2=1$ ，设 $a^2=\cos^2\frac{\theta}{2}$ 且 $ \Vert\mathbf{v}\Vert^2=\sin^2\frac{\theta}{2} $ 。因此，存在  $\theta\in[0,2\pi)$ 使得 $a=\cos\frac{\theta}{2}$ 且 $\Vert\mathbf{v}\Vert=\sin\frac{\theta}{2}\ge 0$  。令 $\mathbf{u}=\frac{\mathbf{v}}{\Vert\mathbf{v}\Vert}$  。每一个单位四元数$q=a+\mathbf{v}$ 定可以表示为

$\displaystyle q=\cos\frac{\theta}{2}+\sin\frac{\theta}{2}\mathbf{u}$

其中$\Vert\mathbf{u}\Vert=1$。我们可以证明：以单位向量$\mathbf{u}$为转轴，线性变换$L_q(\mathbf{x})=q\mathbf{x}q^\ast$ 代表$ \mathbf{x}$逆时针旋转$\theta$角度。将给定向量$\mathbf{x}$分解成$\mathbf{x}=\mathbf{z}+\ mathbf{n}$，其中$\mathbf{z}$是$\mathbf{x}$在转轴$\mathbf{u}$的正交投影，$\mathbf{n}$代表垂直于$\mathbf{u}$的成分。因为$L_q（\mathbf{x}）=L_q（\mathbf{z}+\mathbf{n}）=L_q（\mathbf{z}）+L_q（\mathbf{n}）$，我们要证明$L_q$不改变$\mathbf{z}$且$L_q$将$\mathbf{n}$逆时针旋转$\theta$角度，见下图。

![](https://ccjou.files.wordpress.com/2014/04/quaternion-and-3d-rotations.gif)

因为存在唯一$\alpha\in\mathbb{R}$使$\mathbf{z}=\alpha\mathbf{u}$，并且由性质 $\displaystyle \Vert L_q（\mathbf{x}）\Vert=\Vert q\mathbf{x}q^\ast\Vert=\vert q\vert~\Vert\mathbf{x}\Vert~\vert q^\ast\vert=\Vert\mathbf{x}\Vert$， 可知$L_q（\mathbf{z}）=\mathbf{z}$。利用$\mathbf{n}\perp\mathbf{u}$，即$\mathbf{n}\perp\mathbf{v}$，可得

$\displaystyle\begin{aligned} L_q（\mathbf{n}）&=（a^2-\Vert\mathbf{v}\Vert^2）\mathbf{n}+2（\mathbf{v}\cdot\mathbf{n}）\mathbf{v}+2a（\mathbf{v}\times\mathbf{n}）\\ &=（a^2-\Vert\mathbf{v}\Vert^2）\mathbf{n}+2a（\mathbf{v}\times\mathbf{n}）=（a^2-\Vert\mathbf{v}\Vert^2）\mathbf{n}+2a\Vert\mathbf{v}\Vert（\mathbf{u}\times\mathbf{n}）\\ &=（a^2-\Vert\mathbf{v}\Vert^2）\mathbf{n}+2a\Vert\mathbf{v}\Vert\mathbf{n}_{\perp}，\end{aligned}$

上面令$\mathbf{n}_{\perp}=\mathbf{u}\times\mathbf{n}$。注意$\mathbf{n}_\perp$垂直于$\mathbf{n}$且

$\displaystyle \Vert\mathbf{n}_\perp\Vert=\Vert \mathbf{u}\times{\mathbf{n}}\Vert=\Vert\mathbf{u}\Vert~\Vert\mathbf{n}\Vert\sin\frac{\pi}{2}=\Vert\mathbf{n}\Vert$

最后，将$a=\cos\frac{\theta}{2}$和$\Vert\mathbf{v}\Vert=\sin\frac{\theta}{2}$代入$L_q（\mathbf{n}）$的表达式，使用倍角公式，

$ L_q（\mathbf{n}）= （\cos^2\frac{\theta}{2}-\sin^2\frac{\theta}{2}）\mathbf{n}+ （2\cos\frac{\theta}{2}\sin\frac{\theta}{2} ）\mathbf{n}_\perp=\cos\theta\mathbf{n}+\sin\theta\mathbf{n}_\perp $

证得$L_q（\mathbf{n}）$即是垂直于转轴$\mathbf{u}$的平面上$\mathbf{n}$逆时针旋转$\theta$而得的向量。

将单位四元数$q=\cos\frac{\theta}{2}+\sin\frac{\theta}{2}\mathbf{u}$代入旋转变换$L_q（\mathbf{x}）$的展开式，套用倍角公式和半角公式，推得

$  L_q（\mathbf{x}）= （\cos^2\frac{\theta}{2}-\sin^2\frac{\theta}{2} ）\mathbf{x}+2 （\sin\frac{\theta}{2}\mathbf{u}\cdot\mathbf{x}   ）\sin\frac{\theta}{2}\mathbf{u}+2\cos\frac{\theta}{2} （\sin\ frac{\theta}{2}\mathbf{u}\times\mathbf{x} ） $

$=\cos\theta\mathbf{x}+（1-\cos\theta）（\mathbf{u}\cdot\mathbf{x}）\mathbf{u}+\sin\theta（\mathbf{u}\times\mathbf{x}） $


# 四元数运算性质的验证

## 1. 

用四元数 $q$ 旋转点 $p$ 时，结果为： $p'=qpq^{-1}$

此时 $p'$ 必定为虚四元数（实部为零）

证明：

$q=(c,sv), p =(0,x)$

$p' =qpq^{-1} =(c,sv)*(0,x)*(c,-sv)=(-svx,sv\times x+cx)*(c,-sv)$

$=(-scvx+(sv\times x+cx)(sv) ,(sv\times x+cx)\times (sv)+(sv\times x+cx)c+svxsv)$

实部为 $-scvx+(sv\times x+cx)(sv) = sv\times x ·v = 0 $



## 2.

上式亦可以写成矩阵运算： $p' =Qp$ ，根据推导给出矩阵 $Q_{4\times 4}$ 

$q =w+x i+y j +kz$

$Q=\begin{bmatrix}    1-2y^2 -2z^2 &&2 xy+2wz && 2xz -2wy  \\    2xy-2wz &&  1-2x^2 -2z^2  && 2 yz +2wx   \\  2 x z+ 2wy&&   2yz-2wx && 1-2x^2 -2y^2 \end{bmatrix}$

# C++11

```cpp

#include <iostream> 
#include <vector> 
#include <algorithm> 
using namespace std; 
class A {
public: 
	A(const int& i ) : index(i) {} 
	int index = 0; 
}; 
int main() { 
	A a1(3), a2(5), a3(9); 
	vector<A> avec{a1, a2, a3};
	std::sort(avec.begin(), avec.end(), [](const A&a1, const A&a2) {return a1.index<a2.index;});
	for( auto& a: avec ) cout<<a.index<<" "; 
	cout<<endl; 
	return 0; 
}
```

` for( auto& a: avec ) ` 基于范围的for循环

` auto& a` 类型推导

` [](const A&a1, const A&a2)` lambda 表达式