---
title: 使用G2O解决优化问题的简单例子
categories: 实用工具
tags:
  - G2O
abbrlink: 58e495ff
date: 2022-10-16 09:31:08
---




## 优化问题描述



假设一个机器人初始起点在0处，然后机器人向前移动，通过编码器测得它向前移动了1m，到达第二个地点x1。接着，又向后返回，编码器测得它向后移动了0.8米。但是，通过闭环检测，发现它回到了原始起点。可以看出，编码器误差导致计算的位姿和观测到有差异，那机器人这几个状态中的位姿到底是怎么样的才最好的满足这些条件呢？

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/0.png)



首先构建位姿之间的关系，即图的边：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/1.png)



线性方程组中变量小于方程的个数，要计算出最优的结果，使出杀手锏***最小二乘法***。先构建残差平方和函数：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/2.png)



为了使残差平方和最小，我们对上面的函数每个变量求偏导，并使得偏导数等于0.

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/3.png)

<!--more-->

整理得到：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/4.png)

接着矩阵求解线性方程组：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/5.png)



所以调整以后为满足这些边的条件，机器人的位姿为：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/6.png)



可以看到，因为有了x2->x0的闭环检测，几个位置点间才能形成互相约束。这对使用最小二乘解决该优化问题起到了决定性的作用。



该问题描述来源于：[https://heyijia.blog.csdn.net/article/details/47686523](https://heyijia.blog.csdn.net/article/details/47686523)

下面利用G2O来解上面的问题，以便理解如何使用G2O。



## 定义顶点

在该问题中，一个位置点就是图优化中的一个顶点。一个顶点可以包含多个需优化量。比如二维环境下的机器人位置一般是3维的(x,y,theta)，即一个顶点有三个需要优化的量。



在此问题中，我们只需优化求解一个一维的距离值。即是，一个顶点只包含一个需优化量。



```c++

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class SimpleVertex : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SimpleVertex(bool fixed = false)
  {
    setToOriginImpl();
    setFixed(fixed);
  }

  SimpleVertex(double e, bool fixed = false)
  {
    _estimate = e;
    setFixed(fixed);
  }

  // 重置
  virtual void setToOriginImpl() override {
    _estimate = 0;
  }

  // 更新
  virtual void oplusImpl(const double *update) override {
    _estimate += *update;
  }

  // 存盘和读盘：留空
  virtual bool read(istream &in) {return true;}

  virtual bool write(ostream &out) const {return true;}
};
```



上面代码中`g2o::BaseVertex<1, double>`就表示了优化的量是一维的，数据类型是`double`。

如果查看`TEB`中设置的优化量，可以发现它是这样写的：

```c++
g2o::BaseVertex<3, PoseSE2 >
```

`TEB`中的优化量是三维的，即机器人的位姿(x,y,theta)。此处的`PoseSE2`便是描述机器人位姿的结构体。



自定义一个顶点通常需要重新实现`setToOriginImpl()`和`oplusImpl(const double *update)`这两个函数。至于`read(istream &in)`和`write(ostream &out)`是为了保存和读取当前顶点数据的。用不上可留空。



`setToOriginImpl()`函数用于重置顶点的数据。顶点包含的数据变量是`_estimate`。该变量的类型即是`g2o::BaseVertex<1, double>`中设置的`double`。该函数正是用于重置`_estimate`和使顶点恢复默认状态。



`oplusImpl(const double *update)`函数用于叠加优化量的步长。注意有时候这样的叠加操作并不是线性的。比如`2D-SLAM`中位置步进叠加操作则不是线性的。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/7.jpeg)



在此问题中我们是直接线性叠加一维的距离值。



`TEB`中的位置叠加也是线性叠加的。放置在下面以作参考。

```c++
void plus(const double* pose_as_array)
{
    _position.coeffRef(0) += pose_as_array[0];
    _position.coeffRef(1) += pose_as_array[1];
    _theta = g2o::normalize_theta( _theta + pose_as_array[2] );
}

virtual void oplusImpl(const double* update) override
{
    _estimate.plus(update);
}
```



另外，顶点是可以设置成固定的。当不需要变动某个顶点时，使用`setFixed`函数来固定。通常，一个优化问题中，至少需要固定一个顶点，否则所有的顶点都在浮动，优化效果也不会好。



## 定义边

边即是顶点之间的约束。在该问题中就是两位置间的测量值和观测值之间的差值要趋近于0。这里需要定义的边其实就是下面的等式。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/8.png)



实际上，G2O中边有三种：一元边(`g2o::BaseUnaryEdge`)，二元边(`g2o::BaseBinaryEdge`)和多元边(`g2o::BaseMultiEdge`)。

```c++
g2o::BaseUnaryEdge<D, E, VertexXi>
g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
g2o::BaseMultiEdge<D, E>
```

不同类型的边有不同数量的模板参数。其中`D` 是 int 型，表示误差值的维度 （dimension）， `E` 表示测量值的数据类型（即`_measurement`的类型），` VertexXi`，`VertexXj `分别表示不同顶点的类型。当`D`为2时，`_error`的类型变为`Eigen::Vector2d`，当`D`为3时，`_error`的类型变为`Eigen::Vector3d`。



在上面的约束中，有一个一元边(f1)和三个二元边(f2,f3,f4)。在G2O中可如下定义：

```c++

// 误差模型 模板参数：测量值维度，测量值类型，连接顶点类型
class SimpleUnaryEdge : public g2o::BaseUnaryEdge<1, double, SimpleVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleUnaryEdge() : BaseUnaryEdge() {}

  // 计算模型误差
  virtual void computeError() override {
    const SimpleVertex *v = static_cast<const SimpleVertex *> (_vertices[0]);
    const double abc = v->estimate();
    _error(0, 0) = _measurement - abc;
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    _jacobianOplusXi[0] = -1;//一元边只有一个顶点，所以只有_jacobianOplusXi。_jacobianOplusXi的维度和顶点需优化的维度是一致的
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

};

// 误差模型 模板参数：测量值维度，测量值类型，连接顶点类型
class SimpleBinaryEdge : public g2o::BaseBinaryEdge<1, double, SimpleVertex, SimpleVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleBinaryEdge() {}

  // 计算模型误差
  virtual void computeError() override {
    const SimpleVertex *v1 = static_cast<const SimpleVertex *> (_vertices[0]);
    const SimpleVertex *v2 = static_cast<const SimpleVertex *> (_vertices[1]);
    const double abc1 = v1->estimate();
    const double abc2 = v2->estimate();
    _error[0] = _measurement - (abc1 - abc2);
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    _jacobianOplusXi(0,0) = -1;//二元边有两个顶点，偏差_error对每个顶点的偏导数都需要求解;如果_error是多维的，则每一维的偏导都需要求解。即会出现_jacobianOplusXi(1,0)

    _jacobianOplusXj(0,0) = 1;//因为误差_error只有一维，所以_jacobianOplusXi只有_jacobianOplusXi(0,0)项。此时_jacobianOplusXi[0]与_jacobianOplusXi(0,0)效果等价。
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

};
```



因为我们这里测量值是一维的距离值（double类型）而顶点类型为`SimpleVertex`，所以边需设置成`g2o::BaseUnaryEdge<1, double, SimpleVertex>`和`g2o::BaseBinaryEdge<1, double, SimpleVertex, SimpleVertex>`类型。



自定义边同样需要特别关注两个函数`computeError()`和`linearizeOplus()`。

`computeError()`是用于计算迭代误差的。顶点间的约束正是由误差计算函数构建的。优化时误差项将逐步趋近于0。`_error`的维度和类型通常由构建的模型决定。比如该问题中误差为距离误差。

`TEB`的速度边构建的误差项为线速度与最大线速度的差值和角速度与最大角速度的差值。所以这里的误差项是二维的。

```c++
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    
    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();
    
    double dist = deltaS.norm();
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    double vel = dist / deltaT->estimate();
    
//     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction
    
    const double omega = angle_diff / deltaT->estimate();
  
    _error[0] = penaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }
```



`linearizeOplus()`函数里主要是配置雅克比矩阵。当然，G2O是支持自动求导的，该函数可以不实现。优化时由G2O自动处理。但准确的实现可加快优化计算的速度。下面介绍雅克比矩阵该如何计算。



### 计算雅克比矩阵



雅克比矩阵存储了误差项的每一维相对于顶点各优化成员的偏导数。



#### 一元边

一元边只有一个顶点所以只定义了`_jacobianOplusXi`，其对应顶点类型`VertexXi`。注意，`_jacobianOplus`变量才完整地描述雅克比矩阵，`_jacobianOplusXi`只是`_jacobianOplus`中相关部分的引用，以方便配置雅克比矩阵。

```c++
// This could be a simple using statement, but in multiple places
// _jacobianOplusXi is used.
template <int D, typename E, typename VertexXi>
class BaseUnaryEdge : public BaseFixedSizedEdge<D, E, VertexXi> {
 public:
  using VertexXiType = VertexXi;
  BaseUnaryEdge() : BaseFixedSizedEdge<D, E, VertexXi>(){};

 protected:
  typename BaseFixedSizedEdge<D, E, VertexXi>::template JacobianType<
      D, VertexXi::Dimension>& _jacobianOplusXi =
      std::get<0>(this->_jacobianOplus);
};
```



本优化问题的一元边按下面代码的方式计算雅克比矩阵。

```c++

// 误差模型 模板参数：误差项维度，测量值类型，连接顶点类型
class SimpleUnaryEdge : public g2o::BaseUnaryEdge<1, double, SimpleVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleUnaryEdge() : BaseUnaryEdge() {}

  // 计算曲线模型误差
  virtual void computeError() override {
    const SimpleVertex *v = static_cast<const SimpleVertex *> (_vertices[0]);
    const double abc = v->estimate();
    _error(0, 0) = _measurement - abc;
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    _jacobianOplusXi[0] = -1;//一元边只有一个顶点，所以只有_jacobianOplusXi。_jacobianOplusXi的维度和顶点需优化的维度是一致的
  }//此处就是求误差相对于abc变量的导数

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

};
```





#### 二元边

二元边有两个顶点，所以定义了`_jacobianOplusXi`和`_jacobianOplusXj`。

```c++

// This could be a simple using statement, but in multiple places
// _jacobianOplusXi and _jacobianOplusYi are used.
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseBinaryEdge : public BaseFixedSizedEdge<D, E, VertexXi, VertexXj> {
 public:
  using VertexXiType = VertexXi;
  using VertexXjType = VertexXj;
  BaseBinaryEdge() : BaseFixedSizedEdge<D, E, VertexXi, VertexXj>(){};

 protected:
  typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj>::template JacobianType<
      D, VertexXi::Dimension>& _jacobianOplusXi =
      std::get<0>(this->_jacobianOplus);
  typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj>::template JacobianType<
      D, VertexXj::Dimension>& _jacobianOplusXj =
      std::get<1>(this->_jacobianOplus);
};
```



本优化问题的二元边按下面代码的方式计算雅克比矩阵。

```c++

// 误差模型 模板参数：误差项维度，测量值类型，连接顶点类型
class SimpleBinaryEdge : public g2o::BaseBinaryEdge<1, double, SimpleVertex, SimpleVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleBinaryEdge() {}

  // 计算曲线模型误差
  virtual void computeError() override {
    const SimpleVertex *v1 = static_cast<const SimpleVertex *> (_vertices[0]);
    const SimpleVertex *v2 = static_cast<const SimpleVertex *> (_vertices[1]);
    const double abc1 = v1->estimate();
    const double abc2 = v2->estimate();
    _error[0] = _measurement - (abc1 - abc2);
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    _jacobianOplusXi(0,0) = -1;//二元边有两个顶点，偏差_error对每个顶点的偏导数都需要求解;如果_error是多维的，则每一维的偏导都需要求解。即会出现_jacobianOplusXi(1,0)

    _jacobianOplusXj(0,0) = 1;//因为误差_error只有一维，所以_jacobianOplusXi只有_jacobianOplusXi(0,0)项。此时_jacobianOplusXi[0]与_jacobianOplusXi(0,0)效果等价。
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

};
```



#### 多元边

多元边直接配置`_jacobianOplus`变量。



下面的示例代码是`teb`中的速度约束，来源于`edge_velocity.h`文件。

```c++
class EdgeVelocity : public BaseTebMultiEdge<2, double>
{
public:
  
  /**
   * @brief Construct edge.
   */	      
  EdgeVelocity()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }
  
  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    
    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();
    
    double dist = deltaS.norm();
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    double vel = dist / deltaT->estimate();
    
//     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction
    
    const double omega = angle_diff / deltaT->estimate();
  
    _error[0] = penaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }

#ifdef USE_ANALYTIC_JACOBI
#if 0 //TODO the hardcoded jacobian does not include the changing direction (just the absolute value)
      // Change accordingly...

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    
    Eigen::Vector2d deltaS = conf2->position() - conf1->position();
    double dist = deltaS.norm();
    double aux1 = dist*deltaT->estimate();
    double aux2 = 1/deltaT->estimate();
    
    double vel = dist * aux2;
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) * aux2;
    
    double dev_border_vel = penaltyBoundToIntervalDerivative(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    double dev_border_omega = penaltyBoundToIntervalDerivative(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);
    
    _jacobianOplus[0].resize(2,3); // conf1
    _jacobianOplus[1].resize(2,3); // conf2
    _jacobianOplus[2].resize(2,1); // deltaT
    
//  if (aux1==0) aux1=1e-6;
//  if (aux2==0) aux2=1e-6;
    
    if (dev_border_vel!=0)
    {
      double aux3 = dev_border_vel / aux1;
      _jacobianOplus[0](0,0) = -deltaS[0] * aux3; // vel x1
      _jacobianOplus[0](0,1) = -deltaS[1] * aux3; // vel y1	
      _jacobianOplus[1](0,0) = deltaS[0] * aux3; // vel x2
      _jacobianOplus[1](0,1) = deltaS[1] * aux3; // vel y2
      _jacobianOplus[2](0,0) = -vel * aux2 * dev_border_vel; // vel deltaT
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // vel x1
      _jacobianOplus[0](0,1) = 0; // vel y1	
      _jacobianOplus[1](0,0) = 0; // vel x2
      _jacobianOplus[1](0,1) = 0; // vel y2	
      _jacobianOplus[2](0,0) = 0; // vel deltaT
    }
    
    if (dev_border_omega!=0)
    {
      double aux4 = aux2 * dev_border_omega;
      _jacobianOplus[2](1,0) = -omega * aux4; // omega deltaT
      _jacobianOplus[0](1,2) = -aux4; // omega angle1
      _jacobianOplus[1](1,2) = aux4; // omega angle2
    }
    else
    {
      _jacobianOplus[2](1,0) = 0; // omega deltaT
      _jacobianOplus[0](1,2) = 0; // omega angle1
      _jacobianOplus[1](1,2) = 0; // omega angle2			
    }

    _jacobianOplus[0](1,0) = 0; // omega x1
    _jacobianOplus[0](1,1) = 0; // omega y1
    _jacobianOplus[1](1,0) = 0; // omega x2
    _jacobianOplus[1](1,1) = 0; // omega y2
    _jacobianOplus[0](0,2) = 0; // vel angle1
    _jacobianOplus[1](0,2) = 0; // vel angle2
  }
#endif
#endif
 
  
public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
```



上面的代码中`_error[0]`对应线速度`vel`误差，`_error[1]`对应角速度`omega`误差。所以需要求解`vel`和`omega`分别相对于顶点变量`x,y,theta`的导数。

`_jacobianOplus[0].resize(2,3);`对应第一个顶点，其中误差项有两维，顶点优化变量有3维，所以雅克比矩阵是一个2x3的矩阵。

`_jacobianOplus[1].resize(2,3);`对应第二个顶点，其中误差项有两维，顶点优化变量有3维，所以雅克比矩阵是一个2x3的矩阵。

`_jacobianOplus[2].resize(2,1);`对应第三个顶点，其中误差项有两维，顶点是时间差，只有一维，所以雅克比矩阵是一个2x1的矩阵。



## 创建优化器



```c++
  // 构建图优化，先设定g2o
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<1, 1>> BlockSolverType;  // 每个误差项优化变量维度为1，误差值维度为1
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

  // 梯度下降方法，可以从GN, LM, DogLeg 中选
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;     // 图模型
  optimizer.setAlgorithm(solver);   // 设置求解器
  optimizer.setVerbose(true);       // 打开调试输出
```



`teb`中的优化器设置

```c++

//! Typedef for the block solver utilized for optimization
typedef g2o::BlockSolverX TEBBlockSolver;

//! Typedef for the linear solver utilized for optimization
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;
//typedef g2o::LinearSolverCholmod<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
std::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static std::once_flag flag;
  std::call_once(flag, [this](){this->registerG2OTypes();});

  // allocating the optimizer
  std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
  auto linearSolver = std::make_unique<TEBLinearSolver>(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  auto blockSolver = std::make_unique<TEBBlockSolver>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}
```

从上面的代码对比可以看出，`BlockSolver`可设成固定的，也可以设成动态的。

下面是本人对于`BlockSolver`设置的理解，不一定正确。仅为参考。

优化变量的维度通常是明确的，但误差项的维度可能是变化的。每条边的误差项维度可能都不一样。这时应该使用`g2o::BlockSolverX`，以便能动态适应误差项的维度。

`linear solver`也是可选的。主要有下面几种可选：

```c++
g2o::LinearSolverCSparse
g2o::LinearSolverCholmod
g2o::LinearSolverEigen
g2o::LinearSolverDense
g2o::LinearSolverPCG
```

不同的`linear solver`有不同的依赖，需要注意是否已经安装了相应的依赖。



## 添加顶点



```c++

  // 往图中增加顶点
  std::vector<SimpleVertex *> vertexs;

  SimpleVertex *v = new SimpleVertex();
  v->setEstimate(0);
  v->setId(0);
  v->setFixed(true);
  optimizer.addVertex(v);
  vertexs.push_back(v);

  SimpleVertex *v1 = new SimpleVertex();
  v1->setEstimate(1);
  v1->setId(1);
  optimizer.addVertex(v1);
  vertexs.push_back(v1);

  SimpleVertex *v2 = new SimpleVertex();
  v2->setEstimate(0.1);
  v2->setId(2);
  optimizer.addVertex(v2);
  vertexs.push_back(v2);

```

顶点的`id`需要确保不能重复。

顶点可使用`setEstimate`接口设置初始值。



## 添加边



```c++
  // 往图中增加边
  SimpleUnaryEdge *edge = new SimpleUnaryEdge();
  // edge->setId(i);
  edge->setVertex(0, vertexs[0]);                // 设置连接的顶点
  edge->setMeasurement(0);      // 观测数值
  edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // 信息矩阵：协方差矩阵之逆
  optimizer.addEdge(edge);

  SimpleBinaryEdge * edge1 = new SimpleBinaryEdge();
  // edge->setId(i);//id 不设置似乎没有关系，如果设置需要每条边设置成不一样的
  edge1->setVertex(0, vertexs[1]);//这里设置的序号对应的顶点要和边的computeError函数里设定的顶点是一一对应的
  edge1->setVertex(1, vertexs[0]);                // 设置连接的顶点
  edge1->setMeasurement(1);      // 观测数值
  edge1->setInformation(Eigen::Matrix<double, 1, 1>::Identity()*1.0); // 信息矩阵：协方差矩阵之逆
  optimizer.addEdge(edge1);

  SimpleBinaryEdge * edge2 = new SimpleBinaryEdge();
  // edge->setId(i);
  edge2->setVertex(0, vertexs[2]);                // 设置连接的顶点
  edge2->setVertex(1, vertexs[1]);                // 设置连接的顶点
  edge2->setMeasurement(-0.8);      // 观测数值
  edge2->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // 信息矩阵：协方差矩阵之逆
  optimizer.addEdge(edge2);

  SimpleBinaryEdge * edge3 = new SimpleBinaryEdge();
  // edge->setId(i);
  edge3->setVertex(0, vertexs[2]);                // 设置连接的顶点
  edge3->setVertex(1, vertexs[0]);                // 设置连接的顶点
  edge3->setMeasurement(0);      // 观测数值
  edge3->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // 信息矩阵：协方差矩阵之逆
  optimizer.addEdge(edge3);

```

边也有`setId`接口，但不设置好像也可以。如果设置也需确保不重复。但`id`号的顺序似乎并没有要求。

使用`setVertex`接口设置顶点时是有顺序的。这个顺序与边的`computeError`函数中使用顶点的顺序要对应起来。

`setMeasurement`接口用于设置内部的`_measurement`值。

`setInformation`接口用于设置信息矩阵。信息矩阵是方正矩阵，其行列数由误差项的维度决定。

```c++
void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  ...
}
```



最终优化求解的结果为`estimated model: 0 0.933333 0.0666667 `。基本和标准结果一致。

如果将`edge1`的信息矩阵如下设置：

```c++
edge1->setInformation(Eigen::Matrix<double, 1, 1>::Identity()*10.0);
```

优化求解的结果为：`estimated model: 0 0.990476 0.0952381`

可以发现x1更接近1了。说明此时我们更相信编码器测量的从x0到x1的距离值。



完整的测试代码可查看下面的链接：

[https://github.com/shoufei403/g2o_learning.git](https://github.com/shoufei403/g2o_learning.git)



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)



