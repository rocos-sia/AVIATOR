> 历史记录：仓库现为纯 C++ 项目，Python 绑定及 SWIG 依赖已删除，文中相关内容不再适用。

> 历史迁移记录：当前 Pinocchio 3.9.0 实现及连续关节接口见 [文档首页](README.md)。本文中的进度与版本差异描述可能已过时。

# 连续关节对 IK 求解的影响详解

## 雅可比矩阵维度

### 结论（重要！）
**雅可比矩阵的维度不变！**

- Pinocchio 2.x/3.x: J ∈ ℝ⁶ˣⁿ
- Pinocchio 4.x: J ∈ ℝ⁶ˣⁿ（相同）

连续关节在**配置空间**用 2 个变量表示，但在**速度空间**仍是 1 个变量（角速度）。

雅可比矩阵是关于**速度**的，所以维度不受影响。

## 详细说明

### 1. 配置空间 vs 速度空间

```
配置空间（Configuration Space）:
  Pinocchio 2.x:  q = [x, y, z, roll, pitch, yaw]           (6维)
  Pinocchio 4.x:  q = [x, y, z, roll, pitch, cos(yaw), sin(yaw)]  (7维)
                                                ↑
                                            连续关节用2个变量

速度空间（Velocity/Tangent Space）:
  Pinocchio 2.x:  v = [ẋ, ẏ, ż, roll̇, pitcḣ, yaẇ]        (6维)
  Pinocchio 4.x:  v = [ẋ, ẏ, ż, roll̇, pitcḣ, yaẇ]        (6维)  ← 相同！
                                            ↑
                                        仍然是角速度
```

### 2. 雅可比矩阵计算

```cpp
// Pinocchio 2.x 和 4.x 的雅可比计算方式相同
pinocchio::Data data(model);
pinocchio::computeFrameJacobian(model, data, q, tip_frame_id, 
                                pinocchio::LOCAL_WORLD_ALIGNED, J);

// J 的维度:
// - 行数: 6 (末端执行器的 twist: 3线速度 + 3角速度)
// - 列数: model.nv (速度空间维度，NOT model.nq!)

// 对于测试机器人:
// model.nq = 7  (配置空间)
// model.nv = 6  (速度空间)
// J: 6×6 矩阵
```

### 3. IK 迭代公式

标准的雅可比 IK 迭代：

```
δx = x_target - x_current        (6维位姿误差)
δq = J⁺ * δx                      (关节增量)
q_new = q + δq                    (更新配置)
```

**问题来了**: 这个公式在 Pinocchio 4.x 中需要修改！

#### Pinocchio 2.x/3.x (简单)
```cpp
// δq 和 q 都在同一个向量空间
q_new = q + α * δq;  // α是步长
```

#### Pinocchio 4.x (需要特殊处理)
```cpp
// δq 在切空间(6维), q 在配置空间(7维)
// 不能直接相加！需要使用流形上的积分

// 方法1: 使用 Pinocchio 的 integrate 函数
pinocchio::integrate(model, q, α * δq, q_new);

// 方法2: 手动处理
for (int i = 0; i < 5; ++i) {
    q_new(i) = q(i) + α * δq(i);  // 普通关节
}
// 连续关节(yaw)特殊处理:
double yaw_current = std::atan2(q(6), q(5));  // 从(cos,sin)恢复角度
double yaw_new = yaw_current + α * δq(5);     // 更新角度
q_new(5) = std::cos(yaw_new);
q_new(6) = std::sin(yaw_new);
```

## 对现有代码的影响

### 1. pinocchio_tl.cpp (PIN-IK 的核心迭代求解器)

当前代码（需要修改）:
```cpp
int ChainIkSolverPos_TL::CartToJnt(const Eigen::VectorXd& q_init, 
                                    const SE3& p_in,
                                    Eigen::VectorXd& q_out, 
                                    const Motion bounds) {
    Eigen::VectorXd q = q_init;
    
    // ... 迭代求解 ...
    
    // ❌ 问题: 直接相加在 Pinocchio 4.x 中不正确
    q += delta_q;  
    
    // ✅ 应该使用:
    pinocchio::integrate(model, q, delta_q, q);
}
```

### 2. nlopt_ik.cpp (NLopt 优化求解器)

当前代码（需要修改）:
```cpp
void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double* error) {
    // x 是优化变量 (在什么空间?)
    
    // ❌ 如果 x 在配置空间 (7维):
    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    pinocchio::forwardKinematics(model, data, q);  // OK
    
    // ✅ 如果 x 在"逻辑"空间 (6维角度):
    // 需要转换: angles(6) -> config(7)
    Eigen::VectorXd q(7);
    q.head<5>() = Eigen::Map<const Eigen::VectorXd>(x.data(), 5);
    q(5) = std::cos(x[5]);  // yaw
    q(6) = std::sin(x[5]);
    pinocchio::forwardKinematics(model, data, q);
}
```

## 实现建议

### 方案 A: 在配置空间优化 (简单但不够优雅)

```cpp
// 优化变量: q ∈ ℝ⁷ (配置空间)
// 约束: q(5)² + q(6)² = 1 (单位圆约束)

优点: 不需要转换
缺点: 需要处理约束，优化器效率可能降低
```

### 方案 B: 在速度空间优化 (推荐)

```cpp
// 优化变量: θ ∈ ℝ⁶ (角度/速度空间)
// 前向运动学前转换: θ -> q
// 雅可比: J ∈ ℝ⁶ˣ⁶ (速度空间)

优点: 物理意义清晰，优化器效率高
缺点: 需要实现转换函数

// 转换函数
Eigen::VectorXd anglesToConfig(const Eigen::VectorXd& angles, const Model& model) {
    Eigen::VectorXd q(model.nq);
    int q_idx = 0, v_idx = 0;
    
    for (JointIndex i = 1; i < model.njoints; ++i) {
        if (model.joints[i].shortname() == "JointModelRUB") {
            // 连续关节
            q(q_idx) = std::cos(angles(v_idx));
            q(q_idx+1) = std::sin(angles(v_idx));
            q_idx += 2;
            v_idx += 1;
        } else {
            // 普通关节
            int nq = model.joints[i].nq();
            q.segment(q_idx, nq) = angles.segment(v_idx, nq);
            q_idx += nq;
            v_idx += nq;
        }
    }
    return q;
}
```

### 方案 C: 使用 Pinocchio 的内置函数 (最优雅)

```cpp
// Pinocchio 提供了完整的流形操作:

// 1. 积分 (速度 -> 配置)
pinocchio::integrate(model, q, v * dt, q_new);

// 2. 差分 (配置 -> 速度)
pinocchio::difference(model, q1, q2, v);

// 3. 中性配置 (零配置)
Eigen::VectorXd q0 = pinocchio::neutral(model);

// 在 IK 中使用:
Eigen::VectorXd q = q_init;
for (int iter = 0; iter < max_iter; ++iter) {
    // 计算雅可比和误差
    pinocchio::computeFrameJacobian(model, data, q, tip_frame, J);
    SE3 error = target.actInv(data.oMf[tip_frame]);
    Motion delta_twist = log6(error);
    
    // 求解
    Eigen::VectorXd delta_v = J.completeOrthogonalDecomposition().solve(delta_twist.toVector());
    
    // 更新 (在流形上)
    pinocchio::integrate(model, q, alpha * delta_v, q);  // ✅ 正确！
}
```

## 总结

1. **雅可比矩阵维度不变**: 仍然是 6×6 (对于6-DOF机器人)
2. **关键区别**: 配置空间(7维) vs 速度空间(6维)
3. **需要修改的地方**:
   - 配置更新: 使用 `pinocchio::integrate()` 而不是直接相加
   - 初始化: 使用 `pinocchio::neutral()` 或正确设置 (cos, sin)
   - 优化: 在速度/角度空间优化，然后转换到配置空间

4. **推荐做法**: 使用方案 C (Pinocchio 内置函数)，代码最简洁且物理意义清晰

## 代码修改清单

- [ ] pinocchio_tl.cpp: 使用 `pinocchio::integrate()` 更新配置
- [ ] nlopt_ik.cpp: 决定优化变量空间并实现转换
- [ ] 测试: 验证 IK 收敛性和精度
