# GVF引导向量场优化说明

## 概述
本次优化为GVF（Guiding Vector Field）引导向量场添加了自适应参数调整功能，能够根据无人机与引导轨迹的距离动态调整K1和K2参数的影响。

## 核心特性

### 1. 自适应参数调整
- **距离较远时**：增大K2的影响，使无人机更倾向于指向引导轨迹
- **距离较近时**：增大K1的影响，使无人机更倾向于沿着轨迹运行
- **平滑过渡**：使用sigmoid函数实现参数间的平滑过渡

### 2. 可配置参数
所有参数都可以在launch文件中配置：

```xml
<!-- 自适应GVF参数配置 -->
<param name="gvf/adaptive_enabled" value="true" />           <!-- 是否启用自适应功能 -->
<param name="gvf/convergence_bandwidth" value="0.3" />      <!-- 收敛带宽 -->
<param name="gvf/k1_min_scale" value="0.3" />              <!-- K1最小缩放因子 -->
<param name="gvf/k1_max_scale" value="1.2" />              <!-- K1最大缩放因子 -->
<param name="gvf/k2_min_scale" value="0.8" />              <!-- K2最小缩放因子 -->
<param name="gvf/k2_max_scale" value="1.8" />              <!-- K2最大缩放因子 -->
<param name="gvf/debug_output" value="true" />              <!-- 是否输出调试信息 -->
```

## 参数说明

### convergence_bandwidth (收敛带宽)
- 定义：控制参数调整的敏感距离范围
- 默认值：0.3米
- 建议：根据轨迹曲率和无人机速度调整

### K1缩放因子
- **k1_min_scale**：距离较远时K1的缩放因子（默认0.3）
- **k1_max_scale**：距离较近时K1的缩放因子（默认1.2）
- 作用：控制沿轨迹方向的引导强度

### K2缩放因子
- **k2_min_scale**：距离较近时K2的缩放因子（默认0.8）
- **k2_max_scale**：距离较远时K2的缩放因子（默认1.8）
- 作用：控制指向轨迹的吸引力强度

## 工作原理

### 1. 距离归一化
```
normalized_dist = dist / convergence_bandwidth
```

### 2. Sigmoid函数计算
```
sigmoid_factor = 1 / (1 + exp(normalized_dist - 1))
```
- 当dist = convergence_bandwidth时，sigmoid_factor = 0.5
- 当dist < convergence_bandwidth时，sigmoid_factor > 0.5
- 当dist > convergence_bandwidth时，sigmoid_factor < 0.5

### 3. 参数调整
```
effective_K1 = K1 * (k1_min + (k1_max - k1_min) * sigmoid_factor)
effective_K2 = K2 * (k2_max - (k2_max - k2_min) * sigmoid_factor)
```

## 调参建议

### 1. 收敛带宽调整
- **小值（0.1-0.3）**：适合高精度跟踪，参数变化敏感
- **大值（0.5-1.0）**：适合平滑跟踪，参数变化平缓

### 2. K1缩放因子调整
- **k1_min_scale**：建议0.2-0.5，确保远距离时仍有基本引导
- **k1_max_scale**：建议1.0-1.5，避免近距离时过度引导

### 3. K2缩放因子调整
- **k2_min_scale**：建议0.6-1.0，避免近距离时过度吸引
- **k2_max_scale**：建议1.5-2.0，确保远距离时有足够吸引力

## 调试功能

启用debug_output后，系统会每100次调用输出一次调试信息：
```
GVF Debug - Dist: 0.250, r: 0.300, sigmoid: 0.622, K1: 1.000->0.858, K2: -1.000->-1.400
```

## 性能考虑

- 自适应功能会增加少量计算开销
- 建议在生产环境中关闭debug_output以提高性能
- 可以通过设置adaptive_enabled=false快速回退到原始方法

## 故障排除

### 1. 无人机振荡
- 减小convergence_bandwidth
- 调整K1和K2的缩放范围

### 2. 跟踪精度不足
- 增大convergence_bandwidth
- 调整K2_max_scale确保足够吸引力

### 3. 响应过慢
- 减小convergence_bandwidth
- 调整K1_max_scale提高响应速度
