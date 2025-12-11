# 4麦克风矩形排列 DOA 使用说明

## 当前情况分析

### ESP-SR DOA 的限制
- 当前的 `esp_doa_process()` 函数只支持**2麦克风线性阵列**
- 返回**一维角度**（0-180度），只能确定声音在水平面上的方向
- 对于4麦克风矩形阵列，需要计算**2D角度**（水平和垂直方向）

### 你的硬件配置
- **ES7210**: 支持4通道麦克风输入
- 当前配置: `"MMMM"` 格式，表示4个麦克风通道

## 4麦克风矩形排列的优势

### 阵列布局示例
```
    [Mic1] -------- [Mic2]
      |                |
      |                |
    [Mic3] -------- [Mic4]
```

### 优势
1. **2D角度估计**: 可以同时计算水平和垂直方向的角度
2. **更高的精度**: 更多麦克风提供更多信息
3. **更好的抗干扰**: 冗余数据可以提高鲁棒性

## 实现方案

### 方案1: 使用 AFE DOA（简化版）
如果你只需要水平方向的角度，可以使用：
- 选择一对水平麦克风（Mic1-Mic2 或 Mic3-Mic4）
- 或选择一对垂直麦克风（Mic1-Mic3 或 Mic2-Mic4）

### 方案2: 自定义2D DOA（完整版）
需要自己实现二维角度计算：

#### 步骤1: 分别计算水平和垂直角度

```c
// 伪代码示例
float calculate_2d_doa(int16_t* mic1, int16_t* mic2, 
                       int16_t* mic3, int16_t* mic4, 
                       int frame_size, float mic_spacing_x, float mic_spacing_y) {
    // 计算水平角度（使用 Mic1-Mic2 和 Mic3-Mic4）
    float angle_horizontal_1 = doa_pair_process(mic1, mic2, mic_spacing_x);
    float angle_horizontal_2 = doa_pair_process(mic3, mic4, mic_spacing_x);
    
    // 计算垂直角度（使用 Mic1-Mic3 和 Mic2-Mic4）
    float angle_vertical_1 = doa_pair_process(mic1, mic3, mic_spacing_y);
    float angle_vertical_2 = doa_pair_process(mic2, mic4, mic_spacing_y);
    
    // 融合结果（平均或加权平均）
    float angle_h = (angle_horizontal_1 + angle_horizontal_2) / 2.0f;
    float angle_v = (angle_vertical_1 + angle_vertical_2) / 2.0f;
    
    // 返回2D角度结构
    return {angle_h, angle_v};
}
```

#### 步骤2: 使用现有的 DOA 函数处理麦克风对

```c
// 创建两个 DOA 处理器
doa_handle_t *doa_horizontal = esp_doa_create(16000, 20, 0.06, 1024);  // 水平间距
doa_handle_t *doa_vertical = esp_doa_create(16000, 20, 0.06, 1024);    // 垂直间距

// 分别处理水平和垂直方向
float angle_h = esp_doa_process(doa_horizontal, mic1_data, mic2_data);
float angle_v = esp_doa_process(doa_vertical, mic1_data, mic3_data);
```

#### 步骤3: 融合多个麦克风对的结果

可以使用以下策略：
- **平均值**: 简单平均多个方向估计
- **加权平均**: 根据信噪比加权
- **投票机制**: 选择最一致的结果

## 实际代码示例

### 示例1: 使用 AFE DOA 处理4麦克风（水平方向）

```c
#include "esp_afe_doa.h"

// 在 initSpeechRecog 中初始化
afe_doa_handle_t *doa_handle = afe_doa_create(
    "MMMM",        // 4个麦克风通道
    16000,         // 采样率
    20,            // 角度分辨率（度）
    0.06,          // 麦克风间距（米），根据你的实际间距调整
    1024           // 输入时间数据样本数
);

// 在 detect_Task 中处理
while (1) {
    afe_fetch_result_t* res = afe_handle->fetch(afe_data);
    if (res && res->data) {
        // 假设你有4通道的原始数据
        int16_t *four_channel_data = /* 获取4通道数据 */;
        
        // 处理DOA
        float direction = afe_doa_process(doa_handle, four_channel_data);
        printf("Sound direction: %.1f degrees\n", direction);
    }
}
```

### 示例2: 自定义2D DOA实现

```c
typedef struct {
    float angle_horizontal;  // 水平角度 (0-180度)
    float angle_vertical;    // 垂直角度 (0-180度)
} doa_2d_result_t;

doa_2d_result_t calculate_2d_doa_rectangular(
    int16_t* mic_data,      // 4通道交错的音频数据 [M1,M2,M3,M4,M1,M2,...]
    int frame_size,          // 每通道的样本数
    float mic_spacing_x,     // 水平方向麦克风间距（米）
    float mic_spacing_y      // 垂直方向麦克风间距（米）
) {
    // 分离4个通道的数据
    int16_t *mic1 = /* 提取通道1 */;
    int16_t *mic2 = /* 提取通道2 */;
    int16_t *mic3 = /* 提取通道3 */;
    int16_t *mic4 = /* 提取通道4 */;
    
    // 创建DOA处理器
    static doa_handle_t *doa_h = NULL;
    static doa_handle_t *doa_v = NULL;
    
    if (!doa_h) {
        doa_h = esp_doa_create(16000, 20, mic_spacing_x, frame_size);
    }
    if (!doa_v) {
        doa_v = esp_doa_create(16000, 20, mic_spacing_y, frame_size);
    }
    
    // 计算水平角度（使用上方一对和下方一对的平均值）
    float angle_h1 = esp_doa_process(doa_h, mic1, mic2);
    float angle_h2 = esp_doa_process(doa_h, mic3, mic4);
    float angle_horizontal = (angle_h1 + angle_h2) / 2.0f;
    
    // 计算垂直角度（使用左侧一对和右侧一对的平均值）
    float angle_v1 = esp_doa_process(doa_v, mic1, mic3);
    float angle_v2 = esp_doa_process(doa_v, mic2, mic4);
    float angle_vertical = (angle_v1 + angle_v2) / 2.0f;
    
    doa_2d_result_t result = {
        .angle_horizontal = angle_horizontal,
        .angle_vertical = angle_vertical
    };
    
    return result;
}
```

## 麦克风间距设置

根据你的硬件布局设置间距：

### 典型矩形布局
- **水平间距**: 麦克风1-2 或 3-4 之间的距离（通常 3-6 cm）
- **垂直间距**: 麦克风1-3 或 2-4 之间的距离（通常 3-6 cm）

### 示例配置
```c
// 如果矩形阵列尺寸是 6cm x 6cm
float mic_spacing_x = 0.06;  // 6厘米 = 0.06米
float mic_spacing_y = 0.06;  // 6厘米 = 0.06米

// 如果矩形阵列尺寸是 5cm x 8cm
float mic_spacing_x = 0.05;  // 5厘米
float mic_spacing_y = 0.08;  // 8厘米
```

## 注意事项

1. **麦克风排列顺序**: 确保你的4个通道对应正确的麦克风位置
2. **坐标系统**: 定义好0度的方向（例如：正前方）
3. **性能考虑**: 2D DOA 计算量更大，需要更多CPU资源
4. **精度**: 矩形阵列通常比线性阵列有更好的精度和更少的模糊性

## 建议

对于大多数应用：
- 如果只需要水平方向定位 → 使用方案1（简化版）
- 如果需要完整的2D定位 → 使用方案2（完整版）
- 如果对精度要求很高 → 考虑使用更高级的算法（如 SRP-PHAT 完整实现）

