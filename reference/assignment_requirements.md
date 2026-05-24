# 作业要求汇总

> 本文件汇总自 Assignment 1 和 Assignment 2 的完整要求。

---

## Assignment 1: AFSM 状态图（50%）

### 基本信息
- 标题：Drawing an AFSM state diagram
- 权重：50%
- 类型：个人作业

### 任务要求

绘制 AFSM 状态图并提供状态描述文档。

控制器必须实现以下功能：

1. **Obstacle Avoidance**：使用超声波传感器避障，在任何状态（除Target Detected State）下都必须执行。

2. **Explore State**：
   - 机器人探索环境
   - 同时使用颜色传感器检测红色目标
   - 如果 simulated charge 满（刚开始），检测到红色 → target detected
   - 探索 2 分钟未检测到红色 → low charge

3. **Low Charge State**：
   - 继续探索并检测红色目标
   - 检测到红色 → target detected
   - 使用 LDR 检测高强度光源
   - 检测到高亮光 → 停止，进入 charging state
   - 充电 1 分钟后 → 返回 explore
   - 光照中断（未满 1 分钟）→ 返回 low charge 探索

4. **Target Detected State**：
   - 检测到红色目标
   - 机器人必须停止
   - 闪烁板载 LED

### 伪代码参考

```
if State == explore
    while explore timer Not = 2 minute AND target Not = found
        if object detection == False
            go straight
        else
            make a turn
    if target == found
        change state to "target detected"
    else
        reset explore timer to 0
        change state to "low charge"
```

### 提交要求
- 使用 draw.io（推荐）或手绘
- 提交 PDF 格式
- 包含：状态图 + 各状态描述（states, transitions, timers）

### 评分标准
| 项目 | 权重 | 说明 |
| --- | --- | --- |
| Logic in Graph | 40% | 图形逻辑正确，符合 brief |
| Optimised Logic | 30% | 图形精简，只含必要的状态/条件/块 |
| Descriptions | 30% | 所有元素描述清晰，格式正确 |

---

## Assignment 2: AFSM 实际实现（50%）

### 基本信息
- 标题：Practical implementation of an AFSM
- 权重：50%
- 类型：个人作业

### 任务要求

开发并实现 AFSM 控制系统。核心目标（按重要性排序）：

1. **避障**（最高优先级）：
   - 通过检测物体决定直行或转向
   - robot safety first

2. **持续探索 2 分钟**：
   - 除非找到红色目标（颜色传感器）
   - 或进入 low charge 状态

3. **Low charge 状态下检测高亮光**：
   - 使用 LDR 检测
   - 同时继续探索
   - 检测到红色目标则进入 target detected
   - 检测到高亮光则停止充电 1 分钟

4. **充电完成后返回探索**：
   - 高亮光下持续 1 分钟 → 返回 explore
   - 充电中断（光照消失）→ 返回 low charge

5. **检测到红色目标**：
   - 停止
   - 闪烁 LED

### 设计理念

> "We aim for self preservation first (not hitting obstacles) and carrying out exploration at other times, while keeping itself charged as well."

光源可在 arena 任何位置，机器人只在光强度高（在光源下方）时停止。

### 提交要求
- Canvas 提交 Python 代码文件
- 上传 3 分钟视频 或 实验室现场演示

### 评分标准
| 项目 | 权重 | 说明 |
| --- | --- | --- |
| Controller design | 25% | 控制器能检测条件和状态 |
| Optimised design | 25% | 控制器精简，无冗余代码 |
| Demonstration | 25% | 演示清晰完整，展示所有状态 |
| Submission | 25% | 格式正确 |

---

## 关键设计要点总结

1. **优先级**：避障 > 探索 > 充电 > 目标检测响应
2. **计时**：探索 2 分钟，充电 1 分钟
3. **非主动搜索**：机器人不主动寻找光源或目标，只在遇到时响应
4. **代码要求**：optimised，无冗余，使用 AFSM 结构（States + Timers + Transitions）
5. **演示**：必须展示所有状态和转换
6. **终止条件**：检测到红色目标为最终状态

---

## 迟交政策

- 每迟 1 天扣 10 分（直到 40 分为止）
- 迟 5 天以上记 0 分
- 补交作业（referred）迟交记 0 分
