
https://mp.weixin.qq.com/s/W3gmYDqqMNIKNTIa3aGeCA

Claude 用 Skills 来“把特定任务做得更专业”。一个 Skill 就是一个文件夹，内含指令、脚本与资源；Claude 在需要时动态加载。整个系统采用“声明式、纯提示”的发现与调用逻辑：

术语含义Skill 工具（首字母大写）出现在工具数组里的唯一元工具，负责加载所有技能。skills（小写）单个技能，如 pdf、skill-creator、internal-comms，本质上是一段待注入的提示模板。


下表区分 Tools 与 Skills 及其能力：
![alt text](image.png)

每个Skill都在一个名为SKILL.md（区分大小写）的markdown文件中定义，其中包含技能的元数据。SKILL.md文件可以是Python脚本、shell脚本、字体定义、模板等。使用skill-creator作为示例，它包含SKILL.md、LICENSE.txt for the license，以及tech/scripts文件夹skill-creator：不包含任何/references或/assets。




https://mp.weixin.qq.com/s/J4F5-_FI2Q4JtqSuonPsTg

统一框架：把“演化”抽象成四大模块

图 3：系统输入 → 智能体系统 → 环境反馈 → 优化器 → 回到系统

模块职责示例System Inputs定义任务/数据/约束金融问答、代码修复Agent System被优化的“本体”LLM + 提示 + 记忆 + 工具Environment提供可量化反馈单元测试结果、人类评分Optimiser搜索更好配置贝叶斯优化、RL、进化算法


2 记忆不再“金鱼”：Memory Optimisation短程记忆长程记忆递归摘要、动态过滤外挂向量库、知识图谱、遗忘曲线代表工作：MemGPT、HippoRAG、A-MEM、MemoryBank

3 工具“自生”：Tool OptimisationTraining-Based：ToolLLM、Confucius、ReTool（用 RL 学调用）Inference-Time：EASYTOOL 把 100+ API 文档压缩成 1 句人话Tool Creation：CREATOR、LATM、Alita——直接让 Agent 写代码造新工具


图 6：多智能体工作流搜索空间 vs 优化算法 vs 目标（准/快/省/安全）演化维度做法代表Prompt 级多 Agent 提示一起搜DSPy、AutoAgentsTopology 级把“谁跟谁说话”变成可微边GPTSwarm、DynaSwarm、G-Designer统一联合提示 + 拓扑 + 工具同时搜ADAS、EvoFlow、MAS-ZEROBackbone 级用对抗轨迹继续 SFT/RLMaPoRL、OPTIMA、Sirius