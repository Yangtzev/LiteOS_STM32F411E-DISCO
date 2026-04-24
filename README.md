# LiteOS for STM32F411E-DISCO
### 概述
LiteOS 是一个为嵌入式系统设计的轻量级操作系统。本项目基于[野火]《物联网操作系统 LiteOS开发实战指南》STM32F407系列源码，专为 STM32F411E-DISCO 开发板定制，包含支持实时应用开发的各种模块和驱动。

### 特性
- 为嵌入式系统优化的轻量级Huawei LiteOS内核
- 支持 STM32F411E-DISCO 开发板
- 模块化架构，包含 CMSIS 和 HAL 驱动
- 实时任务调度
- 内存管理
- 外设驱动（如加速度计、USART 等）
- 包含智感姿态识别模型，模型主页https://github.com/STMicroelectronics/stm32ai-modelzoo/tree/main/human_activity_recognition

### 项目结构
- **Libraries**: 包含 CMSIS 和 HAL 驱动
- **Listings**: 汇编和启动文件
- **LiteOS**: 核心内核和架构相关代码
- **Output**: 构建产物和日志
- **Project**: 用户定义的项目文件
- **User**: 应用相关代码

### 快速开始
1. 克隆仓库：
   ```bash
   git clone https://github.com/Yangtzev/LiteOS_STM32F411E-DISCO.git
   ```
2. 使用Keil MDK 5打开Project目录下的项目工程。
3. 编译项目并将其下载到 STM32F411E-DISCO 开发板。
4. 参考文档获取详细使用说明。

### 环境要求
- STM32F411E-DISCO 开发板
- Keil MDK 或 STM32CubeIDE（本项目使用Arm Compiler 5.x编译）
- 用于编程的 USB 数据线，串口线

### 许可证
本项目使用 MIT 许可证授权。详情请参阅 LICENSE 文件。
