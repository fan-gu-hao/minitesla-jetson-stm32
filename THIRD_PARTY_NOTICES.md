# 第三方來源／Third-party notices

此儲存庫包含原工程附帶的第三方程式。其原有授權檔與來源標頭保留於對應目錄，未以本次整理替換或刪除。

This repository includes third-party source shipped with the original engineering project. Existing license files and source headers are retained in their respective directories.

| 來源／Component | 保存位置／Retained notices |
| --- | --- |
| ST STM32F4 HAL | [`firmware/stm32f4/Drivers/STM32F4xx_HAL_Driver/LICENSE.txt`](firmware/stm32f4/Drivers/STM32F4xx_HAL_Driver/LICENSE.txt) |
| CMSIS | [`firmware/stm32f4/Drivers/CMSIS/LICENSE.txt`](firmware/stm32f4/Drivers/CMSIS/LICENSE.txt) |
| ST STM32F4 device support | [`firmware/stm32f4/Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt`](firmware/stm32f4/Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt) |
| CMSIS DSP ComputeLibrary | [`firmware/stm32f4/Drivers/CMSIS/DSP/ComputeLibrary/LICENSE.txt`](firmware/stm32f4/Drivers/CMSIS/DSP/ComputeLibrary/LICENSE.txt) |
| SLAMTEC RPLIDAR SDK | [`jetson/rplidar_sdk-master/LICENSE`](jetson/rplidar_sdk-master/LICENSE) |
| ST VL53L0X API | 原始授權與來源說明保留於 [`main.c`](firmware/stm32f4/Core/Src/main.c) 內嵌區段。Original notices remain in the embedded source sections. |

PyTorch、torchvision、Ultralytics、OpenCV、NumPy 與 pySerial 等執行環境依賴由其各自專案提供；此處保存應用程式與建置紀錄，不包含完整套件安裝環境。

Runtime dependencies such as PyTorch, torchvision, Ultralytics, OpenCV, NumPy, and pySerial are supplied by their respective projects. This repository preserves application code and build records, not a complete installed dependency environment.

本次發布未替自有程式或模型新增整個專案的授權條款，也未宣稱第三方內容為本專案原創。

No additional project-wide license is assigned to first-party code or the model in this publication. Third-party content is not presented as original work of this project.
