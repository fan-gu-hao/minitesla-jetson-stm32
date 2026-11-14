# 原始建置紀錄／Original build logs

本目錄保留原始紀錄內容，沒有把 `.log` 加入忽略清單，也沒有改寫錯誤、警告或版本資訊。

Original log contents are preserved. `.log` files are intentionally tracked, and errors, warnings, and version information are not rewritten.

| 檔案／File | 能佐證的內容／Evidence |
| --- | --- |
| [`torchvision_build.log`](torchvision_build.log) | `torchvision-0.16.1+fdea156`、Python 3.8／Linux AArch64、`nvcc` 編譯 CUDA 擴充，以及成功建置／安裝訊息。Source-build and successful-installation record. |
| [`m4/wheelcontrol.build_log.htm`](m4/wheelcontrol.build_log.htm) | 原 M4 工程保存的歷史 Keil 建置紀錄，不代表本版四向 ToF 原始碼已重新建置。Historical Keil build record, not a fresh build of current firmware. |

`torchvision` 紀錄顯示 `FORCE_CUDA: False`，同時也包含 `nvcc` 與 `-DWITH_CUDA` 編譯行；因此不以單一環境旗標判定是否編譯 CUDA。這是當時一次建置的證據，不足以單獨重建整個 JetPack／PyTorch 相容矩陣。

The log shows `FORCE_CUDA: False` alongside actual `nvcc`/`-DWITH_CUDA` compilation commands. The environment flag alone does not determine whether CUDA extensions were built. This records one historical build and does not establish the complete installed JetPack/PyTorch compatibility matrix.

[`notes/compile_torchvision.txt`](../notes/compile_torchvision.txt) 記載下載 `v0.15.1` 的指令，而保存的建置紀錄是 `0.16.1+fdea156`；兩者作為不同歷史資料保留，不宣稱是完全相同的一次操作。

The recovered command note downloads `v0.15.1`, while the retained build log identifies `0.16.1+fdea156`. Both are preserved as historical records without claiming they describe exactly the same build.

本次沒有收到可發布的新行駛紀錄或完整影片；`demo_recorder.py` 可於之後展示時產生新的紀錄。

No new driving log or complete demonstration video was supplied for this publication. The recorder can produce new records during later demonstrations.
