# 高分辨率 CJK 字体指引

为了让 C3 UI 的中文/日文/韩文字体更清晰，我们在 `selfdrive/assets/fonts/process.py` 中做了以下更新：

1. 引入 `os`，并允许用环境变量控制 fallback 字体尺寸（默认为 64 px）：
   ```python
   import os
   ...
   font_size = {
     # Allow overriding unifont size via env (default to high-res 64px for smoother fallback glyphs)
     "unifont.otf": int(os.getenv("UNIFONT_SIZE", "64")),
   }.get(font_path.name, 200)
   ```

2. 生成命令可以在任意 shell 内执行，推荐步骤如下：

   ```bash
   cd /mnt/e/openpilot/selfdrive/assets/fonts

   # 可选：设置希望的字号，比如 72（单位 px）
   export UNIFONT_SIZE=72

   # 生成 .png/.fnt（含 Inter + unifont）
   python process.py
   ```

3. 脚本会覆盖 `selfdrive/assets/fonts/unifont.fnt` 与 `unifont.png`，记得将更新后的文件同步到目标设备。例如：
   ```bash
   rsync -av selfdrive/assets/fonts/ comma@comma-device:/data/openpilot/selfdrive/assets/fonts/
   ```

4. 重启 UI（或整机）即可看到效果。

> 注意：`UNIFONT_SIZE` 不设置时默认 64 px；若想恢复上游 16 px 点阵，可执行 `UNIFONT_SIZE=16 python process.py`。

后续如需再次生成，只需重复第 2 步即可；代码部分无需再次修改。这样 AI/开发者都能直接复用同样的流程。
