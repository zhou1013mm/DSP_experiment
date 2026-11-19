# -*- coding: utf-8 -*-
"""检查 E5 后处理生成的图是否非空白（统计非白/深色像素比例，不依赖 read_image）。"""
import glob
import os

import numpy as np
from PIL import Image

FIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analysis_figs")
assert os.path.isdir(FIG), FIG
paths = sorted(glob.glob(os.path.join(FIG, "*.png")))
print(f"图目录: {FIG}  共 {len(paths)} 张")
ok = True
for p in paths:
    im = Image.open(p).convert("L")
    a = np.asarray(im, dtype=np.int16)
    non_white = float(np.mean(a < 250))
    dark = float(np.mean(a < 128))
    colored = np.asarray(Image.open(p).convert("RGB"), dtype=np.int16)
    sat = float(np.mean(np.max(colored, axis=2) - np.min(colored, axis=2) > 25))
    flag = "OK" if non_white > 0.005 and dark > 0.0002 else "疑似空白!"
    if flag != "OK":
        ok = False
    print(f"{os.path.basename(p):<28s} {im.size[0]}x{im.size[1]}  "
          f"{os.path.getsize(p) / 1024:6.1f} KiB  "
          f"非白={non_white * 100:5.1f}%  深色={dark * 100:5.2f}%  "
          f"彩色={sat * 100:4.1f}%  {flag}")
print("RESULT:", "ALL OK" if ok else "HAS BLANK/FAILED")
raise SystemExit(0 if ok else 1)
