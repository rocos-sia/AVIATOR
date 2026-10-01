"""触觉原始寄存器 → 每个部位的二维矩阵。

地址划分与 `python/touch_data.py` 一致（摘自手册表 56 压阻式）：

    小指/无名指/中指/食指  各 370 字节 = 185 点
        指端 3x3 (9) + 指尖 12x8 (96) + 指腹 10x8 (80)
    拇指                   420 字节 = 210 点
        指端 3x3 (9) + 指尖 12x8 (96) + 指中 3x3 (9) + 指腹 12x8 (96)
    掌心                   224 字节 = 112 点
        14x8（按 touch_data.py 转置为 8x14）
"""
from __future__ import annotations

import numpy as np

from .handlink import TOUCH_MAX

# 手册给的是「每指 370 字节 / 185 个 16 位点」，即一个点占 2 个地址。
# 若实测发现热力图错位，把这里改成 1 即可（配合 handlink 全区间读取）。
TOUCH_STRIDE = 2

FOUR_FINGERS = ["小指", "无名指", "中指", "食指"]


def _take(raw: list[int], stride: int = TOUCH_STRIDE) -> list[int]:
    """把原始寄存器列表按步长抽出触觉点。"""
    vals = list(raw)[::stride]
    return vals


def _sanitize(vals) -> np.ndarray:
    """超量程的值视为无效（固件在未用地址上会返回位标志/遥测值）。"""
    a = np.asarray(vals, dtype=np.float32)
    a[(a < 0) | (a > TOUCH_MAX)] = 0.0
    return a


def parse_finger(name: str, raw: list[int]) -> dict[str, np.ndarray] | None:
    """解析四指 / 拇指。返回含 tip_end / tip_touch / pad 等键的字典。"""
    v = _sanitize(_take(raw))
    if name == "拇指":
        need = 9 + 96 + 9 + 96
        if v.size < need:
            return None
        i = 0
        tip_end = v[i:i + 9].reshape(3, 3); i += 9
        tip_touch = v[i:i + 96].reshape(12, 8); i += 96
        middle = v[i:i + 9].reshape(3, 3); i += 9
        pad = v[i:i + 96].reshape(12, 8)
        return {"tip_end": tip_end, "tip_touch": tip_touch, "middle": middle, "pad": pad}

    need = 9 + 96 + 80
    if v.size < need:
        return None
    i = 0
    tip_end = v[i:i + 9].reshape(3, 3); i += 9
    tip_touch = v[i:i + 96].reshape(12, 8); i += 96
    pad = v[i:i + 80].reshape(10, 8)
    return {"tip_end": tip_end, "tip_touch": tip_touch, "pad": pad}


def parse_palm(raw: list[int]) -> np.ndarray | None:
    """掌心 8 行 x 14 列。

    手册 2.6.21：数据点按列优先、且行从下往上——
    点 1->第 8 行第 1 列，点 2->第 7 行第 1 列，…，点 8->第 1 行第 1 列，
    点 9->第 8 行第 2 列，以此类推。
    所以先按 (14,8) 切再转置得到列优先，最后上下翻转把行序倒过来。
    """
    v = _sanitize(_take(raw))
    if v.size < 112:
        return None
    return v[:112].reshape(14, 8).T[::-1]


def build_image(name: str, parsed: dict[str, np.ndarray]) -> np.ndarray:
    """把分段的矩阵拼成一张整指图像（掌心直接返回）。"""
    if name == "掌心":
        return parsed  # type: ignore[return-value]

    rows: list[np.ndarray] = []
    if name == "拇指":
        order = ["tip_end", "tip_touch", "middle", "pad"]
    else:
        order = ["tip_end", "tip_touch", "pad"]

    for key in order:
        blk = parsed[key]
        if blk.shape[1] == 8:
            rows.append(blk)
        else:  # 3x3 指端居中放进 8 列
            pad = np.zeros((blk.shape[0], 8), dtype=np.float32)
            off = (8 - blk.shape[1]) // 2
            pad[:, off:off + blk.shape[1]] = blk
            rows.append(pad)
    return np.vstack(rows)
