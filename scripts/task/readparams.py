#!/usr/bin/env python3
"""
readparams.py ―― 轻量级参数读取模块
────────────────────────────────────────────────────────────
同目录下需有一个 **params.txt**，内容形如：

    p1=3,p2=3,p3=3,p4=3

本模块提供五个公开函数：

    get_p1()       → int
    get_p2()       → int
    get_p3()       → int
    get_p4()       → int
    get_all()      → dict[str, int]

特点
• 只依赖标准库；首次调用后结果缓存，后续读取无需再次 I/O  
• 键不存在或文件缺失时统一返回 0（或空字典），避免崩溃  
• 函数签名简洁，便于在其他脚本中直接 import 使用
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Dict

# params.txt 路径：与本文件同目录
_PARAMS_PATH = Path(__file__).with_name("params.txt")


# ─────────────────── 内部工具 ────────────────────
@lru_cache(maxsize=1)
def _load() -> Dict[str, int]:
    """
    从 params.txt 解析键值对并缓存。
    若文件不存在或解析异常，返回空 dict。
    """
    try:
        raw = _PARAMS_PATH.read_text().strip()
        kv_pairs = (p.split("=", 1) for p in raw.split(",") if "=" in p)
        return {k.strip(): int(v) for k, v in kv_pairs if v.strip().isdigit()}
    except FileNotFoundError:
        return {}
    except Exception:
        return {}


def _get(key: str) -> int:
    """内部统一取值，缺失时返回 0"""
    return _load().get(key, 0)


# ──────────────────── 公共 API ────────────────────
def get_p1() -> int:  # noqa: D401
    """返回 p1 数值；文件缺失或无该键时返回 0"""
    return _get("p1")


def get_p2() -> int:
    """返回 p2 数值"""
    return _get("p2")


def get_p3() -> int:
    """返回 p3 数值"""
    return _get("p3")


def get_p4() -> int:
    """返回 p4 数值"""
    return _get("p4")


def get_all() -> Dict[str, int]:
    """一次性返回四个参数的字典；缺失键自动补 0"""
    d = _load()
    return {f"p{i}": d.get(f"p{i}", 0) for i in range(1, 5)}


# ─────────────────── CLI / 测试 ───────────────────
if __name__ == "__main__":
    import json
    print("p1 =", get_p1())
    print("p2 =", get_p2())
    print("p3 =", get_p3())
    print("p4 =", get_p4())
    print("all =", json.dumps(get_all(), indent=2))
