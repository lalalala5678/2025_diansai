# app.py
# 简洁版 Flask 后端：参数调节 + 任务执行 + 一键保存参数
# 仅依赖 Flask（pip install Flask==3.*）

from flask import Flask, render_template, request, jsonify
import subprocess
import os
import pathlib

app = Flask(__name__)

# 1) 四个可调参数，初始值随意设 0
PARAMS = {"p1": 0, "p2": 0, "p3": 0, "p4": 0}

# 2) 四个任务对应要执行的脚本或命令
TASKS = {
    "task1": "bash ./scripts/task1.sh",
    "task2": "bash ./scripts/task2.sh",
    "task3": "bash ./scripts/task3.sh",
    "task4": "bash ./scripts/task4.sh",
}

# -------------------- 页面路由 -------------------- #
@app.route("/")
def params_page():
    """参数调节界面"""
    return render_template("params.html")


@app.route("/tasks")
def tasks_page():
    """任务执行界面"""
    return render_template("tasks.html")


# -------------------- 参数 API -------------------- #
@app.route("/api/params", methods=["GET"])
def get_params():
    return jsonify(PARAMS)


@app.route("/api/param/<name>", methods=["POST"])
def update_param(name):
    """body: {"delta": ±1}"""
    if name not in PARAMS:
        return jsonify({"error": "unknown param"}), 400
    data = request.get_json(force=True)
    delta = int(data.get("delta", 0))
    PARAMS[name] += delta
    return jsonify(PARAMS)


# ---------- 保存参数到 scripts/task/params.txt ---------- #
@app.route("/api/save", methods=["POST"])
def save_params():
    """把当前 PARAMS 写入 params.txt，格式 p1=...,p2=...,p3=...,p4=..."""
    txt_path = pathlib.Path(__file__).parent / "scripts" / "task" / "params.txt"
    txt_path.parent.mkdir(parents=True, exist_ok=True)

    line = ",".join(f"{k}={v}" for k, v in PARAMS.items())
    txt_path.write_text(line + "\n", encoding="utf-8")

    return "参数已保存到 scripts/task/params.txt"


# -------------------- 任务 API -------------------- #
@app.route("/api/run/<task>", methods=["POST"])
def run_task(task):
    """执行脚本并把结果返回前端"""
    cmd = TASKS.get(task)
    if not cmd:
        return jsonify({"error": "unknown task"}), 400

    result = subprocess.run(
        cmd,
        shell=True,                   # 方便直接写 "./xxx.sh"
        text=True,
        cwd=os.path.dirname(__file__),  # 保证相对路径正确
        capture_output=True,
    )
    return jsonify(
        {
            "exit_code": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    )


# -------------------- 主入口 -------------------- #
if __name__ == "__main__":
    # 开发阶段直接跑；上线可用 gunicorn -w 2 -b 0.0.0.0:8000 app:app
    app.run(host="0.0.0.0", port=5000, debug=True)
