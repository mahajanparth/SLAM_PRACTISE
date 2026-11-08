#!/usr/bin/env python3
"""SLAM Practice — local HackerRank-style judge.

Usage:  python3 app/server.py [--port 8008]
Then open http://127.0.0.1:8008

Stdlib only. Binds to 127.0.0.1 (the terminal endpoint runs shell commands — never expose it).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

ROOT = Path(__file__).resolve().parent.parent
STATIC = ROOT / "app" / "static"
PROBLEMS = ROOT / "problems"
NOTES = ROOT / "notes"
WORKSPACE = ROOT / "workspace"
BUILD = ROOT / "build" / "web"
PROGRESS = WORKSPACE / "progress.json"

COMPILE_TIMEOUT = 90
RUN_TIMEOUT = 15
TERM_TIMEOUT = 60
MAX_OUT = 200_000

CXX = os.environ.get("CXX", "g++")
BASE_FLAGS = ["-std=c++17", "-Wall", "-Wextra", f"-I{ROOT}", "-I/usr/include/eigen3", "-pthread"]
DEBUG_FLAGS = ["-O0", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
ID_RE = re.compile(r"^[A-Za-z0-9_\-]+$")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
SUMMARY_RE = re.compile(r"(\d+) passed, (\d+) failed")

_lock = threading.Lock()
_term_cwd = {"cwd": str(ROOT)}


# ---------------------------------------------------------------- helpers
def read_json(p: Path, default=None):
    try:
        return json.loads(p.read_text())
    except Exception:
        return default


def load_progress():
    return read_json(PROGRESS, {}) or {}


def save_progress(prog):
    WORKSPACE.mkdir(exist_ok=True)
    tmp = PROGRESS.with_suffix(".tmp")
    tmp.write_text(json.dumps(prog, indent=2))
    tmp.replace(PROGRESS)


def record(pid, **fields):
    with _lock:
        prog = load_progress()
        entry = prog.setdefault(pid, {"status": "attempted", "attempts": 0})
        for k, v in fields.items():
            if k == "status":
                # never downgrade a solved problem
                if entry.get("status") == "solved" and v != "solved":
                    continue
            entry[k] = v
        entry["updated"] = time.strftime("%Y-%m-%d %H:%M")
        save_progress(prog)
        return entry


def problem_dir(pid):
    if not ID_RE.match(pid or ""):
        return None
    d = PROBLEMS / pid
    return d if (d / "meta.json").exists() else None


def list_problems():
    out = []
    for meta_path in sorted(PROBLEMS.glob("*/meta.json")):
        meta = read_json(meta_path)
        if not meta:
            continue
        meta["id"] = meta_path.parent.name
        out.append(meta)
    return out


def truncate(s):
    if len(s) > MAX_OUT:
        return s[:MAX_OUT] + f"\n... [output truncated at {MAX_OUT} chars]"
    return s


def run_proc(cmd, timeout, cwd=None, env=None):
    t0 = time.time()
    try:
        p = subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, timeout=timeout)
        out = p.stdout.decode(errors="replace") + p.stderr.decode(errors="replace")
        return p.returncode, truncate(out), time.time() - t0, False
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode(errors="replace") + (e.stderr or b"").decode(errors="replace")
        return -1, truncate(out), time.time() - t0, True


def compile_and_run(pid, code, mode, src=None, tag=""):
    """mode: run (public tests) | submit (public+hidden) | scratch (code has its own main).
    src: where to write the code (defaults to workspace/<pid>.cpp, the user's saved answer)."""
    work = BUILD / pid
    work.mkdir(parents=True, exist_ok=True)
    src = Path(src) if src else WORKSPACE / f"{pid}.cpp"
    src.parent.mkdir(parents=True, exist_ok=True)
    src.write_text(code)
    exe = work / f"{mode}{tag}.out"

    if mode == "scratch":
        cmd = [CXX, *BASE_FLAGS, *DEBUG_FLAGS, str(src), "-o", str(exe)]
    else:
        pdir = problem_dir(pid)
        tests = pdir / "tests.cpp"
        cmd = [CXX, *BASE_FLAGS, *DEBUG_FLAGS, "-include", str(src)]
        if mode == "submit":
            cmd.append("-DHIDDEN")
        cmd += [str(tests), "-o", str(exe)]

    rc, cout, ctime, cto = run_proc(cmd, COMPILE_TIMEOUT, cwd=str(ROOT))
    res = {"mode": mode, "compile_time": round(ctime, 2), "compile_output": cout.replace(str(ROOT) + "/", "")}
    if cto or rc != 0:
        res.update(status="compile_error" if not cto else "compile_timeout", run_output="", passed=0, failed=0)
        return res

    env = dict(os.environ, ASAN_OPTIONS="detect_leaks=1:abort_on_error=0", UBSAN_OPTIONS="print_stacktrace=1")
    run_cmd = [str(exe)]
    if shutil.which("setarch"):  # GCC11 ASan + kernel 6.x high mmap_rnd_bits -> random crashes w/o this
        run_cmd = ["setarch", os.uname().machine, "-R", str(exe)]
    rc, rout, rtime, rto = run_proc(run_cmd, RUN_TIMEOUT, cwd=str(ROOT), env=env)
    rout = rout.replace(str(ROOT) + "/", "")
    res.update(run_output=rout, run_time=round(rtime, 3), exit_code=rc)

    clean = ANSI_RE.sub("", rout)
    m = SUMMARY_RE.findall(clean)
    passed, failed = (int(m[-1][0]), int(m[-1][1])) if m else (0, 0)
    res.update(passed=passed, failed=failed)
    sanitizer = "AddressSanitizer" in clean or "runtime error:" in clean or "LeakSanitizer" in clean

    if rto:
        status = "timeout"
    elif mode == "scratch":
        status = "runtime_error" if (rc != 0 or sanitizer) else "ok"
    elif sanitizer:
        status = "sanitizer_error"
    elif not m:
        status = "runtime_error"
    elif failed > 0:
        status = "wrong_answer"
    elif rc != 0:
        status = "runtime_error"
    else:
        status = "accepted"
    res["status"] = status
    return res


def grade_quiz(pid, answers):
    pdir = problem_dir(pid)
    quiz = read_json(pdir / "quiz.json", {})
    qs = quiz.get("questions", [])
    results = []
    score = 0
    for i, q in enumerate(qs):
        given = answers.get(str(i)) if isinstance(answers, dict) else None
        ok = given is not None and int(given) == int(q["answer"])
        score += ok
        results.append({"correct": ok, "answer": q["answer"], "explanation": q.get("explanation", "")})
    return {"score": score, "total": len(qs), "results": results}


def run_terminal(cmd):
    cwd = _term_cwd["cwd"]
    marker = "__SLAMPRACTICE_CWD__"
    script = f"{cmd}\n__rc=$?\necho; echo {marker}; pwd; exit $__rc"
    env = dict(os.environ, TERM="dumb", CLICOLOR_FORCE="1")
    rc, out, _, to = run_proc(["bash", "-c", script], TERM_TIMEOUT, cwd=cwd, env=env)
    if marker in out:
        body, _, tail = out.rpartition(marker)
        newcwd = tail.strip().splitlines()[0] if tail.strip() else cwd
        if os.path.isdir(newcwd):
            _term_cwd["cwd"] = newcwd
        out = body.rstrip("\n")
        if out.endswith("\n"):
            out = out[:-1]
    if to:
        out += f"\n[timed out after {TERM_TIMEOUT}s]"
    cwd_disp = _term_cwd["cwd"].replace(str(Path.home()), "~")
    return {"output": out, "exit_code": rc, "cwd": cwd_disp}


# ---------------------------------------------------------------- HTTP
class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(STATIC), **kw)

    def log_message(self, fmt, *args):  # quieter
        if "/api/" in (self.path or ""):
            return
        super().log_message(fmt, *args)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _json(self, obj, code=200):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n > 2_000_000:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            return {}

    def _local_only(self):
        host = (self.headers.get("Host") or "").split(":")[0]
        if host not in ("127.0.0.1", "localhost"):
            self._json({"error": "forbidden"}, 403)
            return False
        return True

    # GET ------------------------------------------------------------
    def do_GET(self):
        path = urlparse(self.path).path
        if not path.startswith("/api/"):
            return super().do_GET()
        if not self._local_only():
            return
        parts = [unquote(p) for p in path.split("/")[2:]]
        try:
            if parts == ["catalog"]:
                modules = read_json(PROBLEMS / "modules.json", {"tracks": []})
                return self._json({"modules": modules, "problems": list_problems(), "progress": load_progress()})
            if len(parts) == 2 and parts[0] == "problem":
                pdir = problem_dir(parts[1])
                if not pdir:
                    return self._json({"error": "not found"}, 404)
                meta = read_json(pdir / "meta.json", {})
                meta["id"] = parts[1]
                saved = WORKSPACE / f"{parts[1]}.cpp"
                starter = (pdir / "starter.cpp").read_text() if (pdir / "starter.cpp").exists() else ""
                quiz = read_json(pdir / "quiz.json")
                if quiz:  # strip answers before sending
                    quiz = {"questions": [{k: v for k, v in q.items() if k not in ("answer", "explanation")}
                                          for q in quiz.get("questions", [])]}
                return self._json({
                    "meta": meta,
                    "statement": (pdir / "statement.md").read_text() if (pdir / "statement.md").exists() else "",
                    "starter": starter,
                    "code": saved.read_text() if saved.exists() else starter,
                    "quiz": quiz,
                    "has_solution": (pdir / "solution.cpp").exists(),
                    "progress": load_progress().get(parts[1]),
                })
            if len(parts) == 2 and parts[0] == "solution":
                pdir = problem_dir(parts[1])
                sol = pdir / "solution.cpp" if pdir else None
                if not sol or not sol.exists():
                    return self._json({"error": "no solution"}, 404)
                record(parts[1], solution_viewed=True)
                return self._json({"code": sol.read_text()})
            if len(parts) == 2 and parts[0] == "notes":
                if not ID_RE.match(parts[1]):
                    return self._json({"error": "bad id"}, 400)
                f = NOTES / f"{parts[1]}.md"
                return self._json({"markdown": f.read_text() if f.exists() else ""})
            return self._json({"error": "unknown endpoint"}, 404)
        except Exception as e:  # pragma: no cover
            return self._json({"error": str(e)}, 500)

    # POST -----------------------------------------------------------
    def do_POST(self):
        if not self._local_only():
            return
        path = urlparse(self.path).path
        body = self._body()
        pid = body.get("id", "")
        try:
            if path == "/api/save":
                if not problem_dir(pid) and pid != "scratchpad":
                    return self._json({"error": "bad id"}, 400)
                WORKSPACE.mkdir(exist_ok=True)
                (WORKSPACE / f"{pid}.cpp").write_text(body.get("code", ""))
                return self._json({"ok": True})
            if path == "/api/run":
                mode = body.get("mode", "run")
                if pid == "scratchpad":
                    mode = "scratch"
                elif not problem_dir(pid) or mode not in ("run", "submit"):
                    return self._json({"error": "bad request"}, 400)
                res = compile_and_run(pid, body.get("code", ""), mode)
                if pid != "scratchpad":
                    prog = load_progress().get(pid, {})
                    fields = {"attempts": prog.get("attempts", 0) + 1, "last_status": res["status"]}
                    if mode == "submit":
                        fields["status"] = "solved" if res["status"] == "accepted" else "attempted"
                        if res["status"] == "accepted" and body.get("elapsed"):
                            best = prog.get("best_time_s")
                            el = int(body["elapsed"])
                            fields["best_time_s"] = el if best is None else min(best, el)
                    else:
                        fields["status"] = "attempted"
                    res["progress"] = record(pid, **fields)
                return self._json(res)
            if path == "/api/quiz":
                if not problem_dir(pid):
                    return self._json({"error": "bad id"}, 400)
                g = grade_quiz(pid, body.get("answers", {}))
                prog = load_progress().get(pid, {})
                best = max(prog.get("best_score", 0), g["score"])
                g["progress"] = record(pid, status="solved" if g["score"] == g["total"] else "attempted",
                                       best_score=best, total=g["total"],
                                       attempts=prog.get("attempts", 0) + 1)
                return self._json(g)
            if path == "/api/reset":
                f = WORKSPACE / f"{pid}.cpp"
                if ID_RE.match(pid) and f.exists():
                    f.unlink()
                return self._json({"ok": True})
            if path == "/api/term":
                cmd = body.get("cmd", "")
                if body.get("reset_cwd"):
                    _term_cwd["cwd"] = str(ROOT)
                return self._json(run_terminal(cmd) if cmd.strip() else {"output": "", "cwd": _term_cwd["cwd"]})
            return self._json({"error": "unknown endpoint"}, 404)
        except Exception as e:  # pragma: no cover
            return self._json({"error": str(e)}, 500)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8008)
    args = ap.parse_args()
    WORKSPACE.mkdir(exist_ok=True)
    BUILD.mkdir(parents=True, exist_ok=True)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"SLAM Practice running at http://127.0.0.1:{args.port}  (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
