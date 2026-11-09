#!/usr/bin/env python3
"""SLAM Practice — local HackerRank-style judge.

Usage:  python3 app/server.py [--port 8008]
Then open http://127.0.0.1:8008

Stdlib only. Binds to 127.0.0.1 (the terminal endpoint runs shell commands — never expose it).
"""
import argparse
import glob
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

CLAUDE_TIMEOUT = 300
CLAUDE_MODEL = os.environ.get("CLAUDE_MODEL", "")  # empty = your Claude Code default

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


# ---------------------------------------------------------------- Ask Claude
def find_claude():
    """Locate the Claude Code CLI: $CLAUDE_BIN, PATH, then the VS Code extension's bundled binary."""
    cands = [os.environ.get("CLAUDE_BIN"), shutil.which("claude"),
             str(Path.home() / ".claude" / "local" / "claude"), str(Path.home() / ".local" / "bin" / "claude")]
    for pattern in ("~/.vscode/extensions/anthropic.claude-code-*/resources/native-binary/claude",
                    "~/.cursor/extensions/anthropic.claude-code-*/resources/native-binary/claude"):
        cands += sorted(glob.glob(os.path.expanduser(pattern)), key=os.path.getmtime, reverse=True)
    for c in cands:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


CLAUDE_SYSTEM = """You are a senior robotics/SLAM engineer acting as a live-coding interviewer and code reviewer \
inside a local C++ practice judge. The candidate is preparing for a C++17 SLAM interview.

Rules:
- Allowed in solutions: C++17 standard library + Eigen only (no PCL, Sophus, Ceres).
- Give feedback on the candidate's CURRENT code: correctness bugs, edge cases, complexity (time/space), \
numerical issues, and C++ quality (const-correctness, copies, references, reserve, UB, naming).
- Cite line numbers from the numbered code (e.g. "L23"). Show tiny snippets only for the specific fix.
- Do NOT write a complete solution unless the candidate explicitly asks for one. Prefer pointing at the bug \
and asking a probing question ("what happens when N = 1?").
- Be concise, direct, and honest — no over-praise. Use short Markdown sections/bullets.
- If the code has no obvious bugs, say so, then give 1-3 follow-up questions an interviewer would ask."""

CLAUDE_MOCK = """
MOCK INTERVIEW IN PROGRESS: behave strictly like the interviewer. Never write code or reveal the fix. \
Answer only with clarifications, a nudge, or a probing question, as a real interviewer would."""


def build_claude_prompt(body):
    pid = body.get("id", "")
    pdir = problem_dir(pid)
    title, statement = "Scratchpad (free-form C++ with its own main())", ""
    if pdir:
        meta = read_json(pdir / "meta.json", {})
        title = f"{meta.get('title', pid)} [{pid}, {meta.get('difficulty', '')}]"
        st = pdir / "statement.md"
        statement = st.read_text() if st.exists() else ""
    elif pid != "scratchpad":
        title = "No problem open (general question)"
    code = str(body.get("code", ""))[:60_000]
    numbered = "\n".join(f"{i + 1:4d}| {l}" for i, l in enumerate(code.split("\n"))) if code.strip() else "(no code)"

    parts = [f"## Problem: {title}"]
    if statement:
        parts.append(statement[:20_000])
    parts.append(f"## Candidate's current code\n```cpp\n{numbered}\n```")
    last = body.get("last_result") or {}
    if last.get("status"):
        out = ANSI_RE.sub("", str(last.get("output", "")))[-6000:]
        parts.append(f"## Latest judge result: {last['status']} ({last.get('mode', 'run')}, "
                     f"{last.get('passed', 0)} passed / {last.get('failed', 0)} failed)\n```\n{out}\n```")
    hist = [h for h in (body.get("history") or []) if isinstance(h, dict)][-10:]
    if hist:
        conv = "\n\n".join(f"{'Candidate' if h.get('role') == 'user' else 'You'}: {str(h.get('text', ''))[:4000]}"
                           for h in hist)
        parts.append(f"## Conversation so far\n{conv}")
    parts.append(f"## Candidate's new message\n{str(body.get('question', '')).strip()[:8000]}")
    system = CLAUDE_SYSTEM + (CLAUDE_MOCK if body.get("mock") else "")
    return system, "\n\n".join(parts)


def stream_claude(body, emit):
    """Run `claude -p` with no tools and stream text deltas through emit(dict)."""
    exe = find_claude()
    if not exe:
        return emit({"error": "Claude Code CLI not found. Install it or set CLAUDE_BIN=/path/to/claude."})
    system, prompt = build_claude_prompt(body)
    cmd = [exe, "-p", "--output-format", "stream-json", "--verbose", "--include-partial-messages",
           "--tools", "", "--no-session-persistence", "--setting-sources", "", "--system-prompt", system]
    if CLAUDE_MODEL:
        cmd += ["--model", CLAUDE_MODEL]
    cwd = BUILD / "claude"  # empty dir: no project context is picked up
    cwd.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen(cmd, cwd=str(cwd), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, bufsize=1)
    killer = threading.Timer(CLAUDE_TIMEOUT, proc.kill)
    killer.start()
    got_text, final = False, None
    try:
        proc.stdin.write(prompt)
        proc.stdin.close()
        for line in proc.stdout:
            try:
                ev = json.loads(line)
            except ValueError:
                continue
            if ev.get("type") == "stream_event":
                e = ev.get("event", {})
                if e.get("type") == "content_block_delta" and e.get("delta", {}).get("type") == "text_delta":
                    got_text = True
                    emit({"t": e["delta"]["text"]})
            elif ev.get("type") == "result":
                final = ev
        proc.wait()
    except (BrokenPipeError, ConnectionResetError):
        proc.kill()
        return
    finally:
        killer.cancel()
    if final and final.get("is_error"):
        return emit({"error": str(final.get("result") or "Claude returned an error.")})
    if not got_text:
        if final and final.get("result"):
            emit({"t": final["result"]})
        else:
            err = (proc.stderr.read() or "").strip()[-1500:]
            return emit({"error": err or f"claude exited with code {proc.returncode} (timeout {CLAUDE_TIMEOUT}s?)"})
    emit({"done": True, "cost_usd": (final or {}).get("total_cost_usd"),
          "duration_s": round(((final or {}).get("duration_ms") or 0) / 1000, 1)})


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
            if parts == ["claude", "status"]:
                exe = find_claude()
                return self._json({"available": bool(exe), "bin": exe or ""})
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
            if path == "/api/claude":
                if not str(body.get("question", "")).strip():
                    return self._json({"error": "empty question"}, 400)
                self.send_response(200)
                self.send_header("Content-Type", "application/x-ndjson")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()

                def emit(obj):
                    self.wfile.write((json.dumps(obj) + "\n").encode())
                    self.wfile.flush()
                stream_claude(body, emit)
                return
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
