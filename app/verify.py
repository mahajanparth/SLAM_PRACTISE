#!/usr/bin/env python3
"""Validate problems. Usage:
    python3 app/verify.py                 # all problems
    python3 app/verify.py <id> [<id>...]  # specific problems
    python3 app/verify.py --module M04    # all problems in a module

Checks for code problems:
  - meta.json fields, module exists in problems/modules.json
  - statement.md, starter.cpp, tests.cpp, solution.cpp exist
  - starter + tests COMPILES and is NOT accepted (tests must catch the stub)
  - solution + tests: Run and Submit (with hidden tests) both ACCEPTED, sanitizer-clean
Checks for quiz problems: quiz.json well-formed (>=3 questions, valid answer index, explanations).
Never touches workspace/ (the user's code).
"""
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import server  # noqa: E402

REQ_META = ["title", "track", "module", "order", "type", "difficulty", "target_min"]
TMP = server.ROOT / "build" / "verify"


def module_ids():
    mods = server.read_json(server.PROBLEMS / "modules.json", {"tracks": []})
    return {m["id"] for t in mods["tracks"] for m in t["modules"]}


def check(pid, mods):
    errs = []
    pdir = server.PROBLEMS / pid
    meta = server.read_json(pdir / "meta.json")
    if meta is None:
        return [f"meta.json missing/invalid"]
    for k in REQ_META:
        if k not in meta:
            errs.append(f"meta missing '{k}'")
    if meta.get("module") not in mods:
        errs.append(f"module '{meta.get('module')}' not in modules.json")
    if meta.get("difficulty") not in ("Easy", "Medium", "Hard"):
        errs.append("difficulty must be Easy|Medium|Hard")
    if not (pdir / "statement.md").exists():
        errs.append("statement.md missing")

    if meta.get("type") == "quiz":
        quiz = server.read_json(pdir / "quiz.json")
        if not quiz or len(quiz.get("questions", [])) < 3:
            errs.append("quiz.json missing or < 3 questions")
        else:
            for i, q in enumerate(quiz["questions"]):
                opts = q.get("options", [])
                if not q.get("prompt") or len(opts) < 2:
                    errs.append(f"q{i}: prompt/options missing")
                if not isinstance(q.get("answer"), int) or not 0 <= q["answer"] < len(opts):
                    errs.append(f"q{i}: bad answer index")
                if not q.get("explanation"):
                    errs.append(f"q{i}: explanation missing")
        return errs

    if meta.get("type") != "code":
        return errs + ["type must be code|quiz"]
    for f in ("starter.cpp", "tests.cpp", "solution.cpp"):
        if not (pdir / f).exists():
            errs.append(f"{f} missing")
    if errs:
        return errs
    tests = (pdir / "tests.cpp").read_text()
    if "#ifdef HIDDEN" not in tests:
        errs.append("tests.cpp has no '#ifdef HIDDEN' section")

    starter = (pdir / "starter.cpp").read_text()
    r = server.compile_and_run(pid, starter, "run", src=TMP / pid / "starter.cpp", tag="_vstarter")
    if r["status"] in ("compile_error", "compile_timeout"):
        errs.append("starter does not compile:\n" + r["compile_output"][-1500:])
    elif r["status"] == "accepted":
        errs.append("starter is ACCEPTED by public tests (tests too weak)")

    sol = (pdir / "solution.cpp").read_text()
    for mode in ("run", "submit"):
        r = server.compile_and_run(pid, sol, mode, src=TMP / pid / "solution.cpp", tag="_vsol")
        if r["status"] != "accepted":
            detail = r["compile_output"][-1500:] if "compile" in r["status"] else r["run_output"][-1500:]
            errs.append(f"solution {mode}: {r['status']}\n{detail}")
            break
        if mode == "submit" and r["passed"] < 3:
            errs.append(f"only {r['passed']} checks in submit — add more tests")
    return errs


def main():
    args = sys.argv[1:]
    mods = module_ids()
    all_ids = sorted(p.parent.name for p in server.PROBLEMS.glob("*/meta.json"))
    if args[:1] == ["--module"]:
        ids = [i for i in all_ids if (server.read_json(server.PROBLEMS / i / "meta.json") or {}).get("module") == args[1]]
    else:
        ids = args or all_ids
    bad = 0
    with ThreadPoolExecutor(max_workers=4) as ex:
        for pid, errs in zip(ids, ex.map(lambda i: check(i, mods), ids)):
            if errs:
                bad += 1
                print(f"\033[31mFAIL\033[0m {pid}")
                for e in errs:
                    print("   - " + e.replace("\n", "\n     "))
            else:
                print(f"\033[32m OK \033[0m {pid}")
    print(f"\n{len(ids) - bad}/{len(ids)} problems OK")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
