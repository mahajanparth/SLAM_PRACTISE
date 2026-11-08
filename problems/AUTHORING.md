# Problem authoring guide

Problems power the local judge (`python3 app/server.py`, http://127.0.0.1:8008).
Reference examples you MUST copy the style of: `problems/cpp-m01-quiz/` and `problems/cpp-m01-closest/`.

## Layout
```
problems/<id>/          id = <track>-<module lowercase>-<slug>, e.g. cpp-m02-erase-remove, slam-s4-voxel-grid
  meta.json             {"title","track":"cpp|slam","module":"M02|S4|DSA...","order":N,"type":"code|quiz",
                         "difficulty":"Easy|Medium|Hard","target_min":N,"tags":[...],"hints":[...escalating...]}
  statement.md          problem statement (markdown, KaTeX math with $...$ / $$...$$ allowed)
  # type=code:
  starter.cpp           includes + struct/type definitions + function stubs with `// TODO`. NO main().
  solution.cpp          full reference solution (same includes/types as starter). NO main().
  tests.cpp             main() with CHECKs; public tests first, then `#ifdef HIDDEN ... #endif` hidden tests
  # type=quiz:
  quiz.json             {"questions":[{"prompt": md, "options":[...], "answer": idx, "explanation": md}, ...]}
notes/<MODULE>.md       one revision cheat-sheet per module (see below)
```
`order` sets the position within a module (1, 2, 3…). Module ids and order live in `problems/modules.json`.
Do not edit that file.

## How judging works
`g++ -std=c++17 -Wall -Wextra -O0 -g -fsanitize=address,undefined -I<repo> -I/usr/include/eigen3 -pthread -include <user_code.cpp> [-DHIDDEN] tests.cpp`
- The user's code (from starter.cpp) is **prepended** to tests.cpp. So the starter/solution must contain every
  `#include` and type definition the user needs. tests.cpp includes only `"common/check.h"`
  (plus `"common/cloud_utils.h"` or std headers it needs itself).
- Test macros (common/check.h): `CHECK(cond)`, `CHECK_EQ(a,b)` (both must be printable with <<),
  `CHECK_NEAR(a,b,tol)`, `CHECK_MAT_NEAR(A,B,tol)` (Eigen), and `return TEST_SUMMARY();` at the end of main.
- Data helpers (common/cloud_utils.h, namespace `cu`): `Cloud` (= vector<Eigen::Vector3d>), `uniformCube(n, half)`,
  `planePoints(n, normal, d, extent, noise)`, `spherePoints(n, c, r)`, `addNoise(cloud, sigma)`,
  `randomPose(max_angle, max_trans)` -> Isometry3d, `transform(cloud, T)`, `toMatrix(cloud)` -> Matrix3Xd, `Timer`.
  `cu::rng()` is a seeded mt19937 (deterministic).
- Status: compile error / sanitizer error / wrong answer (any CHECK failed) / timeout (15 s) / accepted.
- Tests run in a sanitizer debug build (~10–50x slower than -O2). Keep sizes modest (≤ ~2e5 points, ≤ 2 s total).

## Quality bar
- **Starter must compile and must NOT pass the public tests.** Stubs return dummy values (0, {}, nullptr, false,
  Identity) — never leave non-void functions without a return.
- Starter stubs should not trigger UB or crashes in tests if possible (e.g. test sizes before indexing:
  `CHECK(v.size()==3); if (v.size()==3) { ... }`), so the user sees clean failures.
- Public tests: 4–8 CHECKs covering the main behaviour. Hidden tests: 4–10 CHECKs on edge cases
  (empty input, single element, degenerate geometry, negative coords, angle wrap, near-π rotations, large n,
  aliasing, etc.) — the ones an interviewer would probe.
- Tests must be deterministic and tolerant (use CHECK_NEAR with sensible tolerances; for randomized
  algorithms like RANSAC/ICP use a seeded rng and robust thresholds; compare against brute force when possible).
- Do not test implementation details the statement doesn't pin down; if order of output matters, say so in
  the statement (or sort before comparing).
- Statement: concise, interview-style. Include: context sentence (robotics flavour), exact signatures in a
  table or code block, input assumptions/constraints, return semantics for edge cases, complexity target, and a
  short "Think about" list (follow-up questions an interviewer would ask). For SLAM problems include the key
  math only when it is a "given" (don't give away the derivation the candidate is supposed to produce).
- Hints: 2–4, escalating from nudge to near-solution.
- Solution: clean, idiomatic, commented where non-obvious; it is shown to the user as the "editorial".
- Only std + Eigen. No PCL/Sophus/Ceres/OpenCV.
- Quizzes: 5–8 questions, predict-the-output / spot-the-bug / concept MCQs with code in fenced ```cpp blocks.
  4 options each, one correct; explanations should teach (what happens in memory, cost, the rule).
  Escape JSON properly (\n, \"). Vary the correct option index.

## notes/<MODULE>.md (revision cheat-sheet, ~80–150 lines)
Sections: **Core ideas** (short snippets) · **What happens in memory / cost table** · **Pitfalls interviewers ask** ·
**Robotics angle** (how it shows up in SLAM/perception code) · **Rapid-fire questions** (5–8 Q with one-line answers
in a collapsible `<details>` block). Use tables and fenced code. Example: `notes/M01.md`.

## Verify (mandatory)
`python3 app/verify.py <id> ...` or `python3 app/verify.py --module M02` — every problem must print OK.
It compiles starter (must compile, must fail), and solution with Run and Submit (must be accepted & sanitizer-clean).
Never write into `workspace/` (that's the user's code).
