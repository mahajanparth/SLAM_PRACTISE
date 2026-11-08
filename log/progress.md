# Progress Log

Scores are 1–4 per axis (1 = no hire signal, 2 = weak, 3 = solid, 4 = strong).
Axes: **Correct** (works + edge cases) · **C++** (idiomatic, const, ownership, no UB) ·
**Math** (understands why) · **Comms** (clarifies, thinks aloud) · **Speed** (vs. target time)

## Weak spots to revisit
_(Claude keeps this list current; items are re-asked in later sessions until scored ≥3.)_
- **Object lifetime / dangling** — missed vector-reallocation invalidating a reference (M1 Q1) and pointer into a destroyed temporary (M1 follow-up). Re-test in M10, M11, M15.
- **`auto` type deduction** — answered with values, not types; recall `auto` drops const+ref, `auto&` keeps const.
- Style: prefer `x*x` / `std::sqrt` over `std::pow`; compare squared distances; include what you use (`<limits>`).

## C++ Revision
| Module | Date | Quiz (1–4) | Exercise | Notes |
|--------|------|------------|----------|-------|
| M1 Fundamentals | 2026-11-08 | 2 | Pass 10/10 | Q2 ✅, Q3 ½ (types), Q1 ❌ + FU2 ❌ (lifetime). Code clean; pow→sqrt, squared dist. |

## Sessions
<!-- Template:
### Session N — <theme> — YYYY-MM-DD
| Q | Topic | Target | Actual | Hints | Correct | C++ | Math | Comms | Speed |
|---|-------|--------|--------|-------|---------|-----|------|-------|-------|
Notes:
-->
