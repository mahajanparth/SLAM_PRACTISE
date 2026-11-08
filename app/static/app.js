/* SLAM Practice — client. Talks to app/server.py (stdlib) on localhost. */
(() => {
  "use strict";

  const MONACO = "https://cdnjs.cloudflare.com/ajax/libs/monaco-editor/0.45.0/min";
  const $ = (s, el = document) => el.querySelector(s);
  const $$ = (s, el = document) => [...el.querySelectorAll(s)];
  const LS = {
    get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch { return d; } },
    set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* ignore */ } },
  };
  const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

  const SCRATCH_TEMPLATE = `// Scratchpad: free-form C++17 with its own main(). Ctrl+Enter to run.
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  Eigen::Matrix3d R = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::cout << "R =\\n" << R << "\\n";
  CHECK_NEAR(R.determinant(), 1.0, 1e-12);
  return TEST_SUMMARY();
}
`;

  const STATUS_LABEL = {
    accepted: "Accepted", wrong_answer: "Wrong Answer", compile_error: "Compilation Error",
    compile_timeout: "Compilation Timeout", sanitizer_error: "Memory / UB Error (sanitizer)",
    runtime_error: "Runtime Error", timeout: "Time Limit Exceeded", ok: "Finished", running: "Running…",
  };

  const S = {
    catalog: null,
    track: LS.get("track", "cpp"),
    current: null,          // {id, data}
    editor: null,
    monaco: null,
    probTab: "desc",
    consoleTab: "results",
    open: new Set(LS.get("openModules", ["M01"])),
    timerPaused: false,
    saveTimer: null,
    termHistory: LS.get("termHistory", []),
    termIdx: -1,
    mock: LS.get("mock", null),
    busy: false,
  };

  // ------------------------------------------------------------------ API
  async function api(path, body) {
    const opt = body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
    const r = await fetch("/api/" + path, opt);
    const j = await r.json().catch(() => ({ error: "bad response" }));
    if (!r.ok && !j.error) j.error = r.statusText;
    return j;
  }

  // ------------------------------------------------------------------ markdown + math
  if (window.marked) {
    marked.setOptions({
      mangle: false, headerIds: false,
      highlight(code, lang) {
        if (!window.hljs) return code;
        try { return lang && hljs.getLanguage(lang) ? hljs.highlight(code, { language: lang }).value : hljs.highlight(code, { language: "cpp" }).value; }
        catch { return code; }
      },
      langPrefix: "hljs language-",
    });
  }
  function renderMD(src) {
    src = String(src || "");
    if (!window.marked) return `<pre>${esc(src)}</pre>`;
    const math = [];
    // keep code (fenced and inline) intact; extract math elsewhere
    const parts = src.split(/(```[\s\S]*?```|`[^`\n]*`)/g);
    for (let i = 0; i < parts.length; i += 2) {
      parts[i] = parts[i]
        .replace(/\$\$([\s\S]+?)\$\$/g, (_, m) => { math.push([m, true]); return `MATHPH${math.length - 1}X`; })
        .replace(/\$([^\$\n]+?)\$/g, (_, m) => { math.push([m, false]); return `MATHPH${math.length - 1}X`; });
    }
    let html = marked.parse(parts.join(""));
    html = html.replace(/MATHPH(\d+)X/g, (_, i) => {
      const [m, display] = math[+i];
      if (!window.katex) return esc(m);
      try { return katex.renderToString(m, { displayMode: display, throwOnError: false }); } catch { return esc(m); }
    });
    return html;
  }

  // ------------------------------------------------------------------ ANSI
  function ansiToHtml(text) {
    const map = { 31: "a-red", 32: "a-green", 33: "a-yellow", 34: "a-blue", 35: "a-magenta", 36: "a-cyan", 91: "a-red", 92: "a-green", 93: "a-yellow", 94: "a-blue", 95: "a-magenta", 96: "a-cyan", 1: "a-bold" };
    let out = "", open = 0;
    const re = /\x1b\[([0-9;]*)m/g;
    let last = 0, m;
    while ((m = re.exec(text))) {
      out += esc(text.slice(last, m.index));
      last = re.lastIndex;
      for (const code of (m[1] || "0").split(";").map(Number)) {
        if (code === 0) { out += "</span>".repeat(open); open = 0; }
        else if (map[code]) { out += `<span class="${map[code]}">`; open++; }
      }
    }
    out += esc(text.slice(last)) + "</span>".repeat(open);
    return out;
  }
  const stripAnsi = (t) => String(t || "").replace(/\x1b\[[0-9;]*m/g, "");

  // ------------------------------------------------------------------ catalog / sidebar
  function problemsOf(track) {
    return S.catalog.problems.filter((p) => p.track === track);
  }
  function modulesOf(track) {
    const t = S.catalog.modules.tracks.find((t) => t.id === track);
    return t ? t.modules : [];
  }
  function progressOf(id) { return S.catalog.progress[id] || null; }
  function statusOf(id) { const p = progressOf(id); return p ? p.status : "todo"; }

  function sortedProblems(module) {
    return S.catalog.problems.filter((p) => p.module === module).sort((a, b) => (a.order - b.order) || a.title.localeCompare(b.title));
  }

  function renderTracks() {
    const el = $("#trackTabs");
    el.innerHTML = S.catalog.modules.tracks.map((t) =>
      `<button class="track-tab ${t.id === S.track ? "active" : ""}" data-track="${t.id}">${esc(t.title)}</button>`).join("");
    $$(".track-tab", el).forEach((b) => b.onclick = () => { setTrack(b.dataset.track); location.hash = "#dash/" + b.dataset.track; });
  }

  function setTrack(t) {
    S.track = t; LS.set("track", t);
    renderTracks(); renderSidebar(); renderOverall();
  }

  function ring(done, total) {
    const r = 6.5, c = 2 * Math.PI * r, f = total ? done / total : 0;
    return `<svg class="module-ring" viewBox="0 0 16 16"><circle cx="8" cy="8" r="${r}" fill="none" stroke="var(--border-2)" stroke-width="2"/>
      <circle cx="8" cy="8" r="${r}" fill="none" stroke="var(--accent)" stroke-width="2" stroke-dasharray="${c * f} ${c}" transform="rotate(-90 8 8)" stroke-linecap="round"/></svg>`;
  }

  function probRow(p) {
    const st = statusOf(p.id);
    const active = S.current && S.current.id === p.id ? "active" : "";
    return `<div class="prob-row ${active}" data-id="${p.id}" title="${esc(p.title)}">
      <span class="dot ${st}"></span><span class="prob-title">${esc(p.title)}</span>
      ${p.type === "quiz" ? '<span class="kind">quiz</span>' : `<span class="diff ${p.difficulty}">${p.difficulty[0]}</span>`}</div>`;
  }

  function renderSidebar() {
    const q = $("#search").value.trim().toLowerCase();
    const mods = modulesOf(S.track);
    let html = "", lastSitting = null;
    for (const m of mods) {
      let probs = sortedProblems(m.id);
      if (q) probs = probs.filter((p) => (p.title + " " + (p.tags || []).join(" ") + " " + p.id + " " + m.title).toLowerCase().includes(q));
      if (q && !probs.length) continue;
      if (m.sitting && m.sitting !== lastSitting) { html += `<div class="sitting-label">Sitting ${m.sitting}</div>`; lastSitting = m.sitting; }
      const all = sortedProblems(m.id);
      const done = all.filter((p) => statusOf(p.id) === "solved").length;
      const open = q || S.open.has(m.id) ? "open" : "";
      html += `<div class="module ${open}" data-mod="${m.id}">
        <div class="module-head"><span class="chev">▶</span><span class="module-id">${m.id}</span>
          <span class="module-title" title="${esc(m.blurb || "")}">${esc(m.title)}</span>
          <span class="module-count ${done === all.length && all.length ? "done" : ""}">${done}/${all.length}</span>${ring(done, all.length)}</div>
        <div class="module-body">
          <div class="notes-link" data-notes="${m.id}">📘 Revision notes</div>
          ${probs.map(probRow).join("") || '<div class="prob-row muted">No problems yet</div>'}
        </div></div>`;
    }
    $("#moduleList").innerHTML = html || '<div class="pad muted">No matches.</div>';
    $$(".module-head").forEach((h) => h.onclick = () => {
      const id = h.parentElement.dataset.mod;
      S.open.has(id) ? S.open.delete(id) : S.open.add(id);
      LS.set("openModules", [...S.open]);
      h.parentElement.classList.toggle("open");
    });
    $$(".notes-link").forEach((n) => n.onclick = () => location.hash = "#notes/" + n.dataset.notes);
    $$("#moduleList .prob-row[data-id]").forEach((r) => r.onclick = () => location.hash = "#p/" + r.dataset.id);
    renderMockPanel();
  }

  function renderOverall() {
    const probs = problemsOf(S.track);
    const done = probs.filter((p) => statusOf(p.id) === "solved").length;
    $("#overallFill").style.width = (probs.length ? (100 * done) / probs.length : 0) + "%";
    $("#overallText").textContent = `${done}/${probs.length} solved`;
  }

  async function refreshCatalog() {
    S.catalog = await api("catalog");
    renderTracks(); renderSidebar(); renderOverall();
  }

  // ------------------------------------------------------------------ layout modes
  function setDashMode(on) {
    $("#layout").classList.toggle("dash-mode", on);
    $("#workPane").classList.toggle("hidden", on);
    $$(".gutter-v")[1].classList.toggle("hidden", on);
    $("#problemPane").style.flex = on ? "1" : "";
    $("#probTabs").classList.toggle("hidden", on);
  }

  // ------------------------------------------------------------------ dashboard
  function showDashboard(track) {
    if (track && track !== S.track) setTrack(track);
    S.current = null; renderSidebar(); updateTimerDisplay();
    setDashMode(true);
    const mods = modulesOf(S.track);
    const probs = problemsOf(S.track);
    const solved = probs.filter((p) => statusOf(p.id) === "solved").length;
    const attempted = probs.filter((p) => statusOf(p.id) === "attempted").length;
    const quizzes = probs.filter((p) => p.type === "quiz").length;
    const next = (() => {
      for (const m of mods) for (const p of sortedProblems(m.id)) if (statusOf(p.id) !== "solved") return p;
      return null;
    })();
    const tr = S.catalog.modules.tracks.find((t) => t.id === S.track);
    $("#probBody").innerHTML = `<div class="dash">
      <h1>${esc(tr ? tr.title : "")}</h1>
      <div class="muted">${S.track === "cpp" ? "C++17 from fundamentals to interview level. Work through the sittings in order: read the notes, take the quiz, then solve the coding problems." : "Live-coding SLAM interview practice. Do these after the C++ track, then run Mock Interviews."}</div>
      <div class="stat-row">
        <div class="stat"><div class="v">${solved}<span class="muted" style="font-size:14px">/${probs.length}</span></div><div class="l">solved</div></div>
        <div class="stat"><div class="v">${attempted}</div><div class="l">in progress</div></div>
        <div class="stat"><div class="v">${probs.length - quizzes}</div><div class="l">coding problems</div></div>
        <div class="stat"><div class="v">${quizzes}</div><div class="l">quizzes</div></div>
      </div>
      ${next ? `<p style="margin-top:16px"><button class="btn primary" id="continueBtn">Continue → ${esc(next.title)}</button></p>` : ""}
      <h2>Modules</h2>
      <div class="dash-grid">${mods.map((m) => {
        const ps = sortedProblems(m.id), d = ps.filter((p) => statusOf(p.id) === "solved").length;
        return `<div class="dash-card" data-mod="${m.id}"><div class="id">${m.id}${m.sitting ? " · " + m.sitting : ""}</div>
          <div class="t">${esc(m.title)}</div><div class="b">${esc(m.blurb || "")}</div>
          <div class="muted" style="font-size:12px">${d}/${ps.length} solved</div>
          <div class="mini-bar"><div style="width:${ps.length ? (100 * d) / ps.length : 0}%"></div></div></div>`;
      }).join("")}</div>
      <h2>How it works</h2>
      <ul>
        <li><b>Run</b> (<kbd>Ctrl</kbd>+<kbd>Enter</kbd>) compiles with g++ plus AddressSanitizer/UBSan and runs the <i>public</i> tests.</li>
        <li><b>Submit</b> (<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>Enter</kbd>) adds the <i>hidden</i> edge-case tests. Green means solved.</li>
        <li>Your code autosaves to <code>workspace/&lt;id&gt;.cpp</code>. The <b>Terminal</b> tab runs a real shell in the repo.</li>
        <li><b>Mock Interview</b> picks 3 problems against a countdown; hints and editorials are locked until you end it.</li>
        <li>Ask Claude in your editor to <i>“review my solution to &lt;id&gt;”</i> for interviewer-style feedback.</li>
      </ul></div>`;
    $$(".dash-card").forEach((c) => c.onclick = () => {
      const first = sortedProblems(c.dataset.mod)[0];
      S.open.add(c.dataset.mod); LS.set("openModules", [...S.open]);
      location.hash = first ? "#p/" + first.id : "#notes/" + c.dataset.mod;
    });
    const cb = $("#continueBtn");
    if (cb) cb.onclick = () => { S.open.add(next.module); LS.set("openModules", [...S.open]); location.hash = "#p/" + next.id; };
    $("#probBody").scrollTop = 0;
  }

  async function showNotes(mod) {
    const m = S.catalog.modules.tracks.flatMap((t) => t.modules.map((x) => ({ ...x, track: t.id }))).find((x) => x.id === mod);
    if (m && m.track !== S.track) setTrack(m.track);
    S.current = null; renderSidebar(); updateTimerDisplay();
    setDashMode(true);
    const r = await api("notes/" + mod);
    const probs = sortedProblems(mod);
    $("#probBody").innerHTML = (r.markdown ? renderMD(r.markdown) : `<h1>${esc(m ? m.title : mod)}</h1><p class="muted">Notes coming soon.</p>`)
      + (probs.length ? `<h2>Practice</h2>${probs.map((p) => `<div class="choice" data-id="${p.id}"><div class="t">${esc(p.title)}</div>
          <div class="d">${p.type === "quiz" ? "Quiz" : p.difficulty} · ~${p.target_min} min · ${statusOf(p.id)}</div></div>`).join("")}` : "");
    $$("#probBody .choice").forEach((c) => c.onclick = () => location.hash = "#p/" + c.dataset.id);
    $("#probBody").scrollTop = 0;
  }

  // ------------------------------------------------------------------ problem view
  async function openProblem(id) {
    flushSave();
    if (id === "scratchpad") return openScratch();
    const data = await api("problem/" + id);
    if (data.error) { toast("Problem not found: " + id); return showDashboard(); }
    if (data.meta.track !== S.track) setTrack(data.meta.track);
    S.open.add(data.meta.module); LS.set("openModules", [...S.open]);
    S.current = { id, data };
    setDashMode(false);
    renderSidebar();
    $(`.prob-row[data-id="${id}"]`)?.scrollIntoView({ block: "nearest" });
    S.probTab = "desc"; renderProbTabs();
    const isQuiz = data.meta.type === "quiz";
    $("#editorWrap").classList.toggle("hidden", isQuiz);
    $("#editorToolbar").classList.toggle("hidden", isQuiz);
    $("#quizWrap").classList.toggle("hidden", !isQuiz);
    $("#console").classList.toggle("hidden", isQuiz);
    $("#consoleGutter").classList.toggle("hidden", isQuiz);
    $("#fileName").textContent = `workspace/${id}.cpp`;
    $("#saveState").textContent = "";
    if (isQuiz) renderQuiz(data);
    else { await editorReady; setCode(data.code); clearMarkers(); }
    resetConsole();
    updateTimerDisplay();
    document.title = data.meta.title + " · SLAM Practice";
  }

  function openScratch() {
    S.current = { id: "scratchpad", data: { meta: { title: "Scratchpad", type: "code", target_min: 0 }, statement: "" } };
    setDashMode(false); renderSidebar();
    $("#editorWrap").classList.remove("hidden"); $("#editorToolbar").classList.remove("hidden");
    $("#quizWrap").classList.add("hidden"); $("#console").classList.remove("hidden"); $("#consoleGutter").classList.remove("hidden");
    $("#fileName").textContent = "workspace/scratchpad.cpp";
    $("#probTabs").classList.add("hidden");
    $("#probBody").innerHTML = renderMD(`# Scratchpad
A free-form C++17 playground with your own \`main()\`. It uses the same compiler flags and sanitizers as the judge.

- \`common/check.h\` gives you \`CHECK\`, \`CHECK_NEAR\`, \`CHECK_MAT_NEAR\` and \`TEST_SUMMARY()\`.
- \`common/cloud_utils.h\` gives you \`cu::uniformCube\`, \`cu::randomPose\`, \`cu::transform\`, \`cu::Timer\`, …
- Eigen 3.4 is on the include path.

Use it to try snippets from the notes, or for "predict the output" experiments. **Run** compiles and executes it.`);
    editorReady.then(() => { setCode(LS.get("scratch", SCRATCH_TEMPLATE)); clearMarkers(); });
    $("#submitBtn").classList.add("hidden");
    resetConsole(); updateTimerDisplay();
  }

  function renderProbTabs() {
    $("#probTabs").classList.remove("hidden");
    $("#submitBtn").classList.remove("hidden");
    $$("#probTabs .tab").forEach((t) => t.classList.toggle("active", t.dataset.tab === S.probTab));
    const d = S.current.data, meta = d.meta;
    const hints = meta.hints || [];
    $('#probTabs [data-tab="hints"]').innerHTML = `Hints<span class="badge">${hints.length}</span>`;
    const body = $("#probBody");
    if (S.probTab === "desc") {
      const prog = progressOf(S.current.id);
      const pills = [
        meta.type === "quiz" ? '<span class="pill">Quiz</span>' : `<span class="pill ${meta.difficulty}">${meta.difficulty}</span>`,
        `<span class="pill">⏱ ~${meta.target_min} min</span>`,
        `<span class="pill">${meta.module}</span>`,
        ...(meta.tags || []).map((t) => `<span class="pill">${esc(t)}</span>`),
        prog && prog.status === "solved" ? '<span class="pill solved">✓ Solved</span>' : "",
        prog && prog.best_time_s ? `<span class="pill">best ${fmtTime(prog.best_time_s)}</span>` : "",
      ].join("");
      let md = d.statement || "";
      let titleHtml = "";
      const m = md.match(/^#\s+(.+)\n/);
      if (m) { titleHtml = `<h1>${renderMD(m[1]).replace(/<\/?p>/g, "")}</h1>`; md = md.slice(m[0].length); }
      else titleHtml = `<h1>${esc(meta.title)}</h1>`;
      body.innerHTML = titleHtml + `<div class="meta-row">${pills}</div>` + renderMD(md);
    } else if (S.probTab === "notes") {
      body.innerHTML = '<div class="muted">Loading…</div>';
      api("notes/" + meta.module).then((r) => {
        if (S.probTab === "notes") body.innerHTML = r.markdown ? renderMD(r.markdown) : '<p class="muted">No notes for this module yet.</p>';
      });
    } else if (S.probTab === "hints") {
      if (mockLocked()) { body.innerHTML = lockCard("🔒", "Hints are locked during a mock interview.", ""); return; }
      const key = "hints:" + S.current.id;
      const shown = Math.min(LS.get(key, 0), hints.length);
      body.innerHTML = `<h2 style="margin-top:0">Hints</h2>` + (hints.length ? "" : '<p class="muted">No hints for this one.</p>') +
        hints.slice(0, shown).map((h, i) => `<div class="hint-card"><div class="hint-n">Hint ${i + 1}</div>${renderMD(h)}</div>`).join("") +
        (shown < hints.length ? `<button class="btn" id="moreHint">Reveal hint ${shown + 1} of ${hints.length}</button>
          <p class="muted" style="font-size:12px">Hints are escalating. In a real interview, asking for one is fine, but say what you've tried first.</p>` : "");
      const b = $("#moreHint"); if (b) b.onclick = () => { LS.set(key, shown + 1); renderProbTabs(); };
    } else if (S.probTab === "solution") {
      if (mockLocked()) { body.innerHTML = lockCard("🔒", "The editorial is locked during a mock interview.", ""); return; }
      if (meta.type === "quiz") { body.innerHTML = '<p class="muted">Quiz explanations appear after you submit your answers.</p>'; return; }
      if (!d.has_solution) { body.innerHTML = '<p class="muted">No editorial for this problem.</p>'; return; }
      const solved = statusOf(S.current.id) === "solved";
      body.innerHTML = lockCard(solved ? "📖" : "🔒", solved ? "You solved it. Compare your code with the reference." : "Try it first. Interviews reward the attempt, not the recall.",
        `<button class="btn ${solved ? "primary" : ""}" id="revealSol">${solved ? "Show editorial" : "Reveal anyway"}</button>`);
      $("#revealSol").onclick = async () => {
        const r = await api("solution/" + S.current.id);
        body.innerHTML = `<h2 style="margin-top:0">Reference solution</h2>` + renderMD("```cpp\n" + (r.code || r.error) + "\n```") +
          `<p class="muted">Ask Claude: <i>“review my solution to ${S.current.id}”</i> for interviewer-style feedback on yours.</p>`;
      };
    }
  }
  const lockCard = (icon, text, extra) => `<div class="lock-card"><div class="big">${icon}</div><div>${text}</div>${extra}</div>`;

  // ------------------------------------------------------------------ quiz
  function renderQuiz(data) {
    const qs = (data.quiz && data.quiz.questions) || [];
    const key = "quiz:" + data.meta.id;
    const saved = LS.get(key, { answers: {}, graded: null });
    const wrap = $("#quizWrap");
    wrap.innerHTML = qs.map((q, i) => `<div class="q-card" data-q="${i}">
        <div class="q-num">QUESTION ${i + 1} / ${qs.length}</div><div class="md">${renderMD(q.prompt)}</div>
        ${q.options.map((o, j) => `<label class="q-opt" data-o="${j}"><input type="radio" name="q${i}" value="${j}" ${saved.answers[i] == j ? "checked" : ""}/>
          <div class="md">${renderMD(o)}</div></label>`).join("")}
        <div class="q-expl hidden md"></div></div>`).join("") +
      `<div class="quiz-foot"><button class="btn primary" id="quizSubmit">Submit answers</button>
        <button class="btn ghost" id="quizRetry">Retry</button><span class="quiz-score" id="quizScore"></span></div>`;
    $$(".q-opt", wrap).forEach((l) => l.addEventListener("change", () => {
      const qi = l.closest(".q-card").dataset.q;
      saved.answers[qi] = +l.dataset.o; saved.graded = null; LS.set(key, saved);
      $$(`.q-card[data-q="${qi}"] .q-opt`, wrap).forEach((x) => x.classList.toggle("picked", x === l));
    }));
    $$(".q-opt input:checked", wrap).forEach((i) => i.parentElement.classList.add("picked"));
    $("#quizSubmit").onclick = async () => {
      const r = await api("quiz", { id: data.meta.id, answers: saved.answers });
      saved.graded = r; LS.set(key, saved);
      showGrade(r);
      if (r.progress) { S.catalog.progress[data.meta.id] = r.progress; renderSidebar(); renderOverall(); }
    };
    $("#quizRetry").onclick = () => { LS.set(key, { answers: {}, graded: null }); renderQuiz(data); };
    if (saved.graded) showGrade(saved.graded);
    function showGrade(r) {
      r.results.forEach((res, i) => {
        const card = $(`.q-card[data-q="${i}"]`, wrap); if (!card) return;
        card.classList.toggle("correct", res.correct); card.classList.toggle("wrong", !res.correct);
        $$(".q-opt", card).forEach((o) => {
          o.classList.toggle("is-answer", +o.dataset.o === res.answer);
          o.classList.toggle("is-wrong", o.classList.contains("picked") && +o.dataset.o !== res.answer);
        });
        const ex = $(".q-expl", card); ex.innerHTML = renderMD((res.correct ? "✅ " : "❌ ") + res.explanation); ex.classList.remove("hidden");
      });
      $("#quizScore").innerHTML = `Score: <span class="${r.score === r.total ? "st-accepted" : "st-wrong_answer"}">${r.score} / ${r.total}</span>`;
    }
    wrap.scrollTop = 0;
  }

  // ------------------------------------------------------------------ editor
  let resolveEditor;
  const editorReady = new Promise((r) => (resolveEditor = r));

  function initEditor() {
    const fallback = () => {
      if (S.editor) return;
      const ta = document.createElement("textarea");
      ta.className = "fallback"; ta.spellcheck = false;
      $("#editor").appendChild(ta);
      ta.addEventListener("keydown", (e) => {
        if (e.key === "Tab") { e.preventDefault(); document.execCommand("insertText", false, "  "); }
      });
      ta.addEventListener("input", onCodeChange);
      S.editor = { getValue: () => ta.value, setValue: (v) => (ta.value = v), fallback: true };
      resolveEditor();
    };
    if (!window.require) return fallback();
    const t = setTimeout(fallback, 10000);
    window.MonacoEnvironment = {
      getWorkerUrl: () => "data:text/javascript;charset=utf-8," + encodeURIComponent(
        `self.MonacoEnvironment={baseUrl:'${MONACO}/'};importScripts('${MONACO}/vs/base/worker/workerMain.min.js');`),
    };
    require.config({ paths: { vs: MONACO + "/vs" } });
    require(["vs/editor/editor.main"], () => {
      clearTimeout(t);
      if (S.editor) return;
      S.monaco = monaco;
      monaco.editor.defineTheme("slam-dark", { base: "vs-dark", inherit: true, rules: [], colors: { "editor.background": "#0d1117", "editorGutter.background": "#0d1117" } });
      monaco.editor.defineTheme("slam-light", { base: "vs", inherit: true, rules: [], colors: { "editor.background": "#ffffff" } });
      S.editor = monaco.editor.create($("#editor"), {
        language: "cpp", theme: themeName(), automaticLayout: true, fontFamily: "JetBrains Mono, monospace",
        fontSize: LS.get("fontSize", 13.5), fontLigatures: false, minimap: { enabled: false }, tabSize: 2,
        scrollBeyondLastLine: false, renderWhitespace: "selection", smoothScrolling: true, padding: { top: 10 },
        bracketPairColorization: { enabled: true }, guides: { bracketPairs: true },
      });
      S.editor.onDidChangeModelContent(onCodeChange);
      S.editor.addCommand(monaco.KeyMod.CtrlCmd | monaco.KeyCode.Enter, () => run("run"));
      S.editor.addCommand(monaco.KeyMod.CtrlCmd | monaco.KeyMod.Shift | monaco.KeyCode.Enter, () => run("submit"));
      S.editor.addCommand(monaco.KeyMod.CtrlCmd | monaco.KeyCode.KeyS, () => flushSave());
      resolveEditor();
    }, fallback);
  }
  const themeName = () => (document.documentElement.dataset.theme === "light" ? "slam-light" : "slam-dark");

  let settingCode = false;
  function setCode(code) { settingCode = true; S.editor.setValue(code || ""); settingCode = false; }
  function onCodeChange() {
    if (settingCode || !S.current) return;
    if (S.current.id === "scratchpad") { LS.set("scratch", S.editor.getValue()); return; }
    $("#saveState").textContent = "editing…";
    clearTimeout(S.saveTimer);
    S.saveTimer = setTimeout(flushSave, 800);
  }
  async function flushSave() {
    clearTimeout(S.saveTimer); S.saveTimer = null;
    if (!S.current || !S.editor || S.current.id === "scratchpad" || S.current.data.meta.type === "quiz") return;
    const r = await api("save", { id: S.current.id, code: S.editor.getValue() });
    $("#saveState").textContent = r.ok ? "saved ✓" : "save failed";
  }

  function clearMarkers() {
    if (S.monaco && S.editor.getModel) S.monaco.editor.setModelMarkers(S.editor.getModel(), "gcc", []);
  }
  function setMarkers(compileOut) {
    if (!S.monaco || !S.current) return;
    const file = `workspace/${S.current.id}.cpp`;
    const markers = [];
    for (const line of stripAnsi(compileOut).split("\n")) {
      const m = line.match(/^(.+?):(\d+):(\d+):\s+(error|warning|note):\s+(.*)$/);
      if (m && m[1].endsWith(file)) {
        const sev = m[4] === "error" ? S.monaco.MarkerSeverity.Error : m[4] === "warning" ? S.monaco.MarkerSeverity.Warning : S.monaco.MarkerSeverity.Info;
        markers.push({ startLineNumber: +m[2], startColumn: +m[3], endLineNumber: +m[2], endColumn: +m[3] + 200, message: m[5], severity: sev });
      }
    }
    S.monaco.editor.setModelMarkers(S.editor.getModel(), "gcc", markers);
  }

  // ------------------------------------------------------------------ run / submit
  async function run(mode) {
    if (!S.current || S.busy) return;
    if (S.current.data.meta.type === "quiz") return;
    const scratch = S.current.id === "scratchpad";
    if (scratch && mode === "submit") mode = "run";
    S.busy = true; setBusy(true);
    await flushSave();
    showConsole("results");
    setChip("running");
    $("#cResults").innerHTML = `<div class="pad"><span class="spinner"></span>&nbsp; Compiling with sanitizers${mode === "submit" ? " + hidden tests" : ""}…</div>`;
    const id = S.current.id;
    const res = await api("run", { id, code: S.editor.getValue(), mode, elapsed: Math.round(LS.get("elapsed:" + id, 0)) });
    S.busy = false; setBusy(false);
    if (!S.current || S.current.id !== id) return;
    if (res.error) { $("#cResults").innerHTML = `<div class="pad st-wrong_answer">${esc(res.error)}</div>`; setChip(""); return; }
    renderResult(res, mode);
    if (res.progress) { S.catalog.progress[id] = res.progress; renderSidebar(); renderOverall(); }
    if (mode === "submit" && res.status === "accepted") { toast("✅ Accepted. All hidden tests passed!"); S.timerPaused = true; updateTimerDisplay(); }
  }

  function setBusy(b) { ["#runBtn", "#submitBtn"].forEach((s) => ($(s).disabled = b)); }
  function setChip(status) {
    const c = $("#statusChip");
    c.className = "status-chip st-" + status;
    c.textContent = status ? STATUS_LABEL[status] || status : "";
  }

  function parseFails(out) {
    const lines = stripAnsi(out).split("\n");
    const fails = [];
    for (const l of lines) {
      const m = l.match(/^FAIL\s+(\S+):(\d+)\s+(.*)$/);
      if (m) fails.push({ loc: `${m[1].replace(/^problems\/[^/]+\//, "")}:${m[2]}`, expr: m[3], detail: [] });
      else if (fails.length && /^\s{3,}\S/.test(l) && fails[fails.length - 1].detail.length < 12) fails[fails.length - 1].detail.push(l.trim());
    }
    return fails;
  }

  function renderResult(res, mode) {
    setChip(res.status);
    const out = (res.compile_output ? res.compile_output + (res.run_output ? "\n" : "") : "") + (res.run_output || "");
    $("#cOutput").innerHTML = out.trim() ? ansiToHtml(out).replace(/(workspace\/[\w\-]+\.cpp):(\d+)/g, '<span class="ce-line" data-line="$2">$1:$2</span>') : '<span class="muted">(no output)</span>';
    $$("#cOutput .ce-line").forEach((s) => s.onclick = () => gotoLine(+s.dataset.line));
    setMarkers(res.compile_output || "");

    const scratch = S.current.id === "scratchpad";
    const total = res.passed + res.failed;
    let label = STATUS_LABEL[res.status] || res.status;
    if (res.status === "accepted" && mode === "run") label = "Public tests passed. Now Submit";
    let html = `<div class="result-head"><span class="result-status st-${res.status}">${esc(label)}</span>
      <span class="result-meta">${mode === "submit" ? "public + hidden tests" : scratch ? "scratchpad" : "public tests"} · compile ${res.compile_time}s${res.run_time !== undefined ? " · run " + res.run_time + "s" : ""}</span></div>`;
    if (total) {
      html += `<div class="bar-wrap"><div class="result-meta" style="margin-bottom:5px">${res.passed} / ${total} checks passed</div>
        <div class="check-bar"><div class="ok" style="width:${(100 * res.passed) / total}%"></div><div class="bad" style="width:${(100 * res.failed) / total}%"></div></div></div>`;
    }
    if (res.status === "compile_error" || res.status === "compile_timeout") {
      const errs = stripAnsi(res.compile_output).split("\n").filter((l) => /error/.test(l)).slice(0, 8);
      html += `<div class="fail-list">${errs.map((e) => `<div class="fail-item">${esc(e)}</div>`).join("")}</div>
        <div class="callout muted">Full compiler log is in the <b>Output</b> tab. Errors are underlined in the editor.</div>`;
    } else {
      const fails = parseFails(res.run_output);
      if (fails.length) html += `<div class="fail-list">${fails.slice(0, 30).map((f) =>
        `<div class="fail-item"><span class="loc">${esc(f.loc)}</span>  ${esc(f.expr)}${f.detail.length ? "\n" + esc(f.detail.join("\n")) : ""}</div>`).join("")}</div>`;
      if (res.status === "sanitizer_error" || res.status === "runtime_error") {
        const rep = stripAnsi(res.run_output).split("\n").filter((l) => /ERROR|SUMMARY|runtime error|#[0-3] .*(workspace|tests\.cpp)|READ|WRITE/.test(l)).slice(0, 14);
        html += `<div class="callout"><b>${res.status === "sanitizer_error" ? "The sanitizer caught undefined behaviour or a leak." : "The program crashed or exited non-zero."}</b>
          <pre class="mono" style="white-space:pre-wrap;margin:8px 0 0">${esc(rep.join("\n") || "(see Output tab)")}</pre></div>`;
      }
      if (res.status === "timeout") html += `<div class="callout">Your program ran longer than the time limit. Look for an infinite loop or deadlock. Tests run in a slow sanitizer build, so prefer the expected complexity.</div>`;
      if (scratch) html += `<div class="callout muted">Program output is in the <b>Output</b> tab.</div>`;
      if (res.status === "accepted" && mode === "submit") html += `<div class="callout">🎉 Solved. Compare with the <b>Editorial</b> tab, or ask Claude for an interviewer-style review.</div>`;
    }
    $("#cResults").innerHTML = html;
    if (scratch && res.status !== "compile_error") showConsole("output");
  }

  function gotoLine(n) {
    if (!S.editor || S.editor.fallback) return;
    S.editor.revealLineInCenter(n); S.editor.setPosition({ lineNumber: n, column: 1 }); S.editor.focus();
  }

  function resetConsole() {
    setChip("");
    $("#cResults").innerHTML = `<div class="muted pad">Run your code to see results. <kbd>Ctrl</kbd>+<kbd>Enter</kbd> runs public tests; <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>Enter</kbd> submits.</div>`;
    $("#cOutput").innerHTML = '<div class="muted">Compiler and program output appear here.</div>';
    if (S.consoleTab !== "terminal") showConsole("results");
  }
  function showConsole(tab) {
    S.consoleTab = tab;
    $$(".console-tabs .tab").forEach((t) => t.classList.toggle("active", t.dataset.ctab === tab));
    $("#cResults").classList.toggle("hidden", tab !== "results");
    $("#cOutput").classList.toggle("hidden", tab !== "output");
    $("#cTerminal").classList.toggle("hidden", tab !== "terminal");
    if (tab === "terminal") setTimeout(() => $("#termInput").focus(), 0);
  }

  // ------------------------------------------------------------------ terminal
  function termPrint(html, cls) {
    const d = document.createElement("div"); d.className = cls; d.innerHTML = html;
    $("#termLog").appendChild(d);
    $("#cTerminal").scrollTop = $("#cTerminal").scrollHeight;
  }
  async function termExec(cmd) {
    const prompt = $("#termPrompt").textContent;
    termPrint(`<span class="term-prompt">${esc(prompt)}</span>${esc(cmd)}`, "term-cmd");
    if (cmd.trim() === "clear") { $("#termLog").innerHTML = ""; return; }
    if (cmd.trim()) { S.termHistory = [cmd, ...S.termHistory.filter((c) => c !== cmd)].slice(0, 100); LS.set("termHistory", S.termHistory); }
    S.termIdx = -1;
    $("#termInput").disabled = true;
    const r = await api("term", { cmd });
    $("#termInput").disabled = false; $("#termInput").focus();
    if (r.output) termPrint(ansiToHtml(r.output), "term-out");
    if (r.cwd) $("#termPrompt").textContent = r.cwd + " $";
  }
  function initTerminal() {
    const inp = $("#termInput");
    inp.addEventListener("keydown", (e) => {
      if (e.key === "Enter") { const v = inp.value; inp.value = ""; termExec(v); }
      else if (e.key === "ArrowUp") { e.preventDefault(); if (S.termIdx < S.termHistory.length - 1) inp.value = S.termHistory[++S.termIdx]; }
      else if (e.key === "ArrowDown") { e.preventDefault(); S.termIdx = Math.max(-1, S.termIdx - 1); inp.value = S.termIdx >= 0 ? S.termHistory[S.termIdx] : ""; }
      else if (e.key === "l" && e.ctrlKey) { e.preventDefault(); $("#termLog").innerHTML = ""; }
    });
    $("#cTerminal").addEventListener("click", (e) => { if (!window.getSelection().toString()) inp.focus(); });
    termPrint(`<span class="muted">Shell in the repo root. Try <b>./run.sh scratch/cpp/m01_fundamentals.cpp</b>, <b>ls problems | head</b>, <b>git status</b>. Ctrl+L clears.</span>`, "term-out");
    api("term", { cmd: "", reset_cwd: true }).then((r) => { if (r.cwd) $("#termPrompt").textContent = r.cwd.replace(/^\/home\/[^/]+/, "~") + " $"; });
  }

  // ------------------------------------------------------------------ timer
  const fmtTime = (s) => { s = Math.max(0, Math.round(s)); const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), x = s % 60; return (h ? h + ":" : "") + String(m).padStart(2, "0") + ":" + String(x).padStart(2, "0"); };
  function updateTimerDisplay() {
    const el = $("#timer");
    if (!S.current || S.current.id === "scratchpad") { el.classList.add("hidden"); return; }
    el.classList.remove("hidden");
    const sec = LS.get("elapsed:" + S.current.id, 0);
    const target = (S.current.data.meta.target_min || 0) * 60;
    $("#timerText").textContent = fmtTime(sec);
    $("#timerTarget").textContent = target ? "/ " + fmtTime(target) : "";
    el.classList.toggle("over", target && sec > target);
    el.classList.toggle("paused", S.timerPaused || statusOf(S.current.id) === "solved");
  }
  function tick() {
    if (S.current && S.current.id !== "scratchpad" && !S.timerPaused && document.visibilityState === "visible" && statusOf(S.current.id) !== "solved") {
      const k = "elapsed:" + S.current.id; LS.set(k, LS.get(k, 0) + 1);
    }
    updateTimerDisplay(); tickMock();
  }

  // ------------------------------------------------------------------ mock interview
  const mockLocked = () => S.mock && !S.mock.ended && S.current && S.mock.problems.includes(S.current.id);
  function pick(list, n) {
    const unsolved = list.filter((p) => statusOf(p.id) !== "solved");
    const pool = (unsolved.length >= n ? unsolved : list).slice();
    for (let i = pool.length - 1; i > 0; i--) { const j = Math.floor(Math.random() * (i + 1)); [pool[i], pool[j]] = [pool[j], pool[i]]; }
    return pool.slice(0, n);
  }
  function buildMock(kind) {
    const code = S.catalog.problems.filter((p) => p.type === "code");
    const by = (track, mods, diffs) => code.filter((p) => p.track === track && (!mods || mods.includes(p.module)) && diffs.includes(p.difficulty));
    let probs = [], minutes = 60, title = "";
    if (kind === "slam") {
      title = "SLAM coding round";
      probs = [...pick(by("slam", ["S1", "S2", "S3", "S4", "DSA"], ["Easy", "Medium"]), 1),
               ...pick(by("slam", ["S4", "S5", "S6", "S8"], ["Medium"]), 1),
               ...pick(by("slam", ["S5", "S6", "S7", "S8"], ["Hard"]), 1)];
    } else if (kind === "cpp") {
      title = "C++ coding round"; minutes = 45;
      probs = [...pick(by("cpp", null, ["Easy"]), 1), ...pick(by("cpp", null, ["Medium"]), 1),
               ...pick(by("cpp", ["M20", "M18", "M19", "M15"], ["Medium", "Hard"]), 1)];
    } else {
      title = "Quick drill"; minutes = 20;
      probs = [...pick(by(S.track, null, ["Medium"]), 1)];
    }
    probs = [...new Map(probs.map((p) => [p.id, p])).values()];
    if (!probs.length) { toast("Not enough problems for this mock yet."); return; }
    S.mock = { kind, title, problems: probs.map((p) => p.id), ends_at: Date.now() + minutes * 60000, minutes, started: Date.now(), ended: false };
    LS.set("mock", S.mock);
    closeModal(); renderSidebar();
    location.hash = "#p/" + probs[0].id;
    toast(`Mock started: ${minutes} min. Talk through your approach as you code.`);
  }
  function openMockModal() {
    if (S.mock && !S.mock.ended) return endMock();
    openModal(`<h2>Mock Interview</h2><p class="muted">Problems are picked at random (unsolved first). Hints and editorials lock until you end the mock. Think aloud, clarify the problem first, test edge cases, and state complexity.</p>
      <div class="choice" data-k="slam"><div class="t">🛰️ SLAM coding round · 60 min</div><div class="d">3 problems: warm-up (Easy/Medium), geometry/point cloud (Medium), estimation/registration (Hard)</div></div>
      <div class="choice" data-k="cpp"><div class="t">⚙️ C++ coding round · 45 min</div><div class="d">3 problems from the C++17 track, escalating to interview-practice level</div></div>
      <div class="choice" data-k="drill"><div class="t">⚡ Quick drill · 20 min</div><div class="d">1 Medium problem from the current track</div></div>
      <div class="modal-actions"><button class="btn ghost" id="mClose">Cancel</button></div>`);
    $$("#modalCard .choice").forEach((c) => c.onclick = () => buildMock(c.dataset.k));
    $("#mClose").onclick = closeModal;
  }
  function renderMockPanel() {
    const el = $("#mockPanel");
    $("#mockBtn").textContent = S.mock && !S.mock.ended ? "End Interview" : "Mock Interview";
    if (!S.mock || S.mock.ended) { el.classList.add("hidden"); $("#mockClock").classList.add("hidden"); return; }
    el.classList.remove("hidden"); $("#mockClock").classList.remove("hidden");
    const probs = S.mock.problems.map((id) => S.catalog.problems.find((p) => p.id === id)).filter(Boolean);
    el.innerHTML = `<h4>${esc(S.mock.title)}</h4>${probs.map(probRow).join("")}<button class="btn small" id="endMock">End interview</button>`;
    $$(".prob-row", el).forEach((r) => r.onclick = () => location.hash = "#p/" + r.dataset.id);
    $("#endMock").onclick = endMock;
  }
  function tickMock() {
    if (!S.mock || S.mock.ended) return;
    const left = (S.mock.ends_at - Date.now()) / 1000;
    const c = $("#mockClock");
    c.textContent = (left < 0 ? "-" : "") + fmtTime(Math.abs(left));
    c.classList.toggle("warn", left < 600 && left >= 0);
    c.classList.toggle("over", left < 0);
    if (left < 0 && !S.mock.warned) { S.mock.warned = true; LS.set("mock", S.mock); toast("⏰ Time's up! Wrap up and end the interview."); }
  }
  function endMock() {
    if (!S.mock) return;
    const used = Math.round((Math.min(Date.now(), S.mock.ends_at + 3600e3) - S.mock.started) / 1000);
    const rows = S.mock.problems.map((id) => {
      const p = S.catalog.problems.find((x) => x.id === id); const pr = progressOf(id) || {};
      return `<tr><td>${esc(p ? p.title : id)}</td><td>${p ? p.difficulty : ""}</td><td class="st-${pr.status === "solved" ? "accepted" : pr.last_status || ""}">${pr.status === "solved" ? "Solved" : pr.last_status ? STATUS_LABEL[pr.last_status] : "Not attempted"}</td><td>${fmtTime(LS.get("elapsed:" + id, 0))}</td></tr>`;
    }).join("");
    S.mock.ended = true; LS.set("mock", S.mock);
    renderSidebar(); if (S.current) renderProbTabs();
    openModal(`<h2>Interview summary</h2><p class="muted">${esc(S.mock.title)} · time used ${fmtTime(used)} of ${S.mock.minutes}:00</p>
      <div class="md"><table><tr><th>Problem</th><th>Level</th><th>Result</th><th>Time</th></tr>${rows}</table></div>
      <p>For a debrief, ask Claude in your editor: <i>“review my mock: ${S.mock.problems.join(", ")}”</i>. You'll get a score on correctness, C++ quality, math, communication and speed.</p>
      <div class="modal-actions"><button class="btn primary" id="mClose">Close</button></div>`);
    $("#mClose").onclick = closeModal;
  }

  // ------------------------------------------------------------------ modal / toast / theme
  function openModal(html) { $("#modalCard").innerHTML = html; $("#modal").classList.remove("hidden"); }
  function closeModal() { $("#modal").classList.add("hidden"); }
  let toastT;
  function toast(msg) { const t = $("#toast"); t.textContent = msg; t.classList.remove("hidden"); clearTimeout(toastT); toastT = setTimeout(() => t.classList.add("hidden"), 3500); }
  function applyTheme(t) {
    document.documentElement.dataset.theme = t; LS.set("theme", t);
    $("#hljs-theme").href = `https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.9.0/styles/${t === "light" ? "github" : "github-dark"}.min.css`;
    if (S.monaco) S.monaco.editor.setTheme(themeName());
  }

  // ------------------------------------------------------------------ gutters
  function initGutters() {
    $$(".gutter-v").forEach((g) => {
      g.addEventListener("mousedown", (e) => {
        const target = $("#" + g.dataset.target); const startX = e.clientX, startW = target.getBoundingClientRect().width;
        g.classList.add("dragging"); document.body.style.userSelect = "none";
        const mv = (ev) => { target.style.width = Math.max(180, startW + ev.clientX - startX) + "px"; };
        const up = () => { g.classList.remove("dragging"); document.body.style.userSelect = ""; removeEventListener("mousemove", mv); removeEventListener("mouseup", up); };
        addEventListener("mousemove", mv); addEventListener("mouseup", up);
      });
    });
    const g = $("#consoleGutter");
    g.addEventListener("mousedown", (e) => {
      const c = $("#console"); const startY = e.clientY, startH = c.getBoundingClientRect().height;
      g.classList.add("dragging"); document.body.style.userSelect = "none";
      const mv = (ev) => { c.style.height = Math.max(80, Math.min(window.innerHeight - 200, startH - (ev.clientY - startY))) + "px"; };
      const up = () => { g.classList.remove("dragging"); document.body.style.userSelect = ""; removeEventListener("mousemove", mv); removeEventListener("mouseup", up); };
      addEventListener("mousemove", mv); addEventListener("mouseup", up);
    });
  }

  // ------------------------------------------------------------------ routing
  function route() {
    const h = decodeURIComponent(location.hash.slice(1));
    const [kind, arg] = h.split("/");
    $("#submitBtn").classList.remove("hidden");
    if (kind === "p" && arg) return openProblem(arg);
    if (kind === "notes" && arg) { flushSave(); return showNotes(arg); }
    if (kind === "scratch") { flushSave(); return openScratch(); }
    flushSave();
    document.title = "SLAM Practice";
    return showDashboard(kind === "dash" ? arg : undefined);
  }

  // ------------------------------------------------------------------ boot
  async function boot() {
    applyTheme(LS.get("theme", "dark"));
    initEditor(); initGutters(); initTerminal();
    $("#search").addEventListener("input", renderSidebar);
    $("#brand").onclick = () => (location.hash = "#dash/" + S.track);
    $("#runBtn").onclick = () => run("run");
    $("#submitBtn").onclick = () => run("submit");
    $("#resetBtn").onclick = async () => {
      if (!S.current) return;
      if (!confirm("Reset to the starter code? Your current code will be lost.")) return;
      if (S.current.id === "scratchpad") { LS.set("scratch", SCRATCH_TEMPLATE); setCode(SCRATCH_TEMPLATE); return; }
      await api("reset", { id: S.current.id });
      setCode(S.current.data.starter); flushSave(); clearMarkers();
    };
    $("#scratchBtn").onclick = () => (location.hash = "#scratch");
    $("#mockBtn").onclick = openMockModal;
    $("#themeBtn").onclick = () => applyTheme(document.documentElement.dataset.theme === "light" ? "dark" : "light");
    $("#timer").onclick = () => { S.timerPaused = !S.timerPaused; updateTimerDisplay(); };
    $("#modal").addEventListener("click", (e) => { if (e.target.id === "modal") closeModal(); });
    $$("#probTabs .tab").forEach((t) => t.onclick = () => { S.probTab = t.dataset.tab; renderProbTabs(); });
    $$(".console-tabs .tab").forEach((t) => t.onclick = () => showConsole(t.dataset.ctab));
    addEventListener("keydown", (e) => {
      const inField = /INPUT|TEXTAREA/.test(document.activeElement.tagName) || document.activeElement.closest?.(".monaco-editor");
      if (e.key === "/" && !inField) { e.preventDefault(); $("#search").focus(); }
      if (e.key === "Escape") closeModal();
      if ((e.ctrlKey || e.metaKey) && e.key === "Enter" && !document.activeElement.closest?.(".monaco-editor")) { e.preventDefault(); run(e.shiftKey ? "submit" : "run"); }
      if ((e.ctrlKey || e.metaKey) && e.key === "`") { e.preventDefault(); showConsole("terminal"); }
    });
    addEventListener("beforeunload", () => { if (S.saveTimer) flushSave(); });
    addEventListener("hashchange", route);
    await refreshCatalog();
    route();
    setInterval(tick, 1000);
    setInterval(async () => { // pick up new problems as they're authored
      const c = await api("catalog"); if (c.problems && c.problems.length !== S.catalog.problems.length) { S.catalog = c; renderSidebar(); renderOverall(); }
    }, 15000);
  }
  boot();
})();
