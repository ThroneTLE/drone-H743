---
spec: 0008-assistant-auto-canary
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-14
---

# Plan 0008 — Assistant Context Health & Compounding Workflows (Weak-Model-First)

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Grounded in full reads of `Conversation.cpp` (key regions),
> `ContextBuilder.cpp`, `Assistant.h`, `ChatStore.h`, `Provider.h`, `CommandRegistry.h`,
> `Redactor.h`, and the QML surface. Gate: do not start `/ss-tasks` until a human marks
> this `approved`.

## Approach (one paragraph)

Five small app-driven mechanisms bolt onto the existing single-writer seams of the assistant,
none of which trusts the model to orchestrate itself. A `SentinelProbe` appends a short
instruction to the (cached) role block, strips/validates the trailing sentinel at the two
places all rendered text already flows through (`flushPendingStreamUpdate` /
`onReplyFinished`), and persists per-provider+model compliance in `QSettings`. A `MemoryStore`
(sidecar JSON modeled on `ChatStore`) holds consent-gated facts whose capped index rides as an
uncached tail system block; a deterministic handoff digest is computed app-side from the chat
and stored additively in the chat snapshot; a `SkillRouter` matches user text against a
bundled trigger vocabulary and injects the skill body as a synthetic `meta.loadSkill`
tool_use/tool_result pair so the existing history-aging machinery manages its cost; and an
auto-verify table in `Conversation` runs the matching dry-run/read-back after apply-class
mutations, folding the outcome into the tool result and the tool-call card. Everything is
toggleable from the existing Settings menu, logged under `serialStudioAI`, and provider-neutral
because it all happens in `Conversation`/`ContextBuilder`, not in providers.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/AI/SentinelProbe.h/.cpp` | **New.** Sentinel text + instruction block; streaming-safe strip (trailing-holdback); validate (healthy/mutated/missing + drifted segment); per-provider+model compliance state in `QSettings` group `ai/compliance`. |
| `app/src/AI/MemoryStore.h/.cpp` | **New.** Sidecar `memory.json` beside `ai/chats/`: fact list `{id, category, text, projectPath?, createdAt, lastUsedAt}`, caps + LRU eviction, capped index string builder, CRUD. `Redactor::scrub` on every write. |
| `app/src/AI/SkillRouter.h/.cpp` | **New.** Loads `:/ai/skill_triggers.json`; matches user text → skill ids; per-conversation dedup; builds the synthetic tool pair. |
| `app/src/AI/ContextBuilder.h/.cpp` | Add `memoryIndexBlock()` + `handoffBlock(text)`; `buildSystemArray` appends them (and the probe instruction into `roleBlock` when enabled) — memory/handoff go **after** the live-state block (uncached tail); probe instruction goes at the end of the role block (cached, stable). |
| `app/src/AI/Conversation.h/.cpp` | Wire-up: `start()` calls `SkillRouter` before `issueRequest()`; `flushPendingStreamUpdate` applies `SentinelProbe::stripForDisplay` beside `rewriteHelpLinks`; `onReplyFinished` validates, strips from `m_assistantText` before the history append, updates probe state; `recordToolResult` runs the auto-verify table and folds `verification` into the payload + card; snapshot gains additive keys `handoff`, `loadedSkills`, `probe`; deterministic handoff digest builder. |
| `app/src/AI/Assistant.h/.cpp` | Five new toggles (`ai/contextProbe`, `ai/memory`, `ai/handoffSeeding`, `ai/skillRouting`, `ai/autoVerify`) as `Q_PROPERTY` following the `allowDeviceControl` pattern; owns `MemoryStore`; QML API for the memory manager (`memoryList`, `addMemory`, `updateMemory`, `deleteMemory`, `confirmProposedMemory`); `newChatFromHandoff(id)`; probe status properties (`contextDegraded`, `degradationDetail`). |
| `app/src/AI/ChatStore.h/.cpp` | None structurally (snapshot keys are additive and `loadSnapshot` ignores unknowns); optional `handoff` line in `Meta`/index if the sidebar should show it — **deferred, not in scope**. |
| `app/src/API/Handlers/AssistantHandler.cpp` | Register `assistant.memory.propose` (returns `{queued:true}`; persistence happens only via the user's confirmation chip). Declaration added in `AssistantHandler.h`. |
| `app/src/AI/ToolDispatcher.cpp` | **Added during review** — third leg of the assistant.* triple registration: route `assistant.memory.propose` through `executeAssistantTool`'s `runCommand` fallthrough + catalog entry (the recorded "dual registration" trap; without it the command is unroutable from the AI path). |
| `app/rcc/ai/command_safety.json` | Add `assistant.memory.propose` to the `safe` tier. |
| `app/rcc/ai/skill_triggers.json` | **New.** Curated per-skill trigger vocabulary (id → keyword/phrase list). |
| `app/rcc/rcc.qrc` | Register `skill_triggers.json`. |
| `app/qml/AI/AssistantPanel.qml` | Degradation banner (clone of `dropToast` pattern, persistent variant) adjacent to `workingStripe`; 5 Settings-menu toggles; "Remember this" affordance; memory-manager menu entry; memory-proposal confirmation chip. |
| `app/qml/AI/MemoryManagerDialog.qml` | **New.** Browse/edit/delete remembered facts (list + per-row edit/delete, category filter). |
| `app/qml/AI/ToolCallCard.qml` | Verified/failed badge from the new `verification` card field; failed state offers the restore affordance (pre-filled `assistant.restore` confirmation flow). |
| `app/qml/AI/ChatSidebar.qml` | "New chat from handoff" action on chats that carry one. |
| `app/CMakeLists.txt` | Add the three new C++ TUs + QML file. |
| `scripts/generate-sdk.py` output (`app/rcc/api/*`) | Regenerated by the sanitize pipeline after the new command registers (no manual edit). |

## Architecture & data flow

All on the **main thread**; no new threads, no hotpath contact.

1. **Probe.** `ContextBuilder::roleBlock()` (already rebuilt per request) appends the sentinel
   instruction when `Assistant::contextProbeEnabled()`. Reply text flows unchanged into
   `m_assistantText`; `flushPendingStreamUpdate()` renders
   `SentinelProbe::stripForDisplay(rewriteHelpLinks(text))` — the holdback keeps the last
   partial line unrendered only while it could still be a sentinel prefix, so no flash and no
   mid-stream mutation verdicts. `onReplyFinished()` (only for final replies, i.e. no pending
   tool calls) runs `SentinelProbe::validate(m_assistantText)`, strips the sentinel before the
   history append (models never see their own past sentinels; copy/export are clean because
   both read the stripped text), updates the per-conversation state machine, and — if the
   model was compliant and now failed — raises `Assistant::contextDegraded`. QML shows the
   banner; its action calls `newChatFromHandoff(activeChatId)`.
2. **Compliance.** `SentinelProbe` keys on `providerKey + '/' + currentModel()`; first-N-reply
   window (N=3 visible replies) decides compliant/non-compliant; stored via `QSettings`
   (`ai/compliance/<key> = {compliant, decidedAt}`) so known-weak models stay muted across
   sessions (spec R5). A changed model id naturally resets.
3. **Memory.** `Assistant` owns `MemoryStore`. `ContextBuilder::memoryIndexBlock()` reads the
   capped index (≤ ~1.5 KB) through `Assistant::instance()` — same pattern as
   `liveProjectStateBlock()`'s dispatcher use — and appends it after the live-state block.
   Writes only via user action: manual "Remember this", the memory manager, or confirming a
   chip raised by `assistant.memory.propose`. Every write passes `Redactor::scrub`.
4. **Handoff.** On `persistActiveChat()` the digest is recomputed: chat title, last ≤3 user
   asks, successful mutating tool-call names+targets, checkpoint paths, ≤ ~600 bytes; stored
   as `snapshot["handoff"]`. `newChatFromHandoff(id)` creates a chat whose conversation
   carries the seed; `ContextBuilder::handoffBlock()` emits it as an uncached tail block
   (wrapped `<untrusted source="previous_chat">` since it quotes user/tool text) for the life
   of that chat.
5. **Skill routing.** `Conversation::start()` (after `.trimmed()`, before `issueRequest()`)
   asks `SkillRouter::match(text, loadedSkills)`; on a hit it appends the synthetic pair to
   `m_history`: assistant message with a `tool_use` of `meta.loadSkill{name}` (app-generated
   id, marked `_synthetic:true`) + user message with the `tool_result` body — exactly the
   shape a compliant model produces, so `ageHistoryToolResults()` and
   `reconcileHistoryToolPairs()` manage it with zero new machinery. A small UI chip on the
   user row shows "skill loaded: X". Dedup set persists in the snapshot.
6. **Auto-verify.** A static table maps apply-class commands to their read-back:
   `project.frameParser.setCode → project.frameParser.dryRun(dryCompile form)`,
   `project.dataset.setTransformCode → project.dataset.transform.dryRun`,
   `project.painter.setCode → project.painter.dryRun`,
   `project.outputWidget.update(transmitFunction) → project.outputWidget.dryRun`,
   `assistant.project.bulkApply → scan its own failureCount` (no extra call),
   `assistant.script.apply → read its own steps[]` (no extra call).
   After a successful mutating result, `recordToolResult` runs the verify command through the
   dispatcher (all Safe-tier, read-only), folds `verification:{ok, detail}` into the
   tool_result payload (model sees it same turn — no extra round trip for Haiku) and into the
   card map (user sees the badge). Device-gated commands are never in the table.

## Hotpath & threading impact

- **Touches the hotpath?** **No.** All changes live in `app/src/AI/` + QML + one API handler;
  none of `FrameReader`/`CircularBuffer`/`FrameBuilder`/`Dashboard` is touched. The only
  dashboard-adjacent call is the existing Safe-tier read commands the verifier reuses.
- **New cross-thread signal/slot?** **No.** Everything runs on the main thread; network
  replies already arrive there via Qt's event loop.
- **New input to a cached hotpath flag?** **No.**
- **Timestamp ownership** — untouched; no frame-path code involved.

## Data model & persistence

- **Chat snapshot** stays `schema: 1` with additive keys (`handoff: string`,
  `loadedSkills: [id]`, `probe: {state, sawSentinel}`) — `loadSnapshot()` already ignores
  unknown keys, old chats load unchanged, old builds ignore the new keys.
- **`memory.json`** (new, `AppLocalDataLocation/ai/memory.json`): `{schema:1, facts:[...]}`,
  hard caps (default 100 facts / 4 KB total index; LRU eviction by `lastUsedAt`).
- **`QSettings`**: five new `ai/*` toggle keys + `ai/compliance/*` group. No migration needed.
- **No project-file (`.ssproj`) changes** — memory deliberately never enters shareable files.

## API / SDK surface

- One new command: `assistant.memory.propose {category, text}` in `AssistantHandler.cpp`
  (registered alongside the existing `assistant.*` rails), `safe` tier in
  `command_safety.json`, returns `{queued:true}` — it never persists directly, so it cannot
  violate R13 even if a model spams it. `sanitize-commit.py`'s `generate-sdk` step regenerates
  `api-schema.json`/SDK. Commercial guard: the whole assistant is already commercial; no new
  `#ifdef BUILD_COMMERCIAL` needed beyond what `AssistantHandler.cpp` has.
- Tool advertisement: `assistant.memory.propose` joins `essentialToolNames()` only when memory
  is enabled; on `needsSmallToolSurface` models it is still included (it is tiny and is the
  one model-side affordance R13 wants).

## QML / UI

- **Degradation banner**: persistent non-modal strip above the composer (sibling of
  `workingStripe`, visual pattern from `dropToast`), text from `degradationDetail`
  (missing vs mutated + drifted segment on expansion), action button "Start fresh chat"
  → `newChatFromHandoff`. Never blocks the composer (spec constraint).
- **Settings menu** (`AssistantPanel.qml:245` area): five checkable items following the
  `autoApproveEdits` MenuItem pattern.
- **Memory**: "Remember this" appears in the message context area; proposal chip renders when
  a `memory_proposal` row/flag arrives; `MemoryManagerDialog.qml` opened from the Settings
  menu. Deleting a fact takes effect on the next request (index is rebuilt per request).
- **ToolCallCard**: `verification` field renders a ✓/✗ badge; ✗ expands to the detail plus a
  "Restore checkpoint…" button that pre-arms the existing confirm flow for
  `assistant.restore` (alwaysConfirm tier still applies — the user clicks the actual approve).
- All new strings via `qsTr()`; no `.ts`/`.qm` regeneration in this feature.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Memory storage home | Sidecar `memory.json` / QSettings blob / embed in `.ssproj` | **Sidecar** — browsable+editable as one unit, mirrors ChatStore precedent; `.ssproj` embedding leaks private facts into shareable files (disqualified). |
| Memory & handoff injection point | Uncached tail block after live state / cached block before the breakpoint | **Uncached tail** — index is capped small (≤ ~400 tokens); paying it uncached every turn beats invalidating the whole cached prefix on every memory edit (prompt caching is prefix-match, `ContextBuilder.cpp:821`). |
| Skill injection shape | Synthetic `meta.loadSkill` tool pair / extra text block on the user turn | **Synthetic pair** — identical to what a compliant model produces, so `ageHistoryToolResults` + `reconcileHistoryToolPairs` manage its cost for free; a user-turn block never ages. |
| Handoff generator | Deterministic app-built digest / model-written summary | **Deterministic digest** — zero tokens, works with any model, honors "assume the model never self-orchestrates"; a model summary is exactly the discipline weak models fail. |
| Memory proposal path | Manual-only / model-tool-only / **manual + safe-tier propose tool** | **Both** — manual affordance is the deterministic baseline (R13 holds with zero model cooperation); the tool is a graceful enhancement whose worst case is "no proposal". |
| Sentinel strip scope | **Rendered text only** / rendered + history | **Rendered only** — reversed during review: stripping prior sentinels from history few-shot-teaches the model to *omit* the sentinel, manufacturing false "Missing" alarms (violates the probe's deciding constraint). History keeps the raw reply; display/copy/export read the stripped UI text, satisfying R2. Costs ~20 tokens per past turn and reinforces compliance on weak models. |
| Compliance record home | `QSettings ai/compliance/*` / sidecar JSON | **QSettings** — mirrors `ai/model/*`, tiny fixed-size records, no loader code. |
| Verify delivery | Fold into the same tool_result / separate follow-up message | **Fold in** — Haiku sees pass/fail in the same turn without an extra round trip; a separate message costs a full request. |

## Risks & mitigations

- **Prompt-cache regression** (spec constraint): all per-chat/per-turn dynamic content
  (memory index, handoff, live state) stays after the cache breakpoint; the probe instruction
  lives in the role block, which is stable per settings-state. Verify with the existing
  `cacheReadTokens`/`cacheCreatedTokens` counters before/after.
- **Sentinel straddles stream chunks** — strip operates on the *accumulated* `m_assistantText`
  at flush time (the `rewriteHelpLinks` precedent), with a trailing holdback window; never on
  individual chunks. Validation only at `onReplyFinished`.
- **Skill body bloats small-surface contexts** — bodies are injected once per chat (dedup),
  aged by the existing tool-result aging, and `budgetedHistory` already cuts at fresh user
  turns; largest body (`dashboard_layout`, 691 lines) is the same cost as the model loading it
  itself. Trigger vocabulary must be high-precision (curated, no fuzzy matching in v1).
- **False skill-routing hits** — curated triggers err toward misses; a miss = status quo (the
  model can still `meta.loadSkill`). Log every match for tuning (R9/AC-level observability).
- **Synthetic tool pair confuses a provider translation layer** — the pair is
  Anthropic-message-shaped exactly like real pairs already in history; provider translators
  (OpenAI/Gemini/…) already handle real `meta.loadSkill` pairs. Scripted-endpoint AC3/AC10
  exercises this.
- **Auto-verify triggers a confirm prompt** — the table only contains Safe-tier read-only
  dry-runs; enforced with a `Q_ASSERT` + registry check at table load (silent-breakage class:
  a future rename moving a verify target to Confirm would otherwise prompt the user out of
  nowhere).
- **Memory grows unbounded / stores secrets** — hard caps + LRU in `MemoryStore`;
  `Redactor::scrub` on every write; refusal when the scrub redacts everything.
- **Trust boundary** — handoff text and memory facts quote user/tool content, so both blocks
  are wrapped in `<untrusted source=...>` envelopes with `neutralizeUntrustedDelimiter`,
  same as project state.
- **`common-mistakes.md` exposure**: setter-guard pattern on all new Assistant properties;
  no `%n`+`.arg()` mixing in new translated strings; no heavy work in dialog-close slots
  (memory dialog mutates the store directly, not via file dialogs).

## Test & verification plan

- **Unit (I can run):** none — no `tests/scripts/` JS-parser surface is touched.
- **Integration (maintainer runs, app up with API server on `localhost:7777`):**
  - New `tests/integration/test_assistant_memory.py`: `assistant.memory.propose` returns
    `{queued:true}` and does **not** create a fact (R13); command appears in `meta`/schema.
  - Existing `assistant.*` tests stay green (checkpoint/restore untouched).
- **Maintainer observation (scripted local provider endpoint where noted):**
  - AC1/AC2 (no sentinel visible / clean copy-export) — hosted frontier model.
  - AC3/AC4 (mutated/missing sentinel → banner; never-compliant model stays muted, incl.
    across new chats) — scripted endpoint replaying canned replies.
  - AC5 (probe off → no instruction in request) + AC9 (memory off → zero memory bytes) —
    `QT_LOGGING_RULES=serialStudioAI.debug=true` request-composition log lines added in
    `issueRequest` (block names + byte sizes, never payloads with secrets).
  - AC6 (≤ ~30 completion tokens) — existing token counters.
  - AC8/AC12/AC13 (memory recall, handoff continue, edit/delete round-trip) — default Haiku.
  - AC10 (skill body present before first domain tool call, no model-initiated load) —
    composition log.
  - AC11 (auto-verify pass/fail badge + restore affordance) — scripted endpoint with a
    canned bad `project.frameParser.setCode` call.
  - AC14 (secret refused/redacted) — attempt to remember an API-key-shaped string.
  - AC7/AC15 — `serialStudioAI` log inspection.
- **Static:** `python scripts/code-verify.py --check` on every touched file; `qt-cpp-review` +
  `qt-qml-review` before handoff; `python scripts/sanitize-commit.py` before commit (also
  regenerates the SDK for the new command).
