# E-E-A-T for the Manual — Experience, Expertise, Authority, Trust

Google's E-E-A-T framework, adapted from claude-blog's `eeat-signals.md` (MIT) for doc/help
entries. The blog machinery (author bios, Person schema, author pages, backlinks) does not
transfer — a manual has no bylines. What transfers is the evaluation stance: pages earn trust
through demonstrated first-hand use and honest limits, and **trust is evaluated first** — a
page with wrong or oversold claims scores low no matter how expert the prose reads.

Run this as part of the review workflow (after the factcheck, whose verdicts feed Trust),
and use the signal tables while writing.

## The four signals, translated

### Experience — was this written by someone who ran the feature?

In a manual, experience shows through *specificity*, never through first-person anecdote
("in our experience..." is blog voice and the linter's tutorial-voice rules reject it).

| Signal | What it looks like |
|--------|--------------------|
| Real worked examples | A frame, script, or config a reader can paste and run; actual output shown, not described |
| Real values | Actual port names per platform (`COM3`, `/dev/ttyUSB0`), actual defaults, actual error text |
| Lived edge cases | The DTR-reset-on-connect note, the multicast checkbox that only matters on UDP — details invisible in a screenshot |
| Executed procedures | Quick-start steps in the order the UI actually presents them, including the step people miss |

Anti-signal: steps and descriptions that could be written from the settings dialog alone.
A page with no example, no concrete value, and no edge case documents nothing the UI doesn't.

### Expertise — is the depth real and proportional?

| Signal | What it looks like |
|--------|--------------------|
| Correct domain terminology | Protocol terms used as the field uses them (parity, QoS, PGN), matching `Glossary.md` |
| Edge cases and boundaries | What happens at the limits: max rates, empty frames, disconnects |
| Depth matches complexity | Modbus gets register-model theory; Process I/O gets a paragraph — not uniform padding |
| Accurate mechanics | Behavior claims that survive the ground-truth factcheck |

Anti-signal: hedged mechanics ("this typically handles...") — an expert checked, so the page
states; see the hedge-stacking tell in second-order-tells.md.

### Authoritativeness — is this page the canonical reference?

| Signal | What it looks like |
|--------|--------------------|
| One source of truth | No two doc/help pages asserting different facts about the same feature; fix every mirror |
| UI-consistent naming | The page's terms match the app's strings verbatim |
| Cited standards | Where the manual leans on an external spec (Modbus, CANopen, MQTT), name and link the spec rather than paraphrasing it as original |
| Graph membership | Registered in `help.json`, cross-linked both directions (heuristic 10) |

### Trustworthiness — the foundation; evaluate it first

| Signal | What it looks like |
|--------|--------------------|
| Honest scope | What the feature does *not* do, stated plainly, not buried |
| Gating disclosed up front | Pro-only features say so in the overview, before the reader invests |
| Platform caveats | Where behavior differs by OS, the page says so and says how |
| Verified claims | Zero WRONG verdicts from the factcheck; NOT FOUND claims flagged, not shipped |
| No overselling | Capability described at its actual size; the linter's marketing rules are the floor, not the bar |

Anti-signals (kept from the blog list, they apply verbatim): "it is well known that...",
unfalsifiable consensus claims, generic prose that could describe any product, and demanding
trust ("rest assured...") instead of earning it with specifics.

## Scoring flow

Trust first, then the other three:

```
Trust: any WRONG claim, undisclosed gating, or oversold capability → page fails, stop
Trust adequate → score Experience / Expertise / Authority 0-4 each (manual-heuristics scale)
```

Severity mapping: failed Trust is P0. Experience 0-1 (no example, no concrete value anywhere)
is P1 — the page is a UI screenshot in prose. Expertise and Authority gaps are P2 unless they
contradict another page (then P1, and fix both).

## Report format

```
## E-E-A-T: <page>

Trust: PASS / FAIL (evidence: factcheck verdicts, gating disclosure, scope honesty)
Experience: 0-4 — <strongest signal present / missing>
Expertise: 0-4 — <note>
Authority: 0-4 — <note>

Findings fold into the review report's P0-P3 list alongside the heuristics rubric.
```
