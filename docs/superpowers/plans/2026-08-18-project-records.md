# FPC Project Records Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and populate a durable documentation system that gives future agents an accurate current project view while retaining the full history of robot faults, decisions, and work.

**Architecture:** `docs/README.md` is the entry point. Stable project boundaries live in `PROJECT_MAINLINE.md`, transient state lives in `CURRENT_STATUS.md`, planned phases live in `ROADMAP.md`, and append-only historical evidence lives in `ROBOT_ISSUES.md` and `WORKLOG.md`. Cross-links connect current problems to permanent issue IDs and existing detailed specs/plans.

**Tech Stack:** Markdown, Git, shell-based link and content checks.

---

## File map

- Create `docs/README.md`: index and required agent reading order.
- Create `docs/PROJECT_MAINLINE.md`: durable mission, responsibilities, deployment flow, and workspace constraints.
- Create `docs/CURRENT_STATUS.md`: current phase, verified capabilities, open problems, and immediate next actions.
- Create `docs/ROADMAP.md`: ordered project phases and completion criteria.
- Create `docs/ROBOT_ISSUES.md`: permanent robot issue registry with stable IDs and evidence.
- Create `docs/WORKLOG.md`: reverse-chronological record of significant work and decisions.
- Preserve `docs/superpowers/specs/` and `docs/superpowers/plans/` as detailed design and execution records.

### Task 1: Documentation entry point and stable mainline

**Files:**
- Create: `docs/README.md`
- Create: `docs/PROJECT_MAINLINE.md`

- [ ] **Step 1: Create the documentation index**

Create `docs/README.md` with sections `Agent 必读顺序`, `当前工作入口`, `历史与详细设计`, and `维护规则`. The reading order starts with root `AGENTS.md`, then links `PROJECT_MAINLINE.md` and `CURRENT_STATUS.md`. Link the roadmap, issue registry, worklog, existing FruitTest perception design/plan, and this record-system design/plan.

- [ ] **Step 2: Record the stable project mainline**

Create `docs/PROJECT_MAINLINE.md` with sections `当前目标`, `系统分工`, `工作副本与参考代码`, `部署与备份`, `工作约束`, and `当前阶段不做什么`.

Record these facts without credentials:

- Work happens in a disposable FruitTest-derived copy, not in the robot's other legacy projects.
- RTX 5090 performs DINO/SAM inference; Wanda combines masks with local aligned depth, intrinsics, and robot pose to produce a `base_link` suction point.
- Editable locations are `/home/cvailab/fpc` and `/home/unix_ai/fpc`; other robot paths are read-only references that may be copied.
- `PengchengFang-cs/BookRobot` is a public GitHub backup, not the Wanda deployment transport.
- Deployment goes from the 5090 workspace to Wanda over SSH; Wanda is not expected to pull from GitHub.
- Hardware motion requires explicit user approval and a confirmed clear physical environment.
- Current focus is perception and component-level robot verification, not DataReplay or the legacy full pipeline.

### Task 2: Current status and roadmap

**Files:**
- Create: `docs/CURRENT_STATUS.md`
- Create: `docs/ROADMAP.md`

- [ ] **Step 1: Create the current project view**

Create `docs/CURRENT_STATUS.md`, dated `2026-08-18`, with sections `当前阶段`, `已完成且仍有效`, `正在进行`, `当前问题`, `下一步`, and `更新要求`.

Include verified facts:

- FruitTest was copied into controlled `fpc` workspaces and book perception replaced the HSV fruit path.
- The 5090 mask client, Wanda-local RGB-D geometry, synchronized frame selection, capture-bound deadline, perception-only entry point, tests, and review fixes exist on `main`.
- Wanda deployment uses SSH from 5090 rather than GitHub pull.
- Base forward/backward movement and lift were physically confirmed after release.
- Current problems link to `BOT-20260818-02` and `BOT-20260818-03`.
- Next work is a real-book capture and mask/suction-point validation before navigation or suction is connected.

- [ ] **Step 2: Create the phase roadmap**

Create `docs/ROADMAP.md` with five phases and completion evidence:

1. Book perception integration — code complete; real-book visual/geometry validation pending.
2. Base turning and approach — in progress; isolate rotation input and use precise odometry feedback.
3. Book suction motion — pending; replace FruitTest's fruit distances with book/suction geometry.
4. End-to-end demo — pending; connect perception, base, arm, and suction in the disposable demo.
5. Competition integration — pending; migrate only repeatably verified components into the actual task architecture.

Link detailed specs and plans where they exist.

### Task 3: Permanent robot issue registry

**Files:**
- Create: `docs/ROBOT_ISSUES.md`

- [ ] **Step 1: Add format and statuses**

Define `待处理`, `调查中`, `已绕过`, `已解决`, and `无法复现`. State that entries are never deleted and uncertain causes are labeled as hypotheses.

- [ ] **Step 2: Add `BOT-20260818-01` — base release**

Use status `已绕过`. Record that after reboot the controller graph could be healthy while movement did not respond; `release_base.sh` uses `fruit_movebase_mode_controller` to send mode `5`, then `0`; after the user handled the physical release control and restarted the robot page, forward/backward movement worked. Keep exact hardware mode semantics explicitly undocumented.

- [ ] **Step 3: Add `BOT-20260818-02` — handheld rotation input**

Use status `调查中`. Record that translation and lift worked, but both user rotation attempts produced `angular.z == 0` on `/cmd_vel`; compiled teleoperation inspection expects right-joystick horizontal input to generate angular velocity. The verified break is upstream of ROS angular-command output, and no permanent fix exists yet.

- [ ] **Step 4: Add `BOT-20260818-03` — 30-degree spin discrepancy**

Use status `已绕过`. Record these measurements:

- `/spin` target: `0.5235987756 rad`.
- Last action feedback: about `0.5735 rad` or `32.9°`.
- Final `/odom` and TF: about `1.009 rad` or `57.8°`, matching user observation.
- The approximately `24.9°` outside action feedback may be related to first post-reboot release/mode transition, but this remains a hypothesis until reproduced.
- Low-speed `/odom` feedback corrected yaw from `57.799°` to `30.307°`; later static pose was about `30.2°`, odometry twist was zero, and `diffbot_base_controller` was active.
- Do not use `/spin` as the precision primitive until isolated; use odometry feedback for controlled tests.

### Task 4: Historical worklog

**Files:**
- Create: `docs/WORKLOG.md`

- [ ] **Step 1: State append discipline**

State that new dated sections are inserted above older dates, recorded facts are not rewritten to hide mistakes, and corrections are appended with their correction date.

- [ ] **Step 2: Add the 2026-08-17 foundation record**

Record without credentials: SSH and a persistent reverse tunnel made Wanda reachable from 5090 through `tsinghuaBot`; `PengchengFang-cs/BookRobot` became the public backup while the local directory remained `fpc`; FruitTest was copied into controlled workspaces; book perception replaced fruit color detection and review fixes landed on `main`.

- [ ] **Step 3: Add the 2026-08-18 hardware record**

Record base release and restored translation (`BOT-20260818-01`), handheld rotation monitoring (`BOT-20260818-02`), the 30-degree discrepancy and odometry correction (`BOT-20260818-03`), and the decision to maintain current plus permanent historical records.

### Task 5: Verification and commit

**Files:**
- Verify: `docs/*.md`
- Verify: `docs/superpowers/specs/*.md`
- Verify: `docs/superpowers/plans/*.md`

- [ ] **Step 1: Check required files**

Run:

```bash
test -e AGENTS.md
test -e docs/README.md
test -e docs/PROJECT_MAINLINE.md
test -e docs/CURRENT_STATUS.md
test -e docs/ROADMAP.md
test -e docs/ROBOT_ISSUES.md
test -e docs/WORKLOG.md
test -e docs/superpowers/specs/2026-08-17-fruittest-book-perception-design.md
test -e docs/superpowers/plans/2026-08-17-fruittest-book-perception.md
test -e docs/superpowers/specs/2026-08-18-project-records-design.md
test -e docs/superpowers/plans/2026-08-18-project-records.md
```

Expected: every command exits `0`.

- [ ] **Step 2: Check current issue references**

Run:

```bash
rg -q 'BOT-20260818-02' docs/CURRENT_STATUS.md
rg -q 'BOT-20260818-02' docs/ROBOT_ISSUES.md
rg -q 'BOT-20260818-03' docs/CURRENT_STATUS.md
rg -q 'BOT-20260818-03' docs/ROBOT_ISSUES.md
```

Expected: every command exits `0`.

- [ ] **Step 3: Check sensitive patterns and placeholders**

Run:

```bash
! rg -n 'BEGIN (OPENSSH|RSA|EC|DSA) PRIVATE KEY|password[[:space:]]*[:=]|token[[:space:]]*[:=]' docs/*.md
! rg -n 'T[B]D|T[O]DO|待[定]|占[位]' docs/*.md
```

Expected: no output and exit status `0`.

- [ ] **Step 4: Check formatting and review the diff**

Run:

```bash
git diff --check
git diff --stat
git diff -- docs/README.md docs/PROJECT_MAINLINE.md docs/CURRENT_STATUS.md docs/ROADMAP.md docs/ROBOT_ISSUES.md docs/WORKLOG.md
```

Expected: six new documentation files consistent with `AGENTS.md`, root `README.md`, the confirmed design, and measured robot evidence.

- [ ] **Step 5: Commit**

```bash
git add docs/README.md docs/PROJECT_MAINLINE.md docs/CURRENT_STATUS.md docs/ROADMAP.md docs/ROBOT_ISSUES.md docs/WORKLOG.md docs/superpowers/plans/2026-08-18-project-records.md
git commit -m "docs: add project status and robot history"
```
