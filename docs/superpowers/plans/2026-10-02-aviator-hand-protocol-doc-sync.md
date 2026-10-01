# aviator_hand Protocol Documentation Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ZMQ protocol and system architecture documents describe the current `nodes/aviator_hand/README.md` hand command, state, timing, and process model.

**Architecture:** Treat the hand README as the current behavior contract. Put exact wire fields and semantics in the ZMQ ICD; keep the architecture document at process, timing, aggregation, deployment, and capacity level while linking its statements to the same current behavior.

**Tech Stack:** Markdown, JSON examples, C++/Python implementation references, grep, Python JSON parser, CTest.

## Global Constraints

- Modify only the two requested formal documents; do not modify runtime code, README, or configuration.
- Preserve unrelated architecture and protocol content.
- Use the current independent `aviator_hand` process, 50 Hz commands, 10 Hz state, 100 ms command watchdog, and 300 ms feedback validity.
- Do not commit changes unless the user explicitly requests a commit.

---

### Task 1: Synchronize the ZMQ ICD

**Files:**
- Modify: `docs/AVIATOR_ZMQ协议格式说明.md`

**Interfaces:**
- Consumes: `nodes/aviator_hand/README.md`, `nodes/aviator_hand/hand_node.cpp`, `nodes/aviator_core/HandControl.cpp`, `config/inspire_hand.yaml`
- Produces: authoritative current `HandCommand` and `HandState` wire documentation

- [ ] **Step 1: Update ownership and timing**

Change node naming and Topic rows so `aviator_core` publishes `hand.command` at 50 Hz and `aviator_hand` publishes `hand.state` at 10 Hz. Record the 100 ms command watchdog and 300 ms feedback-validity boundary separately.

- [ ] **Step 2: Replace HandCommand definition and example**

Document only `NORMALIZED_POSITION` and `GRASP_SETPOINT`; remove `JOINT_POSITION` and `grasp.profile_id` from current capability. State that valid commands require both six-channel targets, while authenticated `valid=false` invalidation may omit `hands`.

- [ ] **Step 3: Replace HandState definition and example**

List all emitted top-level, side, and accepted-command fields. Define top-level `sample_mono_us` as state generation time and side timestamps as feedback snapshot times; describe `OFFLINE/STALE/READY/ACTIVE`, command-valid `enabled`, unsampled hardware error register, null joint fields, and feedback counters.

- [ ] **Step 4: Check the edited slice**

Run targeted searches for old `manipulator` ownership, 100 Hz hand rows, `JOINT_POSITION`, `profile_id`, and the old radians-based HandState example. Inspect the resulting diff for unrelated changes.

### Task 2: Synchronize the architecture document

**Files:**
- Modify: `docs/AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md`

**Interfaces:**
- Consumes: updated ZMQ ICD and current hand README
- Produces: architecture-level process, timing, aggregation, capacity, deployment, and implementation-status description

- [ ] **Step 1: Update process boundaries and data flow**

Separate `aviator_hand` from `manipulator` in the architecture decision, diagrams, process table, target tree, dependency text, and current implementation summary.

- [ ] **Step 2: Update timing and safety summaries**

Set hand command/state rates to 50/10 Hz and describe the 100 ms command watchdog and 300 ms feedback validity. Update thread-model and mixed-frequency statements accordingly.

- [ ] **Step 3: Update hand data semantics**

Describe normalized six-channel command/feedback, null uncalibrated joint fields, independent command/feedback validity, status values, and current lack of hardware fault and grasp verification. Adjust FlightState hand requirements and example away from fabricated radians.

- [ ] **Step 4: Recalculate capacity table**

Use 50 Hz × 1 kB = 50 kB/s for `hand.command` and 10 Hz × 1.5 kB = 15 kB/s for `hand.state`; update totals from 560 Hz/875 kB/s to 420 Hz/690 kB/s and derived hourly figures from 3.15 GB/h to 2.484 GB/h, with 1.5× budget 3.726 GB/h and 29.808 GB/8 h.

### Task 3: Validate documentation consistency

**Files:**
- Verify: `docs/AVIATOR_ZMQ协议格式说明.md`
- Verify: `docs/AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md`

**Interfaces:**
- Consumes: both edited documents
- Produces: evidence that examples parse and stale hand statements are removed

- [ ] **Step 1: Run whitespace and stale-text checks**

Run `git diff --check` and targeted `grep` searches for old hand ownership, rates, timeouts, unsupported modes, and future-work claims.

- [ ] **Step 2: Parse changed JSON examples**

Extract the full `hand.command` and `hand.state` fenced JSON blocks from the ICD and parse them with Python's `json` module.

- [ ] **Step 3: Run focused existing tests**

Run `ctest --test-dir build/debug -R '^aviator_hand_' --output-on-failure` when the configured debug build contains those tests; otherwise report the unavailable test target and retain static validation evidence.

- [ ] **Step 4: Review final diff**

Confirm every changed line traces to the hand documentation sync and summarize the final changes without committing.