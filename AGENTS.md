# fpc workspace policy

## Skill policy

- Within `/home/cvailab/fpc`, do not invoke, read, or follow any
  `superpowers:*` skill unless the user explicitly names that exact skill in
  the current request.
- Coding, debugging, planning, testing, or behavior changes do not by
  themselves authorize using `superpowers:*`.
- Do not create design documents, implementation plans, Git worktrees,
  copies, subagents, commits, or review workflows because of a
  `superpowers:*` skill unless explicitly requested.
- When the user explicitly requests one `superpowers:*` skill, use only that
  named skill. Do not automatically chain into other skills.
- Direct user instructions and the shortest task-specific workflow take
  priority.

## Writable scope

- The writable RTX 5090 project directories are `/home/cvailab/fpc` and
  `/home/cvailab/fpc-ops`.
- `/home/cvailab/fpc` is the authoritative development repository. Keep
  `/home/cvailab/fpc-ops` only as historical operations material unless the
  user explicitly asks to change or remove it.
- The only writable project path on `tsinghuaBot` is `/home/unix_ai/fpc`.
- All other paths on tsinghuaBot are read-only reference material.
- Reference material may be read and copied into the two `fpc` project paths for development.
- Keep the original reference paths read-only; modify copied working files only inside the two `fpc` project paths.
- Never use recursive write, delete, ownership, or permission commands outside
  the listed project paths.

## Validation policy

- Within `/home/cvailab/fpc`, all validation is disabled by default.
- Do not run tests, linters, formatters, type checks, syntax checks, build
  checks, import checks, smoke tests, regression suites, benchmarks, probes,
  diagnostic captures, model inference, visual inspection pipelines, hardware
  checks, service checks, deployment checks, Git cleanliness checks, or any
  other command whose purpose is to verify, validate, diagnose, or confirm a
  result unless the user explicitly approves that exact validation first.
- Before requesting approval for validation, report to the user exactly what
  will be run, why it is needed, what files, services, models, or hardware it
  will touch, and the expected time or cost. Do not begin until the user gives
  explicit approval.
- A request to implement, modify, fix, deploy, or test a feature does not imply
  permission for any additional validation. Perform only the actions explicitly
  requested by the user.
- Do not invoke Grounding DINO, SAM, OCR, depth reconstruction, camera capture,
  or any other model or sensor merely to inspect or validate an implementation.
  Each invocation requires prior explicit user approval.
- After editing, report that the change is unverified unless the user separately
  approved and requested a specific validation.

## Implementation scope policy

- The user's explicitly stated intent is the sole source of truth for all
  implementation decisions in this workspace. Never substitute the agent's own
  preferred design, engineering convention, or interpretation for it.
- Unless the user explicitly asks for ideas, alternatives, recommendations, or
  design input, do not propose, introduce, or act on any agent-generated idea.
- Every additional idea, alternative, concern, or possible change noticed during
  the work must be reported to the user before any action is taken. Reporting an
  idea does not authorize it; implementation requires separate, explicit user
  approval.
- Do not present agent-generated additions as improvements. For this workspace,
  any unrequested addition is incorrect regardless of whether it appears useful
  or technically preferable.
- Implement only the behavior, files, mechanisms, and scope that the user has
  explicitly requested and agreed to. Follow the agreed implementation exactly.
- Every implementation must be the smallest, simplest, and most direct change
  that satisfies the user's stated requirement. Unnecessary code, indirection,
  complexity, files, dependencies, configuration, and execution steps are
  strictly forbidden.
- Do not add unrequested generalization, generic platforms, abstraction layers,
  fallback paths, alternate algorithms, compatibility layers, heuristics,
  safety gates, validation gates, diagnostics, refactors, cleanup, optimizations,
  future-proofing, or supporting features.
- An idea being potentially useful, robust, reusable, conventional, safer, or
  technically preferable is never permission to implement it. All agent-initiated
  additions are forbidden unless the user explicitly requests and approves the
  exact addition first.
- Do not reinterpret a narrow task as authorization to redesign adjacent code or
  build a more general solution. Existing code outside the requested change must
  remain untouched.
- Do not silently choose missing requirements or make implementation assumptions.
  If any requirement is ambiguous, missing, contradictory, or cannot be followed
  exactly, stop immediately, make no further implementation changes, and report
  the specific issue to the user.
- If work in progress is discovered to differ from the user's agreed design,
  stop immediately and report the difference. Do not continue, repair, replace,
  or expand it without explicit user direction.
- User approval for one implementation step applies only to that exact step and
  does not authorize related or additional work.
- If there is any possibility that an action or implementation differs from the
  user's stated idea, stop before taking that action and report the possible
  difference. Do not proceed based on inference, convenience, or default practice.

## Remote operation safety

- Use the read-only mount or read-only commands when inspecting reference code.
- Deploy only committed files from a clean local `main` using
  `scripts/deploy_to_robot.sh`.
- `/home/unix_ai/fpc` is a plain runtime copy, not a Git working tree. Do not
  require it to pull, clone, or fast-forward.
- GitHub is backup-only and is not part of robot deployment.
- Never use `git reset --hard`, force push, or `rsync --delete` on the robot without explicit user approval.
- Never run hardware-motion commands without explicit user approval and a confirmed safe physical environment.
- Host aliases, IP addresses, usernames, ports, project paths, and deployment
  direction may be recorded in Git for this lab environment.
- Do not store passwords, tokens, private keys, certificate contents, datasets,
  models, logs, or caches in Git.
