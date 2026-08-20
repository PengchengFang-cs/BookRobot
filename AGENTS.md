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
