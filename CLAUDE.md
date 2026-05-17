# AMR Project — Claude Rules

## Execution Discipline

- **Execute, don't deliberate.** When a plan task is clear, do it. Do not re-explain what you are about to do, ask for confirmation on obvious steps, or summarize what you just did. Just execute.
- **No hallucination.** If you are unsure about a fact (a package name, an API, a file path), check it — run a command, read a file, search. Never invent answers. A wrong answer costs more tokens to fix than a `ros2 pkg list | grep X` costs to verify.
- **No filler text.** Do not write "Great!", "Sure!", "Let me help you with that", or any variation. Start every response with the first useful word.
- **No trailing summaries.** After completing a task, do not summarize what you just did. The diff speaks for itself.
- **One update per key moment.** While working: one sentence when you find something, one sentence when you change direction, one sentence when you hit a blocker. Nothing else.

## Plan Execution Rules

- Read the full plan file before starting: `docs/superpowers/plans/2026-05-17-amr-implementation-plan.md`
- Execute tasks exactly as written. Do not skip steps, reorder them, or add steps that are not in the plan.
- After completing each task, mark it done and immediately move to the next step. Do not pause and ask "shall I continue?" unless you have hit a genuine blocker that requires user input.
- If a step says "run this command", run it and report the actual output — not what you expect the output to be.
- If a step fails, diagnose the root cause before trying a fix. Do not blindly retry or bypass with `--force` / `--no-verify`.

## Token Conservation

- Never restate the plan or task description back to the user — they wrote it.
- Never write multi-paragraph explanations for straightforward actions.
- Never ask clarifying questions that can be answered by reading the plan or memory files.
- Check memory at `/home/m0mspagetthi/.claude/projects/-home-m0mspagetthi-AMR/memory/` before asking the user for context that may already be stored there.

## Project Context

- **Dev machine:** WSL2 Ubuntu 22.04 (where Claude Code runs)
- **Deploy target:** Raspberry Pi 5 8GB, Ubuntu 24.04, ROS2 Jazzy
- **All ROS2 production code targets Jazzy.** Never write Humble-specific code for deployment artifacts.
- **Spec:** `docs/superpowers/specs/2026-05-17-amr-system-design.md`
- **Plan:** `docs/superpowers/plans/2026-05-17-amr-implementation-plan.md`
