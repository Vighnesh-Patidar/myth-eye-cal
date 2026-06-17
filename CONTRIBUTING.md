# Contributing to Myth-Eye-Cal

Thanks for your interest in improving Myth-Eye-Cal. This document covers the
**contribution rules**, the **issue format**, and the **pull-request format**.
By contributing you agree that your work is licensed under the project's
[Apache 2.0 License](LICENSE).

---

## 1. Contribution rules

### Ground rules
- **Be respectful.** Assume good faith; keep discussion technical.
- **One logical change per PR.** Small, reviewable PRs merge faster than large ones.
- **Discuss big changes first.** For anything that touches the wire format
  (`KeypointFramePayload`), the fusion math, the ECS schedule, or public headers,
  open an issue and agree on the approach before writing code.
- **No AI / tool attribution in commits.** Do not add `Co-authored-by:` trailers
  for assistants/bots, and keep author/committer set to a real contributor.

### Before you open a PR, your change must
1. **Build clean** (warnings-as-noise should be fixed):
   ```sh
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ```
2. **Pass the full test suite** (currently 20/20):
   ```sh
   ctest --test-dir build --output-on-failure
   ```
3. **Pass the sanitizer gate** (zero leaks, zero UB) when you touch C++:
   ```sh
   cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DMEC_SANITIZE=address,undefined
   cmake --build build-asan -j
   ctest --test-dir build-asan --output-on-failure
   ```
4. **Add or update tests** for any behavioural change. New core logic needs a
   `tests/unit/` case; new cross-component behaviour needs `tests/integration/`.
   Register the test in `CMakeLists.txt`.
5. **Update docs** when behaviour, metrics, or architecture change
   (`README.md`, `ARCHITECTURE.md`, `docs/METRICS_REPORT.md` as relevant).

### Coding style
- **C++**: C++17, header-first core in `include/mec/`, implementations in `src/`.
  No third-party dependencies in the fusion core (keep it portable per
  ARCHITECTURE.md §13). Prefer clear names over comments; comment *why*, not *what*.
- **Kotlin/Android**: keep the core math in C++ behind the JNI bridge; the
  Android layer is capture + UI + transport only.
- **Commits**: use [Conventional Commits](https://www.conventionalcommits.org/),
  e.g. `feat(fusion): ...`, `fix(android): ...`, `docs: ...`, `ci: ...`,
  `test: ...`, `refactor: ...`. Imperative mood, concise subject (≤ 72 chars),
  body explaining the *why*.

### Branch & commit flow
- Branch from `main`: `feat/<short-name>` or `fix/<short-name>`.
- Rebase onto the latest `main` before requesting review; keep history tidy.
- CI (build matrix + sanitizers + Mith compile) must be green before merge.

---

## 2. Issue format

Open issues at the
[issue tracker](https://github.com/Vighnesh-Patidar/myth-eye-cal/issues).
Search first to avoid duplicates. Use the matching template under
`.github/ISSUE_TEMPLATE/` (these mirror the formats below).

### Bug report

```
### Summary
One sentence describing the bug.

### Environment
- Component: core / android / ci / docs
- Host OS + compiler (e.g. Linux x86-64, GCC 15.2) and/or device (model, Android version)
- Commit / tag (e.g. v1.1.0 or git SHA)
- Build type: Release / Debug / sanitizer

### Steps to reproduce
1. ...
2. ...
3. ...

### Expected behaviour
What you expected to happen.

### Actual behaviour
What actually happened (include exact error text / logcat / ctest output).

### Logs / evidence
Relevant logs, stack traces, screenshots, or a minimal repro snippet.
```

### Feature request

```
### Problem
What are you trying to do, and why is it hard / impossible today?

### Proposed solution
What you'd like to happen.

### Alternatives considered
Other approaches and why they're worse.

### Scope / impact
Which components change (core / android / ci / docs); any wire-format,
performance, or API implications.
```

---

## 3. Pull-request format

PR titles follow Conventional Commits (same as commit subjects). Fill in the
PR template (`.github/PULL_REQUEST_TEMPLATE.md`), which mirrors this:

```
## Summary
What this PR does and why (link the issue: "Closes #123").

## Changes
- Bullet list of the concrete changes.

## Testing
- [ ] `ctest --test-dir build` passes (N/N)
- [ ] Sanitizer run clean (ASan/LSan/UBSan) — for C++ changes
- [ ] New/updated tests added for the change
- [ ] Verified on device (for Android changes) — model/tier + what you observed

## Docs
- [ ] README / ARCHITECTURE / METRICS_REPORT updated as needed
- [ ] N/A

## Checklist
- [ ] One logical change, rebased on latest `main`
- [ ] Conventional-commit title, no AI/bot attribution in commits
- [ ] CI is green
```

### Review & merge
- At least one maintainer approval and green CI are required to merge.
- Maintainers may request changes for correctness, tests, performance
  regressions (check `mec_bench` if you touched hot paths), or scope.
- Squash-merge is preferred; keep the final commit message conventional and
  attribution-clean.

---

Questions? Open a discussion or a minimal issue. Thanks for contributing!
