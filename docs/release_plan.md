# Release plan

Status: plan, not implemented. Replaces the manual `v*` tag release job in
`.github/workflows/ci-zephyr.yml`.

## Goal

Merging a pull request from `develop` into `main` publishes a GitHub release
without manual steps:

- the version is derived automatically and stored in the Zephyr `VERSION` file;
- the change log is generated from the merged changes;
- every firmware build is attached (BCC, `native_sim`, and any board or
  simulator added later);
- the release itself is a full CI run, so the CI badge on `main` shows it.

## Today

- `app/VERSION` is the Zephyr version file (`0.1.0`), but nothing writes it
  and the firmware file names use `git describe`. There is no tag yet, so
  `git describe` yields only a hash.
- The release job runs on a pushed `v*` tag: someone tags by hand.
  A tag created by `GITHUB_TOKEN` would not start that workflow, so it cannot
  be automated as it stands.
- PRs into `develop` are squash-merged with free-form titles (`Feature/...
  (#30)`), which no tool can classify.
- The badges use `?branch=main` / `?branch=develop`. Tag runs have the tag as
  their branch and PR runs the PR branch, and `cancel-in-progress` cancels
  runs on `main` as well, so a badge can show "no status" or "cancelled".

## Decisions

### 1. Single source of truth: `app/VERSION`

Zephyr derives `APP_VERSION_STRING`, `APP_VERSION_NUMBER` and
`APP_VERSION_TWEAK_STRING` from it (`<app>/VERSION`, `app_version.h`), so the
firmware can report what the release says.

- `VERSION_MAJOR/MINOR/PATCHLEVEL` are set by the release tooling only.
- `VERSION_TWEAK` stays 0. `EXTRAVERSION` carries a pre-release (`rc1`) when
  one is wanted.
- The Cyphal `uavcan.node.GetInfo` software version and `vcs_revision_id`
  (`lib/cyphal/node.hpp`) are filled from `APP_VERSION_*` and the git hash
  (`APP_BUILD_VERSION`). Platform glue only; the core stays free of Zephyr
  headers.
- Artifact names use `APP_VERSION_STRING` instead of `git describe`.

### 2. Automatic version and change log: Conventional Commits with git-cliff

- `git-cliff` (one `cliff.toml`) computes the next version from the commits
  since the last tag and renders `CHANGELOG.md` and the release notes.
  Rules: `feat` = minor, `fix`/`perf`/`refactor` = patch, `!` or
  `BREAKING CHANGE` = major. While the major version is 0, breaking changes
  raise the minor version only.
- PRs into `develop` stay squash-merged; the PR **title** is the commit. A
  `pull_request` job (`amannn/action-semantic-pull-request`) enforces
  `type(scope): subject`. Scopes follow the layout: `motor_control`, `cyphal`,
  `dsdl`, `drive`, `startup`, `bcc`, `sim`, `ci`, `docs`.
- A DSDL or register/subject-ID change is always `feat!`/`fix!`, because it
  changes the bus protocol (`AGENTS.md`).
- The `develop` to `main` PR is merged with a **merge commit** (not squash),
  so the individual changes stay in the history that git-cliff reads.
  Branch protection on `main` allows merge commits only.
- No release is created if no releasable commit (`feat`, `fix`, `perf`,
  `refactor`, `!`) is in the range; `docs`, `ci`, `test`, `chore` alone do not
  bump.

### 3. Where the version bump is committed

The tag must point at a commit whose `VERSION` equals the tag. Bumping after
the merge would need a push to the protected `main`, so the bump goes into the
PR instead:

- Workflow `release-prepare.yml` runs on `pull_request` (base `main`, head
  `develop`) and on pushes to `develop` while such a PR is open. It runs
  `scripts/release_version.py`, which calls `git cliff --bumped-version`,
  rewrites `app/VERSION` and `CHANGELOG.md`, and commits
  `chore(release): vX.Y.Z` to `develop`. It is idempotent: the version is
  always computed from the last tag, never from the previous bump.
- It posts or updates one PR comment with the proposed version and the change
  log excerpt, so the reviewer sees the release before merging.
- The push uses a token that starts CI (a GitHub App token or a
  `RELEASE_TOKEN` PAT). With the default `GITHUB_TOKEN` the bump commit would
  not get its CI run and the required checks would stay pending.
  **This secret is the one open setup item.**
- A version can be forced by the PR label `release-as:X.Y.Z` (and a
  pre-release by `EXTRAVERSION` via label `release-as:X.Y.Z-rc1`).

### 4. Release job runs inside the CI run

Workflow `ci-zephyr.yml` stays the one CI workflow. On `push` to `main`:

1. `twister` runs as today (format, twister with clang-tidy, plain builds).
2. `release` (`needs: twister`) runs only if
   - the pushed commit is the merge of a PR whose head is `develop`
     (`gh api repos/:owner/:repo/commits/$SHA/pulls`), and
   - `v$(VERSION)` does not exist yet (makes re-runs idempotent).
3. `release` checks that `app/VERSION`, the newest `CHANGELOG.md` section and
   the computed version agree, creates the tag on that exact SHA and the
   release in one call (`gh release create vX.Y.Z --target $SHA
   --notes-file ...`). Because the tag is created by the run itself, there is
   no tag-push event to depend on, and the `v*` trigger is removed.
4. A direct push to `main` (hotfix) only runs CI and publishes nothing.

Concurrency: `cancel-in-progress` only for pull requests and `develop`; runs
on `main` are queued so a release run is never cancelled.

### 5. Assets: every build, now and later

- A build matrix replaces the hard-coded two builds:

  | target | board | files |
  |---|---|---|
  | `bcc` | `bcc` | `.elf`, `.hex`, `.bin`, `.map` |
  | `sim` | `native_sim/native/64` | `unimoc-native_sim-<ver>.tar.gz` |

  A new board or simulator is one more matrix row (name, board, file list).
  The release job downloads `unimoc-*` artifacts by pattern, so it needs no
  change.
- All files are named `unimoc-<target>-<ver>.<ext>` with the version from
  `VERSION`.
- Also attached: `SHA256SUMS`, the twister report (`twister.json`), and the
  Doxygen HTML as a zip (built in the release run, `ci-docs.yml` stays for
  pushes).
- Optional later: artifact attestation (`actions/attest-build-provenance`)
  for the firmware.

### 6. CI badge

- Badge URLs get `&event=push`, so only runs of the integration branches
  count (not PR runs or cancelled duplicates):
  `.../ci-zephyr.yml/badge.svg?branch=main&event=push`.
- Because the release is a step of the push run on `main`, the `main` badge
  is the status of the latest merge and of the release at the same time. The
  badge link opens that run.
- Add a release badge (`img.shields.io/github/v/release/Tecnologic/unimoc`)
  next to it.
- Keep the `develop` badge; PR status is shown on the PR itself.

## Files to add or change

| file | change |
|---|---|
| `cliff.toml` | commit parsers, bump rules, release-note template |
| `CHANGELOG.md` | generated, committed by `release-prepare` |
| `scripts/release_version.py` | run git-cliff, write `app/VERSION` and the change log (testable without GitHub) |
| `.github/workflows/release-prepare.yml` | bump commit and PR comment on the `develop` to `main` PR |
| `.github/workflows/pr-title.yml` | Conventional Commit check of PR titles |
| `.github/workflows/ci-zephyr.yml` | matrix, `APP_VERSION` file names, new `release` job, concurrency, no `v*` trigger |
| `lib/cyphal/`, `hardware/zephyr_common/` | GetInfo version from `APP_VERSION_*` |
| `README.md`, `docs/zephyr_workflow.md`, `TODO.md` | badges, release process, remove the manual-tag text |

## Order of work

1. `cliff.toml`, `scripts/release_version.py` and a dry run against the
   current history (no tag yet: the first release is `0.1.0` from `VERSION`
   and a hand-written first change log section).
2. `pr-title.yml`; switch the repository merge settings.
3. CI matrix, version in file names, release job; test on a throw-away
   branch with `workflow_dispatch` in dry-run mode (builds and notes, no tag).
4. `release-prepare.yml` with the token.
5. Firmware reports its version over Cyphal.
6. Badges and documentation; first real release through the
   `develop` to `main` PR (this also closes the TODO item "Merge `develop`
   into `main` as the first Zephyr release").

## Risks

- `main` still holds the legacy firmware (`TODO.md`); the first merge is a
  large history jump, so the first change log is written by hand.
- A bump commit pushed to `develop` while a feature PR is open creates a
  merge back into feature branches; limited because only `VERSION` and
  `CHANGELOG.md` change.
- Version rules depend on PR title discipline; the title check is required on
  `develop`.
