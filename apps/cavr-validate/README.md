# cavr-validate

Runs the pre-execution trajectory validation of the reference welding plan
against a machine profile — joint limits, commanded speeds, axis count and
referenced tool/user frames — the same check CAVR Studio's **Validate** workflow
phase performs. Collision and singularity analysis is out of scope and is
honestly reported as "not evaluated" rather than silently passing.

```bash
cavr-validate                 # validate against the built-in GP25 cell profile
cavr-validate profile.json    # validate against a profile loaded from JSON
```

Prints each issue (severity, step, message) and a summary, and exits `1` when
validation finds errors, `0` otherwise — so it composes into scripts and CI like
a linter.
