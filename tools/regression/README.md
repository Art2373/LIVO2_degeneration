# Full-sequence frontend regression

The fixed migration regression uses the admitted Wheel-float01 input, 0.5 playback, given fusion model allowed at +15 sensor seconds, faults at +43–51, and 3 wall seconds of observation after natural EOF. H/R/B/S run exactly once, in that order. The tool stops on execution or injection errors and does not retry. Strategy outcomes are retained, not used to move events.

Use a fresh evidence directory **outside the source package**. Paths are explicit; the package has no dependency on another source checkout. Commands below assume the project is in the current directory and a clean independent build has completed:

```bash
PROJECT="$PWD"
WS="$(cd ../.. && pwd)"
EVIDENCE="/path/to/fresh-regression-output"
BAG="/path/to/Wheel-float01.bag"
python3 tools/regression/register.py --bag "$BAG" --workspace "$WS" \
  --output "$EVIDENCE" --env "$PROJECT/tools/env.sh" --port 11331
python3 tools/regression/batch.py --plan "$EVIDENCE/RUN_PLAN.json" --execute
python3 tools/regression/analyze.py --root "$EVIDENCE" --out "$EVIDENCE/analysis"
```

Registration checks the admitted bag checksum and input connection types, freezes source/configuration/binary hashes, and constructs an RGB pixel-identity ledger using the matching `cv_bridge` decoder. An already verified ledger may instead be supplied explicitly with `--contract /path/to/input_contract.json`; it is copied to the evidence directory and checked against actual plugin output. It is evidence input, not a software dependency. Do not infer success from a launched PID: `batch_summary.json`, each run's `manifest.json`, native player Done/EOF and all child return codes must agree.

For deliberate serial supervision, `run.py --plan ... --case H` (then R, B, S) enforces completion of earlier cases. Do not also launch the batch worker. The dedicated master uses the selected port; only processes started by the runner are stopped. The bag/player is not trimmed or fast-forwarded. Sources with a different sensor/profile require separate admission and a separately registered protocol.

A is the historical finite-retained event, B is the final surviving pose-information plus finite-commit event, and C is actual manager credit. These are separate fields; none is localization accuracy or full visual-health truth. The recovery predicate is at least three consecutive qualifying calls spanning at least 0.2 corrected-image seconds; applying/withdrawing a model resets the chain. EOF observation never retroactively counts as pre-EOF recovery. PROBING updates the real filter.

Read-only checks:

```bash
tools/env.sh python3 tools/regression/test_runtime.py
tools/env.sh python3 tools/regression/test_analysis.py
```

Pixel replacement affects only RGB bytes. Timestamp, frame, resolution, encoding and delivery rhythm are preserved; the model provider is independent. Service restart uses a new session and separate log directory with identical parameter values. Input counts, image identity, queue ledger, actual loaded libraries and post-call feedback are recorded. Instrumentation carries overhead; results do not establish real-time capacity.
