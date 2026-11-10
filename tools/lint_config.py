#!/usr/bin/env python3
"""Check a partner configuration before running anything.

    python tools/lint_config.py configs/partners/*.yaml
    python tools/lint_config.py configs/examples/new_partner_draft.yaml    # shows typical mistakes
Exit code 1 if any file has an ERROR.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from rik import config  # noqa: E402


def main(paths):
    status = 0
    for p in paths or config.default_paths():
        cfg = config.load(p)
        f = config.lint(cfg)
        errs = config.errors(f)
        print(f"{'FAIL' if errs else 'OK  '} {p}  ({len(errs)} errors, {len(f) - len(errs)} warnings)")
        for x in f:
            print("   ", str(x).replace("\n", "\n    "))
        status |= bool(errs)
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
