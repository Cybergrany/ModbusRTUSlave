#!/usr/bin/env python3
"""Fail closed when immutable Git identities in the fork lock are malformed."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import re


COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}")
COMMIT_KEYS = {
    "commit",
    "reviewed_head",
    "ogm_branch_point",
    "source_commit",
    "ogm_slave_core_merge",
}


def _validate_commit(value: object, path: str) -> None:
    if not isinstance(value, str) or COMMIT_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{path} must be one full lowercase 40-hex commit ID")


def validate_lock(document: dict) -> None:
    def visit(value: object, path: str) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                child_path = f"{path}.{key}" if path else key
                if key in COMMIT_KEYS:
                    _validate_commit(child, child_path)
                visit(child, child_path)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                visit(child, f"{path}[{index}]")

    visit(document, "")

    tuple_path = "clean_public_layout.firmware_artifact_comparison.consumer_tuple"
    consumer_tuple = document["clean_public_layout"]["firmware_artifact_comparison"][
        "consumer_tuple"
    ]
    if not consumer_tuple:
        raise ValueError(f"{tuple_path} must not be empty")
    for repository, commit in consumer_tuple.items():
        _validate_commit(commit, f"{tuple_path}.{repository}")


def _self_test(document: dict) -> None:
    validate_lock(document)
    malformed = copy.deepcopy(document)
    malformed["clean_public_layout"]["firmware_artifact_comparison"][
        "consumer_tuple"
    ]["OGM_Slave_prod"] += "d"
    try:
        validate_lock(malformed)
    except ValueError:
        return
    raise RuntimeError("fork-lock validator accepted a non-40-character commit")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    lock_path = Path(__file__).resolve().parent.parent / "ogm-fork-lock.json"
    document = json.loads(lock_path.read_text(encoding="utf-8"))
    if args.self_test:
        _self_test(document)
        print("fork lock validator self-test: PASS")
    else:
        validate_lock(document)
        print("fork lock immutable identities: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
