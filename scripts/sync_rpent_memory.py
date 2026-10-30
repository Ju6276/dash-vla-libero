"""Download the published RPent LIBERO memory corpus and mount its layers."""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

from huggingface_hub import hf_hub_download, snapshot_download

REPO = "RLinf/RPent-memory"
REVISION = "47f301ef33bf"
PREFIX = "libero/GPT_5.5_xhigh"
OUT = Path(
    "/homeL/dong/Projects/dash-vla-libero/runs/g5_libero/h0_official/memory_gpt55_47f301ef"
)
MOUNT = Path("/homeL/dong/Projects/dash-vla-libero/runs/g5_libero/l52b/memory")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    snapshot = Path(
        snapshot_download(
            REPO,
            repo_type="dataset",
            revision=REVISION,
            allow_patterns=[f"{PREFIX}/**", "libero/manifest.json"],
        )
    )
    manifest_path = hf_hub_download(
        REPO, "libero/manifest.json", repo_type="dataset", revision=REVISION
    )
    manifest = json.loads(Path(manifest_path).read_text())
    expected = manifest["versions"]["GPT_5.5_xhigh"]["files"]
    source = snapshot / PREFIX
    if OUT.exists():
        shutil.rmtree(OUT)
    shutil.copytree(source, OUT)
    actual = {
        path.relative_to(OUT).as_posix(): digest(path)
        for path in OUT.rglob("*")
        if path.is_file()
    }
    if actual != expected:
        missing = sorted(set(expected) - set(actual))
        extra = sorted(set(actual) - set(expected))
        mismatch = sorted(
            name
            for name in set(expected) & set(actual)
            if expected[name] != actual[name]
        )
        raise SystemExit(
            f"checksum mismatch missing={len(missing)} extra={len(extra)} bad={len(mismatch)}"
        )
    for layer in ("task-family", "task-specific", "flash"):
        destination = MOUNT / layer
        if destination.exists():
            shutil.rmtree(destination)
        shutil.copytree(OUT / layer, destination)
    shutil.copy2(OUT / "MEMORY.md", MOUNT / "MEMORY.md")
    global_bad = []
    for path in (OUT / "global").rglob("*"):
        if not path.is_file():
            continue
        current = MOUNT / "global" / path.relative_to(OUT / "global")
        if not current.is_file() or digest(current) != digest(path):
            global_bad.append(path.name)
    counts = {
        layer: sum(1 for path in (MOUNT / layer).rglob("*") if path.is_file())
        for layer in ("global", "task-family", "task-specific", "flash")
    }
    print(json.dumps({"mounted": counts, "global_mismatch": global_bad, "root": str(OUT)}))


if __name__ == "__main__":
    main()
