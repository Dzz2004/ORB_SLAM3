#!/usr/bin/env python3
"""Download and extract the TartanAir assets used by experiment A."""

from __future__ import annotations

import argparse
import os
import zipfile
from pathlib import Path

import boto3
from botocore import UNSIGNED
from botocore.client import Config


BUCKET = "tartanair_v2"
ENDPOINT = (
    "https://airlab-cloud.andrew.cmu.edu:8080/swift/v1/"
    "AUTH_ac8533a83cff4d48bc8c608ad222d330"
)
ENVIRONMENT = "OldTownSummer"
DIFFICULTIES = ("easy", "hard")
MODALITIES = ("image_lcam_front", "depth_lcam_front", "imu")


def download(client, key: str, target: Path) -> None:
    expected_size = int(client.head_object(Bucket=BUCKET, Key=key)["ContentLength"])
    if target.exists() and target.stat().st_size == expected_size:
        print(f"SKIP {key}")
        return
    offset = target.stat().st_size if target.exists() else 0
    kwargs = {"Bucket": BUCKET, "Key": key}
    if offset:
        kwargs["Range"] = f"bytes={offset}-"
    response = client.get_object(**kwargs)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("ab" if offset else "wb") as handle:
        for chunk in response["Body"].iter_chunks(chunk_size=4 * 1024 * 1024):
            if chunk:
                handle.write(chunk)
    if target.stat().st_size != expected_size:
        raise RuntimeError(
            f"Size mismatch for {target}: {target.stat().st_size} != {expected_size}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    args = parser.parse_args()
    data_root = args.data_root.resolve()
    client = boto3.client(
        "s3",
        endpoint_url=ENDPOINT,
        config=Config(signature_version=UNSIGNED),
        region_name="us-east-1",
    )

    archives = []
    for difficulty in DIFFICULTIES:
        folder = data_root / ENVIRONMENT / f"Data_{difficulty}"
        for modality in MODALITIES:
            key = f"{ENVIRONMENT}/Data_{difficulty}/{modality}.zip"
            target = folder / f"{modality}.zip"
            download(client, key, target)
            archives.append(target)

    for archive in archives:
        print(f"VERIFY {archive}")
        with zipfile.ZipFile(archive) as handle:
            bad_member = handle.testzip()
            if bad_member is not None:
                raise RuntimeError(f"Corrupt member {bad_member} in {archive}")
            handle.extractall(data_root)
    print(f"TartanAir A data ready at {data_root}")


if __name__ == "__main__":
    main()
