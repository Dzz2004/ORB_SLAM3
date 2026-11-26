#!/usr/bin/env python
"""Download TartanAir V2 zips (front cam RGB + depth + IMU) from the AirLab
mirror used by tartanairpy 1.4.0 (AirLabDownloader). Avoids importing the
tartanair package, which pulls torch/cupy.

Usage: python download_tartanair.py --env Office --diff easy hard --traj P004 --out data/raw
"""
import argparse
import os
import zipfile

import boto3
from botocore import UNSIGNED
from botocore.client import Config

ENDPOINT = "https://airlab-cloud.andrew.cmu.edu:8080/swift/v1/AUTH_ac8533a83cff4d48bc8c608ad222d330"
BUCKET = "tartanair_v2"
FILES = ["image_lcam_front.zip", "depth_lcam_front.zip", "imu.zip"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", required=True)
    ap.add_argument("--diff", nargs="+", default=["easy", "hard"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--traj", nargs="*", default=None, help="only extract these trajectories, e.g. P004")
    ap.add_argument("--no-unzip", action="store_true")
    a = ap.parse_args()
    s3 = boto3.client("s3", endpoint_url=ENDPOINT, config=Config(signature_version=UNSIGNED))
    for d in a.diff:
        for f in FILES:
            key = f"{a.env}/Data_{d}/{f}"
            dst = os.path.join(a.out, "zips", key)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if not os.path.isfile(dst):
                print("downloading", key, flush=True)
                tmp = dst + ".part"
                s3.download_file(BUCKET, key, tmp)
                os.replace(tmp, dst)
            print(key, os.path.getsize(dst) / 1e9, "GB", flush=True)
            if not a.no_unzip:
                with zipfile.ZipFile(dst) as z:
                    names = z.namelist()
                    if a.traj:
                        names = [n for n in names if n.split("/")[2:3] and n.split("/")[2] in a.traj]
                    z.extractall(a.out, members=names)


if __name__ == "__main__":
    main()
