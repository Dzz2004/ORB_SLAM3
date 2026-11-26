#!/usr/bin/env python
"""List frames per trajectory for every TartanAir V2 env by reading only the
zip central directory of image_lcam_front.zip via HTTP range requests."""
import collections
import sys
import zipfile
from concurrent.futures import ThreadPoolExecutor

import boto3
from botocore import UNSIGNED
from botocore.client import Config

ENDPOINT = "https://airlab-cloud.andrew.cmu.edu:8080/swift/v1/AUTH_ac8533a83cff4d48bc8c608ad222d330"
BUCKET = "tartanair_v2"
s3 = boto3.client("s3", endpoint_url=ENDPOINT, config=Config(signature_version=UNSIGNED, max_pool_connections=32))


class RangeFile:
    def __init__(self, key):
        self.key = key
        self.size = s3.head_object(Bucket=BUCKET, Key=key)["ContentLength"]
        self.pos = 0

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = {0: off, 1: self.pos + off, 2: self.size + off}[whence]
        return self.pos

    def read(self, n=-1):
        if n < 0:
            n = self.size - self.pos
        if n == 0 or self.pos >= self.size:
            return b""
        end = min(self.pos + n, self.size) - 1
        b = s3.get_object(Bucket=BUCKET, Key=self.key, Range=f"bytes={self.pos}-{end}")["Body"].read()
        self.pos += len(b)
        return b


def lengths(key):
    try:
        z = zipfile.ZipFile(RangeFile(key))
        c = collections.Counter(n.split("/")[2] for n in z.namelist() if n.endswith(".png"))
        return key, dict(sorted(c.items()))
    except Exception as e:
        return key, repr(e)


keys = sorted({l.split()[0] for l in open(sys.argv[1]) if l.split()[0].endswith("/image_lcam_front.zip")})
with ThreadPoolExecutor(16) as ex:
    for key, r in ex.map(lengths, keys):
        print(key, r, flush=True)
