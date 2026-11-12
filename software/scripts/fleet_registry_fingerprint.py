#!/usr/bin/env python3
"""Hash of what is inside the registry (LittleFS) image, not of the image.

The image's bytes change on every build even when the files in data/ do not.
The fleet server compares this fingerprint, so a remote is only sent the
registry again when its contents really changed.

Usage: fleet_registry_fingerprint.py [data-directory]   (default: data)
"""
import hashlib
import os
import sys

root = sys.argv[1] if len(sys.argv) > 1 else "data"
digest = hashlib.sha256()
for base, directories, files in os.walk(root):
    directories.sort()
    for name in sorted(files):
        path = os.path.join(base, name)
        relative = os.path.relpath(path, root).replace(os.sep, "/")
        with open(path, "rb") as handle:
            content = hashlib.sha256(handle.read()).digest()
        digest.update(relative.encode() + b"\0" + content)
print(digest.hexdigest())
