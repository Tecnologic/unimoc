#!/usr/bin/env python3
"""Writes the MCUboot release signing key from the environment to a PEM file.

The key comes from the CI secret ``MCUBOOT_SIGNING_KEY`` (``build-firmware.yml``).
A secret pasted through a web form or a shell often loses its PEM framing: the
line breaks turn into spaces or into a literal ``\\n``, Windows line ends or
indentation creep in, or the file was stored base64 encoded. imgtool then fails
deep in the build with ``MalformedFraming``. This script restores the framing,
checks with ``openssl`` that the result is an ECDSA P-256 private key and fails
with a clear message otherwise. It never prints the key.

Usage: KEY=... write_signing_key.py <output.pem>
"""

import base64
import binascii
import os
import re
import subprocess
import sys

_PEM_RE = re.compile(
    r"-----BEGIN ([A-Z ]+)-----(.*?)-----END \1-----", re.DOTALL)


def _fail(message: str) -> None:
    print(f"::error::MCUBOOT_SIGNING_KEY: {message}", file=sys.stderr)
    sys.exit(1)


def normalize(text: str) -> str:
    """Returns the key as PEM with its framing restored."""
    text = text.strip().replace("\r", "").replace("\\n", "\n")
    if "-----BEGIN" not in text:
        try:
            der = base64.b64decode("".join(text.split()), validate=True)
        except binascii.Error:
            _fail("no '-----BEGIN ... PRIVATE KEY-----' line and not base64 either")
        try:
            # The whole PEM file, base64 encoded once more.
            text = der.decode("ascii").strip().replace("\r", "")
        except UnicodeDecodeError:
            # Only the body between the BEGIN and END lines (DER): PKCS#8 as
            # imgtool keygen writes it, or SEC1 as openssl ec writes it.
            label = "EC PRIVATE KEY" if der[2:5] == b"\x02\x01\x01" else "PRIVATE KEY"
            text = f"-----BEGIN {label}-----\n{text}\n-----END {label}-----"
    match = _PEM_RE.search(text)
    if match is None:
        _fail("the BEGIN and END lines are missing or do not match "
              "(was the key cut off when it was pasted?)")
    label, body = match.group(1), "".join(match.group(2).split())
    if "PRIVATE KEY" not in label:
        _fail(f"holds a '{label}', not the private key")
    if "ENCRYPTED" in label:
        _fail("the private key is encrypted; store it without a password")
    lines = [body[i:i + 64] for i in range(0, len(body), 64)]
    return "\n".join([f"-----BEGIN {label}-----", *lines,
                      f"-----END {label}-----", ""])


def main() -> None:
    if len(sys.argv) != 2:
        _fail("usage: write_signing_key.py <output.pem>")
    key = os.environ.get("KEY", "")
    if not key.strip():
        _fail("is not available: a release must not be signed with the debug key")
    pem = normalize(key)
    path = sys.argv[1]
    os.umask(0o077)
    with open(path, "w", encoding="ascii") as out:
        out.write(pem)
    check = subprocess.run(["openssl", "pkey", "-in", path, "-noout", "-text"],
                           capture_output=True, text=True, check=False)
    if check.returncode != 0:
        os.remove(path)
        _fail("openssl cannot read the key (bad base64 inside the PEM?)")
    if "prime256v1" not in check.stdout and "P-256" not in check.stdout:
        os.remove(path)
        _fail("is not an ECDSA P-256 key (imgtool keygen -t ecdsa-p256)")


if __name__ == "__main__":
    main()
