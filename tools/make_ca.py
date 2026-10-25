#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Create the project's development root CA (once).

  make_ca.py <openssl executable> <directory> [--name rtoosCA]

Writes <name>.pem.crt and <name>.pem.key into <directory>; refuses to overwrite.

Profile (chosen for this project; RFC 5280 compliant, passes strict verifiers):
  key            ECDSA P-256, signatures ecdsa-with-SHA256 - the curve the device and
                 its TLS stack use anyway (no RSA anywhere in the chain)
  basicConstraints  critical, CA:TRUE, pathlen:0 (signs server certificates only)
  keyUsage       critical, keyCertSign, cRLSign
  subjectKeyIdentifier  (issued certificates get a matching authorityKeyIdentifier)
  nameConstraints  critical, permitted: DNS *.local, IPv4 10/8, 172.16/12, 192.168/16
                 - even a leaked key cannot vouch for public names or addresses
  validity       10 years
The key is stored unencrypted (the build signs server certificates unattended);
keep the directory private.
"""
import argparse
import pathlib
import subprocess
import tempfile

CONFIG = """\
[req]
distinguished_name = dn
prompt = no
x509_extensions = v3_ca

[dn]
O = Klaus Liebler personal
CN = rtoos FCU development CA

[v3_ca]
basicConstraints = critical, CA:TRUE, pathlen:0
keyUsage = critical, keyCertSign, cRLSign
subjectKeyIdentifier = hash
nameConstraints = critical, permitted;DNS:local, permitted;IP:10.0.0.0/255.0.0.0, permitted;IP:172.16.0.0/255.240.0.0, permitted;IP:192.168.0.0/255.255.0.0
"""


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("openssl")
    ap.add_argument("directory", type=pathlib.Path)
    ap.add_argument("--name", default="rtoosCA")
    args = ap.parse_args()
    crt = args.directory / f"{args.name}.pem.crt"
    key = args.directory / f"{args.name}.pem.key"
    if crt.exists() or key.exists():
        raise SystemExit(f"make_ca: {crt} or {key} exists - not overwriting")
    with tempfile.TemporaryDirectory() as tmp:
        cnf = pathlib.Path(tmp) / "ca.cnf"
        cnf.write_text(CONFIG)
        subprocess.run([args.openssl, "req", "-x509", "-new", "-config", str(cnf),
                        "-newkey", "ec", "-pkeyopt", "ec_paramgen_curve:prime256v1", "-noenc",
                        "-keyout", str(key), "-out", str(crt), "-days", "3650", "-sha256"],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    print(f"make_ca: created {crt} and {key}")


if __name__ == "__main__":
    main()
