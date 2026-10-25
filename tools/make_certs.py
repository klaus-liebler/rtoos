#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Server certificate for the FCU web server, signed by a CA (tools/make_ca.py).

  make_certs.py <openssl executable> <output directory> --ca-cert <crt> --ca-key <key>
  make_certs.py <openssl executable> <output directory>      # local test CA

with --ca-cert/--ca-key  sign with that CA; the key is only read by openssl, never
                         copied. The CA should follow tools/make_ca.py's profile.
without                  create (once) a local test CA with the same profile in the
                         output directory; import ca.crt into the trust store to use it.

Output:
  ca.crt / ca.der              the signing CA's certificate (public)
  server.der / server.key.der  server certificate + its ECDSA P-256 key, embedded
                               into the firmware; SAN DNS fcu.local, IP 192.168.77.2
  server.crt                   the same certificate (PEM)

The server certificate is (re)issued only if it is missing, does not verify strictly
(RFC 5280 profile, -x509_strict) against the current CA, lacks the SAN, or expires
within 30 days - running this at every configure is cheap. FOR DEVELOPMENT AND
TESTS ONLY: the server key is stored unencrypted in the output directory (git-ignored).
"""
import argparse
import pathlib
import secrets
import shutil
import subprocess
import sys

SAN = "DNS:fcu.local,IP:192.168.77.2"


def run(openssl: str, *args: str) -> None:
    subprocess.run([openssl, *args], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def ok(openssl: str, *args: str) -> bool:
    return subprocess.run([openssl, *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def server_cert_valid(openssl: str, out: pathlib.Path, ca_crt: pathlib.Path) -> bool:
    crt = out / "server.crt"
    if not all((out / f).exists() for f in ("server.crt", "server.der", "server.key.der")):
        return False
    if not ok(openssl, "verify", "-x509_strict", "-CAfile", str(ca_crt), "-purpose", "sslserver",
              "-verify_hostname", "fcu.local", "-verify_ip", "192.168.77.2", str(crt)):
        return False
    return ok(openssl, "x509", "-in", str(crt), "-noout", "-checkend", str(30 * 24 * 3600))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("openssl")
    ap.add_argument("out", type=pathlib.Path)
    ap.add_argument("--ca-cert", type=pathlib.Path)
    ap.add_argument("--ca-key", type=pathlib.Path)
    args = ap.parse_args()
    openssl, out = args.openssl, args.out
    out.mkdir(parents=True, exist_ok=True)
    ca_crt = out / "ca.crt"

    if args.ca_cert is not None:
        ca_key = args.ca_key
        if ca_key is None or not ca_key.exists() or not args.ca_cert.exists():
            raise SystemExit(f"make_certs: {args.ca_cert} / {ca_key} not found")
        if not ca_crt.exists() or ca_crt.read_bytes() != args.ca_cert.read_bytes():
            shutil.copyfile(args.ca_cert, ca_crt)
            run(openssl, "x509", "-in", str(ca_crt), "-outform", "DER", "-out", str(out / "ca.der"))
        (out / "ca.key").unlink(missing_ok=True)  # a former local test CA
    else:
        ca_key = out / "ca.key"
        if not ca_key.exists():
            subprocess.run([sys.executable, "-I", str(pathlib.Path(__file__).with_name("make_ca.py")), openssl,
                            str(out), "--name", "testCA"], check=True, stdout=subprocess.DEVNULL)
            (out / "testCA.pem.crt").replace(ca_crt)
            (out / "testCA.pem.key").replace(ca_key)
            run(openssl, "x509", "-in", str(ca_crt), "-outform", "DER", "-out", str(out / "ca.der"))

    if server_cert_valid(openssl, out, ca_crt):
        return
    key, csr, crt, ext = out / "server.key", out / "server.csr", out / "server.crt", out / "server.ext"
    ext.write_text("basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature\n"
                   f"extendedKeyUsage=serverAuth\nsubjectAltName={SAN}\n"
                   "subjectKeyIdentifier=hash\nauthorityKeyIdentifier=keyid\n")
    run(openssl, "ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", str(key))
    run(openssl, "req", "-new", "-key", str(key), "-subj", "/CN=fcu.local", "-out", str(csr))
    run(openssl, "x509", "-req", "-in", str(csr), "-CA", str(ca_crt), "-CAkey", str(ca_key),
        "-set_serial", "0x" + secrets.token_hex(8), "-days", "397", "-sha256",
        "-extfile", str(ext), "-out", str(crt))
    run(openssl, "x509", "-in", str(crt), "-outform", "DER", "-out", str(out / "server.der"))
    run(openssl, "ec", "-in", str(key), "-outform", "DER", "-out", str(out / "server.key.der"))
    for f in (csr, ext, key):
        f.unlink()
    if not server_cert_valid(openssl, out, ca_crt):
        raise SystemExit("make_certs: issued certificate does not verify strictly against the CA")
    print(f"make_certs: issued server certificate for {SAN}")


if __name__ == "__main__":
    main()
