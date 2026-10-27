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
  opcua.der / opcua.key.der    OPC UA application instance certificate (RSA-2048: the
                               Basic256Sha256 security policy needs RSA), SAN URI = the
                               server's ApplicationUri, DNS fcu.local, IP 192.168.77.2
  opcua_client.der/.key.der    OPC UA client certificate signed by the CA (tests, asyncua;
  opcua_client.crt/.key.pem    import into UAExpert & Co.), SAN URI urn:rtoos:test-client
  opcua_rogue.der/.key.der     self-signed OPC UA client certificate: must be rejected

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
OPCUA_URI = "urn:factory-control-unit:opcua-native"  # rtos/opcua: APPLICATION_URI
OPCUA_SAN = f"URI:{OPCUA_URI},DNS:fcu.local,IP:192.168.77.2"
# The CA is name-constrained to .local (tools/make_ca.py): a client needs a .local DNS name
# (wolfSSL checks the CN as a DNS name when the SAN has none).
CLIENT_SAN = "URI:urn:rtoos:test-client,DNS:test-client.local"
# OPC UA Part 6 6.2.2: application instance certificates
OPCUA_KEY_USAGE = "critical,digitalSignature,nonRepudiation,keyEncipherment,dataEncipherment"


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


def opcua_valid(openssl: str, out: pathlib.Path, ca_crt: pathlib.Path, name: str) -> bool:
    crt = out / f"{name}.crt"
    if not all((out / f).exists() for f in (f"{name}.crt", f"{name}.der", f"{name}.key.der")):
        return False
    if not ok(openssl, "verify", "-x509_strict", "-CAfile", str(ca_crt), str(crt)):
        return False
    return ok(openssl, "x509", "-in", str(crt), "-noout", "-checkend", str(30 * 24 * 3600))


def issue_rsa(openssl: str, out: pathlib.Path, name: str, cn: str, san: str, eku: str,
              ca_crt: pathlib.Path | None, ca_key: pathlib.Path | None) -> None:
    """RSA-2048 certificate `name` (DER + PEM, key DER + PEM); self-signed without a CA."""
    key, csr, crt, ext = (out / f"{name}.key.pem", out / f"{name}.csr", out / f"{name}.crt", out / f"{name}.ext")
    ext.write_text(f"basicConstraints=critical,CA:FALSE\nkeyUsage={OPCUA_KEY_USAGE}\n"
                   f"extendedKeyUsage={eku}\nsubjectAltName={san}\nsubjectKeyIdentifier=hash\n"
                   + ("authorityKeyIdentifier=keyid\n" if ca_crt is not None else ""))
    run(openssl, "genpkey", "-algorithm", "RSA", "-pkeyopt", "rsa_keygen_bits:2048", "-out", str(key))
    run(openssl, "req", "-new", "-key", str(key), "-subj", f"/CN={cn}", "-out", str(csr))
    if ca_crt is not None:
        run(openssl, "x509", "-req", "-in", str(csr), "-CA", str(ca_crt), "-CAkey", str(ca_key),
            "-set_serial", "0x" + secrets.token_hex(8), "-days", "397", "-sha256",
            "-extfile", str(ext), "-out", str(crt))
    else:
        run(openssl, "x509", "-req", "-in", str(csr), "-signkey", str(key), "-days", "397", "-sha256",
            "-extfile", str(ext), "-out", str(crt))
    run(openssl, "x509", "-in", str(crt), "-outform", "DER", "-out", str(out / f"{name}.der"))
    run(openssl, "pkey", "-in", str(key), "-outform", "DER", "-out", str(out / f"{name}.key.der"))
    for f in (csr, ext):
        f.unlink()


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

    if not opcua_valid(openssl, out, ca_crt, "opcua"):
        issue_rsa(openssl, out, "opcua", "fcu OPC UA", OPCUA_SAN, "serverAuth,clientAuth", ca_crt, ca_key)
        print(f"make_certs: issued OPC UA certificate for {OPCUA_SAN}")
    if not opcua_valid(openssl, out, ca_crt, "opcua_client"):
        issue_rsa(openssl, out, "opcua_client", "test-client.local", CLIENT_SAN, "clientAuth,serverAuth",
                  ca_crt, ca_key)
    if not (out / "opcua_rogue.der").exists():
        issue_rsa(openssl, out, "opcua_rogue", "rogue client", "URI:urn:rogue", "clientAuth", None, None)

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
