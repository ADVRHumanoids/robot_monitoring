import subprocess
from subprocess import run
import netifaces as ni
import tempfile

def generate_mkcert_certificate():
    """ "Generates a https certificate for localhost and selected addresses. This
    requires that the mkcert utility is installed, and that a certificate
    authority key pair (rootCA-key.pem and rootCA.pem) has been generated. The
    certificates are written to /tmp, where the http server can find them
    ater on."""

    # detect nic
    addresses = ["127.0.0.1"]
    addresses += [
        addr[ni.AF_INET][0]["addr"]
        for addr in map(ni.ifaddresses, ni.interfaces())
        if ni.AF_INET in addr
    ]
    addresses = list(set(addresses))  # deduplicate
    addresses.sort()

    cert_file = tempfile.NamedTemporaryFile(
        mode="w+b", prefix="qtwasmserver-certificate-", suffix=".pem", delete=True
    )
    cert_key_file = tempfile.NamedTemporaryFile(
        mode="w+b", prefix="qtwasmserver-certificate-key-", suffix=".pem", delete=True
    )

    # check if mkcert is installed
    try:
        out = subprocess.check_output(["mkcert", "-CAROOT"])
        root_ca_path = out.decode("utf-8").strip()
        print(
            "Generating certificates with mkcert, using certificate authority files at:"
        )
        print(f"   {root_ca_path}       [from 'mkcert -CAROOT'] \n")
    except Exception as e:
        print("Warning: Unable to run mkcert. Will not start https server.")
        print(e)
        print(f"Install mkcert from github.com/FiloSottile/mkcert to fix this.\n")
        return False, None, None

    # generate certificates using mkcert
    addresses_string = f"localhost {' '.join(addresses)}"
    print("=== begin mkcert output ===\n")
    ret = run(
        f"mkcert -cert-file {cert_file.name} -key-file {cert_key_file.name} {addresses_string}",
        shell=True,
    )
    print("=== end mkcert output ===\n")
    has_certificate = ret.returncode == 0
    if not has_certificate:
        print(
            "Warning: mkcert is not installed or was unable to create a certificate. Will not start HTTPS server."
        )
    return has_certificate, cert_file, cert_key_file