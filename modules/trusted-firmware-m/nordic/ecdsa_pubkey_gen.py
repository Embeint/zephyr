#!/usr/bin/env python3

import argparse

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--key", "-k", required=True, type=str, help="ECDSA .pem key file"
    )
    parser.add_argument(
        "--out", "-i", required=True, type=str, help="Output C source file"
    )
    parser.add_argument(
        "--name", "-n", required=True, type=str, help="Output C variable name"
    )
    parser.add_argument(
        "--align", type=int, default=1, help="Required alignment of variable size"
    )
    args = parser.parse_args()

    # Load the private key
    with open(args.key, "rb") as f:
        private_key = serialization.load_pem_private_key(f.read(), password=None)
    public_key = private_key.public_key()

    # Determine the curve and extract the key coordinates (X and Y)
    if isinstance(public_key.curve, ec.SECP256R1):
        coord_length = 32  # 32 bytes for P-256
    elif isinstance(public_key.curve, ec.SECP384R1):
        coord_length = 48  # 48 bytes for P-384
    else:
        raise ValueError("Unsupported elliptic curve")

    # Get the raw private key numbers (x and y coordinates)
    numbers = public_key.public_numbers()

    # Concatenate 0x04 (uncompressed format), X, and Y
    raw_key = (
        b"\x04"
        + numbers.x.to_bytes(coord_length, "big")
        + numbers.y.to_bytes(coord_length, "big")
    )
    if len(raw_key) % args.align:
        raw_key += b"\x00" * (args.align - (len(raw_key) % args.align))

    with open(args.out, "w", encoding="utf-8") as f:
        f.write(f"/* Generated from {args.key}, DO NOT EDIT */\n")
        f.write(f"static const uint8_t {args.name}[] = {{\n")
        for i in range(0, len(raw_key), 8):
            chunk = raw_key[i : i + 8]
            f.write("\t" + ", ".join(f"0x{b:02x}" for b in chunk) + ",\n")
        f.write("};\n")
