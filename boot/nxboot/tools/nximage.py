"""Python script that prepares the NuttX image to be used with NX bootloader"""

import argparse
import os
import struct
import fastcrc
import semantic_version

class NxImage:
    def __init__(self, path: str, version: str, header_size: int, confirm: bool) -> None:
        self.path = path
        self.size = os.stat(path).st_size
        self.version = semantic_version.Version(version)
        self.header_size = header_size
        self.confirm = confirm

        with open(path,'rb') as f:
            self.crc = fastcrc.crc32.iso_hdlc(f.read(), 0xffffffff)

    def __repr__(self):
        return f"""<NxImage
  path:{self.path}
  size:{self.size}
  version:{self.version}
  header_size:{self.header_size}
  confirm:{self.confirm}
  crc:{self.crc}
>"""

    def add_header(self):
        with open(self.path,'r+b') as f:
            tmp = f.read()
            f.seek(0,0)

            s = b'\x4e\x58\x4f\x53'
            s += struct.pack('<I', self.size)
            s += struct.pack('<I', self.crc)
            s += b'\x00\x00'
            s += struct.pack('<H', self.version.major)
            s += struct.pack('<H', self.version.minor)
            s += struct.pack('<H', self.version.patch)
            s += struct.pack("@44s", bytes(self.version.prerelease[0], 'utf-8'))
            s += bytearray(self.header_size - len(s))
            f.write(s)
            f.write(tmp)

def parse_args() -> argparse.Namespace:
    """Parse passed arguments and return result."""
    parser = argparse.ArgumentParser(description="Tool for Nuttx Bootloader")
    parser.add_argument(
        "--version",
        default="0.0.0",
        help="Image version according to Semantic Versioning 2.0.0.",
    )
    parser.add_argument(
        "--header_size",
        type=lambda x: int(x, 0),
        default=0x200,
        help="Size of the image header.",
    )
    parser.add_argument(
        "--confirm",
        action='store_true',
        help="Confirm image. This should be used for factory images.",
    )
    parser.add_argument(
        "PATH",
        default="nuttx",
        help="Path to the NuttX image.",
    )
    return parser.parse_args()

def main() -> None:
    args = parse_args()
    image = NxImage(args.PATH, args.version, args.header_size, args.confirm)
    print(image)
    image.add_header()

if __name__ == "__main__":
    main()
