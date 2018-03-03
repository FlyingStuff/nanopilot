import msgpack
import yaml
import sys
import os.path


def file_pack(f):
    config = yaml.load(open(f, 'r'))
    print("{}: {}".format(f, config))
    packer = msgpack.Packer(encoding='ascii', use_single_float=True)
    msg_pack_file = os.path.splitext(f)[0] + '.msgpack'
    open(msg_pack_file, 'wb').write(packer.pack(config))

for f in sys.argv[1:]:
    file_pack(f)

if len(sys.argv) == 1:
    print("usage: python {} config/*.json".format(sys.argv[0]))
