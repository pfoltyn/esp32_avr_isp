#!/usr/bin/env python3
import sys

def parse_hex_line(line):
    if not line.startswith(':'):
        return None
    length = int(line[1:3], 16)
    address = int(line[3:7], 16)
    rectype = int(line[7:9], 16)
    data = line[9:9+length*2]
    checksum = line[9+length*2:9+length*2+2]
    return (length, address, rectype, data, checksum)

def write_hex_line(address, rectype, data_bytes):
    length = len(data_bytes)
    line = ':%02X%04X%02X' % (length, address, rectype)
    data_str = ''.join(['%02X' % b for b in data_bytes])
    line += data_str
    # Calculate checksum
    checksum = length + ((address >> 8) & 0xFF) + (address & 0xFF) + rectype + sum(data_bytes)
    checksum = ((~checksum + 1) & 0xFF)
    line += '%02X' % checksum
    return line.upper()

def reformat_hex(input_file, output_file, bytes_per_line=16):
    data_records = []
    extended_addr = 0
    # First, parse all data records into a flat address:data dict
    memory = {}
    with open(input_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or not line.startswith(':'):
                continue
            length, address, rectype, data, checksum = parse_hex_line(line)
            if rectype == 0:  # Data record
                for i in range(length):
                    byte = int(data[i*2:i*2+2], 16)
                    mem_addr = extended_addr + address + i
                    memory[mem_addr] = byte
            elif rectype == 4:  # Extended linear address
                extended_addr = int(data, 16) << 16
            elif rectype == 1:  # End of file
                break
            # (Other record types ignored for simplicity)

    # Now, write out in desired format
    addresses = sorted(memory.keys())
    if addresses:
        min_addr = addresses[0]
        max_addr = addresses[-1]
    else:
        min_addr = max_addr = 0

    with open(output_file, 'w') as out:
        addr = min_addr
        while addr <= max_addr:
            # Gather up to bytes_per_line bytes
            data_bytes = []
            for i in range(bytes_per_line):
                a = addr + i
                if a in memory:
                    data_bytes.append(memory[a])
                else:
                    break
            if data_bytes:
                out.write(write_hex_line(addr, 0, data_bytes) + '\n')
            addr += len(data_bytes)
        # Write EOF
        out.write(':00000001FF\n')

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 reformat_hex.py input.hex output.hex")
        sys.exit(1)
    reformat_hex(sys.argv[1], sys.argv[2], bytes_per_line=16)
    print("Done. Reformatted file written to", sys.argv[2])

