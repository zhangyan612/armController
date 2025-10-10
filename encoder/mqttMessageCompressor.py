import struct
import zlib
import base64
import random

# Generate test data: 100 integers between 0 and 9999
def generate_test_data(size=100, seed=42):
    random.seed(seed)
    return [random.randint(0, 9999) for _ in range(size)]

# 1. Binary Encoding (uint16)
def encode_binary(data):
    return struct.pack(f'>{len(data)}H', *data)

def decode_binary(payload, count):
    return list(struct.unpack(f'>{count}H', payload))

# 2. Delta Encoding + zlib Compression
def encode_delta_zlib(data):
    deltas = [data[0]] + [data[i] - data[i-1] for i in range(1, len(data))]
    packed = struct.pack(f'>{len(deltas)}h', *deltas)  # signed 16-bit
    return zlib.compress(packed)

def decode_delta_zlib(payload, count):
    unpacked = struct.unpack(f'>{count}h', zlib.decompress(payload))
    result = [unpacked[0]]
    for delta in unpacked[1:]:
        result.append(result[-1] + delta)
    return result

# 3. Base64 Encoding of Binary
def encode_base64(data):
    binary = struct.pack(f'>{len(data)}H', *data)
    return base64.b64encode(binary)

def decode_base64(payload, count):
    binary = base64.b64decode(payload)
    return list(struct.unpack(f'>{count}H', binary))

# 4. Bit Packing (14 bits per number)
def encode_bitpack(data):
    bitstream = 0
    for num in data:
        bitstream = (bitstream << 14) | num
    byte_length = (14 * len(data) + 7) // 8
    return bitstream.to_bytes(byte_length, 'big')

def decode_bitpack(payload, count):
    bitstream = int.from_bytes(payload, 'big')
    result = []
    for _ in range(count):
        result.insert(0, bitstream & ((1 << 14) - 1))
        bitstream >>= 14
    return result

# Run tests and compare sizes
def test_all_encodings(data):
    count = len(data)
    methods = {
        "JSON": str(data).encode('utf-8'),
        "Binary": encode_binary(data),
        "Delta+zlib": encode_delta_zlib(data),
        "Base64": encode_base64(data),
        "BitPack": encode_bitpack(data)
    }
    print(encode_bitpack(data))
    print(decode_bitpack(methods["BitPack"], count))

    print(f"\n📊 Encoding Size Comparison for {count} integers:")
    for name, payload in methods.items():
        print(f"{name:<12}: {len(payload):>5} bytes")

    # Verify correctness
    assert decode_binary(methods["Binary"], count) == data
    assert decode_delta_zlib(methods["Delta+zlib"], count) == data
    assert decode_base64(methods["Base64"], count) == data
    assert decode_bitpack(methods["BitPack"], count) == data
    print("\n✅ All decodings verified successfully.")

if __name__ == "__main__":
    test_data = generate_test_data(size=30)
    test_all_encodings(test_data)