"""
  Decryption and encryption of serial communication via LoRa module
"""
# the original example was crated by Helena/AI
import os
import hmac
import hashlib
import base64
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend

from osgar.node import Node
from osgar.drivers.lora import split_lora_buffer, parse_lora_packet


# --- CONFIGURATION ---
ENC_KEY = os.urandom(32)
MAC_KEY = os.urandom(32)

# This is the size of the nonce we will actually SEND.
TRANSMITTED_NONCE_SIZE = 8
TAG_SIZE = 4

# The AES block size is always 16 bytes.
AES_BLOCK_SIZE = 16


def encrypt_to_text(plaintext_bytes: bytes, enc_key: bytes, mac_key: bytes) -> str:
    """
    Encrypts a message, using a short 8-byte nonce for transmission
    but padding it to 16 bytes for the cipher.
    """
    if len(plaintext_bytes) == 0:
        raise ValueError("Plaintext cannot be empty.")

    # 1. Generate the short 8-byte nonce for transmission.
    short_nonce = os.urandom(TRANSMITTED_NONCE_SIZE)

    # 2. Pad the nonce to 16 bytes for the AES-CTR function.
    internal_nonce = short_nonce + b'\x00' * (AES_BLOCK_SIZE - TRANSMITTED_NONCE_SIZE)

    # 3. Encrypt using the padded 16-byte nonce.
    cipher = Cipher(algorithms.AES(enc_key), mode=modes.CTR(internal_nonce), backend=default_backend())
    encryptor = cipher.encryptor()
    ciphertext = encryptor.update(plaintext_bytes) + encryptor.finalize()

    # 4. Create the auth tag over the transmitted nonce and ciphertext.
    # We use the short_nonce here because that's what the recipient will see.
    h = hmac.new(mac_key, short_nonce + ciphertext, hashlib.sha256)
    tag = h.digest()[:TAG_SIZE]

    # 5. Concatenate using the SHORT nonce and encode to Base64.
    binary_blob = short_nonce + ciphertext + tag
    base64_string = base64.b64encode(binary_blob).decode('ascii')
    
    return base64_string


def decrypt_from_text(base64_string: str, enc_key: bytes, mac_key: bytes) -> bytes:
    """
    Verifies and decrypts a Base64 string that uses a short 8-byte nonce.
    """
    try:
        encrypted_blob = base64.b64decode(base64_string)
    except (ValueError, TypeError):
        raise ValueError("Invalid Base64 string.")

    if len(encrypted_blob) < TRANSMITTED_NONCE_SIZE + TAG_SIZE:
        raise ValueError("Invalid encrypted data format.")
        
    # 1. Extract the short 8-byte nonce from the message.
    short_nonce = encrypted_blob[:TRANSMITTED_NONCE_SIZE]
    tag = encrypted_blob[-TAG_SIZE:]
    ciphertext = encrypted_blob[TRANSMITTED_NONCE_SIZE:-TAG_SIZE]
    
    # 2. Verify the tag using the same short_nonce.
    h = hmac.new(mac_key, short_nonce + ciphertext, hashlib.sha256)
    expected_tag = h.digest()[:TAG_SIZE]
    if not hmac.compare_digest(tag, expected_tag):
        raise ValueError("Invalid authentication tag. Message integrity compromised.")

    # 3. Pad the short nonce to 16 bytes to use for decryption.
    internal_nonce = short_nonce + b'\x00' * (AES_BLOCK_SIZE - TRANSMITTED_NONCE_SIZE)
    
    # 4. Decrypt using the padded 16-byte nonce.
    cipher = Cipher(algorithms.AES(enc_key), mode=modes.CTR(internal_nonce), backend=default_backend())
    decryptor = cipher.decryptor()
    plaintext = decryptor.update(ciphertext) + decryptor.finalize()

    return plaintext


class Crypt(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('encrypted', 'decrypted')
        self.enc_key = bytes.fromhex(config['enc_key'])  # must be distributed among robots and basestation
        self.mac_key = bytes.fromhex(config['mac_key'])
        self.buf = b''

    def on_raw(self, data):
        """
        raw data from serial stream, to be split by '\n' in to packets and decrypted
        """
        self.buf, packet = split_lora_buffer(self.buf + data)
        while len(packet) > 0:
            # process packet
            assert packet[-1] == ord('\n'), packet
            assert packet[-2] == ord('\r'), packet  # LoRa is for some reason adding both \r\n
            assert b'|' in packet, packet
            addr, to_decode = parse_lora_packet(packet)
            print(to_decode, packet)
            self.publish('decrypted', decrypt_from_text(to_decode[:-2] + b'=', self.enc_key, self.mac_key))
            self.buf, packet = split_lora_buffer(self.buf)

    def on_packet(self, data):
        """
        LoRa plain text (ASCII) packet to be encrypted and send via serial line
        """
        # LoRa cmd packet has to end with \n character
        assert data[-1] == ord('\n'), data
        # encrypt data without cmd character
        encrypted_text = encrypt_to_text(data[:-1], self.enc_key, self.mac_key)
        self.publish('encrypted', bytes(encrypted_text, 'ascii') + b'\n')


# --- DEMONSTRATION ---
if __name__ == "__main__":
    message = "Final test. (31, 4, 8)"
    print(f"Original message: '{message}' ({len(message)} bytes)")

    encrypted_text = encrypt_to_text(message.encode('ascii'), ENC_KEY, MAC_KEY)
    print(f"Encrypted text: {encrypted_text}")
    print(f"Output length: {len(encrypted_text)} characters\n")

    try:
        decrypted_bytes = decrypt_from_text(encrypted_text, ENC_KEY, MAC_KEY)
        print(f"Decrypted message: '{decrypted_bytes.decode('ascii')}'")
    except ValueError as e:
        print(f"Decryption failed: {e}")
