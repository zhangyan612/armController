"""rs485_send.py

RS485 sender for motor controllers using RTS toggle method that works.
Accepts commands as parameters for easy testing.

Usage:
  python rs485_send.py "1C;"                    # Query device 1
  python rs485_send.py "1C;" "2C;" "3C;"        # Query multiple devices
  python rs485_send.py "1A=45.0;" "1E=1;"       # Multiple commands to device 1
  python rs485_send.py --port COM6 "1C;"        # Different port
  python rs485_send.py --test                   # Run common commands test
  python rs485_send.py --interactive            # Interactive mode
"""

import time
import sys
import argparse
try:
    import serial
except Exception as e:
    print('pyserial required: pip install pyserial')
    raise

# Default settings
DEFAULT_PORT = 'COM5'
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 1.0

def send_blocked_message(msg, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=1.0, 
                        rts_settle=0.08, inter_byte_delay=0.005, post_tx_wait=0.08):
    """Assert RTS once, write message bytes one-by-one while keeping DE asserted,
    then wait and deassert. This avoids DE glitches between characters."""
    
    if not msg:
        print("No message provided!")
        return None
        
    if not msg.endswith(';'):
        msg = msg + ';'  # Auto-add semicolon if missing
        
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f'Opened {port} at {baud} bps')
    except Exception as e:
        print(f'Failed to open {port}: {e}')
        return None

    response_data = bytearray()
    try:
        print(f"--> Sending blocked message: {msg!r}")
        
        # Assert RTS (DE) for transmit
        ser.setRTS(True)
        time.sleep(rts_settle)
        
        # Send message one byte at a time with small delays
        for b in msg.encode('ascii'):
            ser.write(bytes([b]))
            ser.flush()
            time.sleep(inter_byte_delay)
            
        # Wait for transmission to complete
        time.sleep(post_tx_wait)
        
        # Deassert RTS
        ser.setRTS(False)

        # Read response with timeout
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                response_data.extend(chunk)
                # Check if we have a complete response (ends with newline)
                if response_data.endswith(b'\n') or response_data.endswith(b'\r\n'):
                    break
            else:
                time.sleep(0.01)

    except Exception as e:
        print(f"Error during communication: {e}")
    finally:
        try:
            ser.close()
        except Exception:
            pass
            
    return response_data

def display_response(response_data, original_command):
    """Display response in multiple formats for easy reading."""
    
    if not response_data:
        print('  No response received')
        print('  (Device may be offline, wrong ID, or command format error)')
        return
        
    # Remove trailing whitespace and newlines for cleaner display
    response_clean = response_data.rstrip(b'\r\n\t ')
    
    if not response_clean:
        print('  Empty response received')
        return
        
    print('✓ Response received:')
    
    # 1. Show as raw bytes (hex)
    hex_str = response_clean.hex()
    print(f'  Hex: {hex_str}')
    
    # 2. Show as decimal bytes
    ints = ','.join(str(b) for b in response_clean)
    print(f'  Bytes: [{ints}]')
    
    # 3. Show as ASCII/UTF-8 (if printable)
    try:
        printable = response_clean.decode('utf-8', errors='replace')
        print(f'  Text: {printable!r}')
    except Exception:
        print(f'  Text: <cannot decode as UTF-8>')
    
    # 4. Show length
    print(f'  Length: {len(response_clean)} bytes')

def send_commands(commands, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT):
    """Send multiple RS485 commands using the working blocked-message method."""
    
    if not commands:
        print("No commands provided!")
        return
    
    print(f"Sending {len(commands)} command(s) to {port} at {baud} baud")
    print("-" * 50)
    
    for i, cmd in enumerate(commands):
        print(f'\nCommand {i+1}/{len(commands)}:')
        
        response = send_blocked_message(cmd, port, baud, timeout)
        display_response(response, cmd)
        
        # Brief pause between commands
        if i < len(commands) - 1:
            time.sleep(0.2)

def test_common_commands(port, baud):
    """Test a set of common commands to verify device functionality."""
    
    test_commands = [
        "1C",      # Query encoder values (device 1)
        "1E=1",    # Enable motor (device 1)  
        "1E=0",    # Disable motor (device 1)
        "1A=90.0", # Set angle to 90° (device 1)
        "1B=100",  # Set reduction ratio to 100 (device 1)
        "1Q=5",    # Set rotation count to 5 (device 1)
    ]
    
    print("=== Testing Common Commands ===")
    send_commands(test_commands, port, baud)

def interactive_mode(port, baud):
    """Interactive mode for sending commands one by one."""
    
    print("\n=== Interactive Mode ===")
    print("Enter commands to send (press Ctrl+C to exit)")
    print("Examples: 1C, 1E=1, 1A=45.0")
    print("Type 'test' to run common commands test")
    print("Type 'quit' to exit")
    
    while True:
        try:
            user_input = input("\nEnter command> ").strip()
            
            if user_input.lower() in ['quit', 'exit', 'q']:
                break
            elif user_input.lower() == 'test':
                test_common_commands(port, baud)
                continue
            elif user_input:
                response = send_blocked_message(user_input, port, baud)
                display_response(response, user_input)
                
        except KeyboardInterrupt:
            print("\nExiting interactive mode...")
            break
        except Exception as e:
            print(f"Error: {e}")

def main():
    parser = argparse.ArgumentParser(description='Send RS485 commands to motor controller using RTS toggle method')
    parser.add_argument('commands', nargs='*', help='Commands to send (e.g., "1C" "1E=1") - semicolon auto-added')
    parser.add_argument('--port', default=DEFAULT_PORT, help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT, help=f'Response timeout (default: {DEFAULT_TIMEOUT})')
    parser.add_argument('--test', action='store_true', help='Run common commands test')
    parser.add_argument('--interactive', action='store_true', help='Start interactive mode')
    
    args = parser.parse_args()
    
    print(f"RS485 Motor Controller Tester (RTS Toggle Method)")
    print(f"Port: {args.port}, Baud: {args.baud}, Timeout: {args.timeout}s")
    print("=" * 50)
    
    if args.test:
        test_common_commands(args.port, args.baud)
    elif args.interactive:
        interactive_mode(args.port, args.baud)
    elif args.commands:
        send_commands(args.commands, args.port, args.baud, args.timeout)
    else:
        print("No commands specified. Use --help for usage information.")
        print("\nExamples:")
        print("  python rs485_send.py \"1C\"")
        print("  python rs485_send.py \"1C\" \"1E=1\" \"1A=45.0\"")
        print("  python rs485_send.py --test")
        print("  python rs485_send.py --interactive")
        print("\nNote: Semicolon is automatically added to commands")

if __name__ == '__main__':
    # main()
    send_commands(['1A=3.14'])