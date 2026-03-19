import socket
import time
import board
import neopixel
import random

# --- hardware configurations ---
NUM_PIXELS = 28
# Use D18 in our case
pixels = neopixel.NeoPixel(board.D18, NUM_PIXELS, brightness=0.2, auto_write=False)

def color_wheel(pos):
    if pos < 0 or pos > 255: return (0, 0, 0)
    if pos < 85: return (pos * 3, 255 - pos * 3, 0)
    if pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    pos -= 170
    return (0, pos * 3, 255 - pos * 3)

def play_effect(msg):
    msg = msg.upper().strip()
    print(f"✨ message received: {msg}")
   
    if msg == "YELLOW":
        pixels.fill((255, 150, 0))
    elif msg == "GREEN":
        pixels.fill((0, 255, 0))
    elif msg == "BLUE":
        pixels.fill((0, 0, 255))
    elif msg == "RED":
        pixels.fill((255, 0, 0)) 
    elif msg == "PURPLE":
        pixels.fill((150, 0, 255)) 
    elif msg == "RAINBOW":
        # produce a random rainbow color
        for i in range(NUM_PIXELS):
            pixels[i] = color_wheel(random.randint(0, 255))
    elif msg == "OFF":
        pixels.fill((0, 0, 0))

    pixels.show()

# --- Network Listener Configuration ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Allow port reuse to prevent errors when restarting the service
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Bind to 0.0.0.0 to ensure it can receive external signals from Docker
sock.bind(('0.0.0.0', 9999)) 

print("📡 LED Base Station started, listening on port 9999...")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        try:
            msg = data.decode().strip()
            play_effect(msg)  
        except Exception as decode_err:
            print(f"Decode error: {decode_err}")
except KeyboardInterrupt:
    print("\nProgram stopped by user")
except Exception as e:
    print(f"❌ Runtime error: {e}")
finally:
    pixels.fill((0, 0, 0))
    pixels.show()
    sock.close()
