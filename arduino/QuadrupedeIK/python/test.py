import bluetooth
import time

target_name = "SpiderBot"  # The name of your Arduino BLE, as set in your Arduino sketch
target_address = None
port = 1  # Bluetooth port (usually 1)

# Scan for devices
nearby_devices = bluetooth.discover_devices(duration=4, lookup_names=True, flush_cache=True, lookup_class=False)
print("Found {} devices.".format(len(nearby_devices)))

for address, name in nearby_devices:
    print(address, name)
    if target_name == name:
        target_address = address
        break

if target_address is not None:
    print("Found target Bluetooth device with address:", target_address)
else:
    print("Could not find target Bluetooth device nearby")
    exit()

# Create a connection
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((target_address, port))
print("Connected to", target_name)

# Send data
try:
    while True:
        data = input("Enter data to send (type 'exit' to quit): ")
        if data == 'exit':
            break
        sock.send(data)
        time.sleep(1)
except Exception as e:
    print("Error:", e)
finally:
    sock.close()
    print("Disconnected.")

