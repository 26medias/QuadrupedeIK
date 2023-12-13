import asyncio
from bleak import BleakClient

address = "3F:B9:89:CC:6A:0B"  # Arduino's BLE MAC address
UUID = "87654321-4321-4321-4321-210987654321"  # Characteristic UUID

async def main(address):
    async with BleakClient(address) as client:
        connected = await client.is_connected()
        print(f"Connected: {connected}")

        while True:
            command = input("Enter command (or 'exit' to quit): ")
            if command.lower() == 'exit':
                break
            
            await client.write_gatt_char(UUID, command.encode())
            if command.startswith("getShoulderAngle") or command.startswith("buildGaitFrame"):
                response = await client.read_gatt_char(UUID)
                print("Response:", response.decode())

# Use event loop for older versions of Python
loop = asyncio.get_event_loop()
loop.run_until_complete(main(address))
