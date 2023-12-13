import asyncio
from bleak import BleakClient

address = "3F:B9:89:CC:6A:0B"  # Replace with your Arduino's BLE MAC address
UUID = "87654321-4321-4321-4321-210987654321"  # The characteristic UUID

async def main(address):
    async with BleakClient(address) as client:
        connected = await client.is_connected()
        print(f"Connected: {connected}")
        
        while True:
            data = input("Enter data to send (type 'exit' to quit): ")
            if data == 'exit':
                break
            await client.write_gatt_char(UUID, data.encode())

loop = asyncio.get_event_loop()
loop.run_until_complete(main(address))
