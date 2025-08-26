import asyncio
import websockets

async def echo(websocket):
    async for message in websocket:
        await websocket.send(f"Echo: {message}")

async def main():
    async with websockets.serve(echo, "localhost", 8765):
        print("WebSocket server running at ws://localhost:8765")
        await asyncio.Future()

asyncio.run(main())
