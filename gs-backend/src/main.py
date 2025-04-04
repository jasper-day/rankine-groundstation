import asyncio
import json
from utils import *
from packet import Packet
from protocol_ws import protocol_ws
from protocol_mav import protocol_mav
from protocol_path import protocol_path
from websockets.asyncio.server import serve
from mavsdk import System
from functools import partial

# {
#    "i": 0,                   <- id of packet
#    "r": 0,                   <- id of packet to reply to
#    "op": "protocol:method",  <- operation
#    "data": {}                <- data dictionary
# }

# make event loop 1/50th second poll for aircraft info and velocity
# shove that in the direction of the websocket
# and create an IGC-style log file (CSV probably)

WS_PORT = 8237

async def handle_messages(drone: System, websocket) -> None:
    async for message in websocket:
        async def throw_error(err: str, replying_to: int | None = None) -> None:
            print(f"ERR! {err}")
            print(f"ERR! {message}")

            error_packet = {
                "i": 0,
                "op": "error",
                "data": {
                    "message": err
                },
            }

            if replying_to is not None:
                error_packet["r"] = replying_to

            await websocket.send(json.dumps(error_packet))

        try:
            message = json.loads(message)
        except json.JSONDecodeError:
            await throw_error("Invalid JSON!")
            continue

        if "i" not in message:
            await throw_error("No ID specified!")
            continue

        if "op" not in message:
            await throw_error("No operation specified!", replying_to=message["i"])
            continue

        packet = Packet(drone, websocket, message)

        try:
            match packet.protocol:
                case "ws":
                    await protocol_ws(packet)
                case "mav":
                    await protocol_mav(packet)
                case "path":
                    await protocol_path(packet)
                case _:
                    await packet.error("Invalid protocol!")
        except Exception as e:
            await packet.error(str(e))

async def main() -> None:
    print_super_amazing_ascii_text_banner()
    print_start_notice()

    print("Hello from gs-backend!")

    drone = System()
    await drone.connect()

    print("Connected to drone!")

    async with serve(partial(handle_messages, drone), "localhost", WS_PORT) as server:
        print(f"Server started on ws://localhost:{WS_PORT}")
        await server.serve_forever()

if __name__ == "__main__":
    asyncio.run(main())
