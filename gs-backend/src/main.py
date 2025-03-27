import asyncio
import json
from utils import *
from packet import Packet
from protocol_ws import protocol_ws
from protocol_mav import protocol_mav
from websockets.asyncio.server import serve

# {
#    "i": 0,                   <- id of packet
#    "r": 0,                   <- id of packet to reply to
#    "op": "protocol:method",  <- operation
#    "data": {}                <- data dictionary
# }

WS_PORT = 8237

async def handle_messages(websocket) -> None:
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

        packet = Packet(websocket, message)

        try:
            match packet.protocol:
                case "ws":
                    await protocol_ws(packet)
                case "mav":
                    await protocol_mav(packet)
                case _:
                    await packet.error("Invalid protocol!")
        except Exception as e:
            await packet.error(str(e))

async def main() -> None:
    print_super_amazing_ascii_text_banner()
    print_start_notice()

    print("Hello from gs-backend!")

    async with serve(handle_messages, "localhost", WS_PORT) as server:
        print(f"Server started on ws://localhost:{WS_PORT}")
        await server.serve_forever()

if __name__ == "__main__":
    asyncio.run(main())
