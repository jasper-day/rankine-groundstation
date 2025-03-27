from packet import Packet
from path import path_adapter, solve
from pydantic import ValidationError

async def protocol_path(packet: Packet) -> None:
    id, method, replying_to, data = packet.id, packet.method, packet.replying_to, packet.data

    match method:
        case "ping":
            await packet.reply("path:pong")
        case "solve":
            try:
                path = path_adapter.validate_python(data['path'])
            except ValidationError as e:
                await packet.error(e)
            await packet.reply("path:solved", data={
                "path": path_adapter.dump_python(solve(path))
            })

        case _:
            await packet.error("Invalid method!")