from packet import Packet

async def protocol_mav(packet: Packet) -> None:
    id, method, replying_to, data = packet.id, packet.method, packet.replying_to, packet.data

    match method:
        case "ping":
            await packet.reply("mav:pong")
        case _:
            await packet.error("Invalid method!")
