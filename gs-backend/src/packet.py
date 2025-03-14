import json
from typing import Any

class Packet:
    websocket: Any
    message: dict
    
    id: int
    op: str
    protocol: str
    method: str
    replying_to: int | None
    data: dict

    def __init__(self, websocket, message):
        self.websocket = websocket
        self.message = message

        self.id = message["i"]
        self.op = message["op"]
        self.protocol, self.method = self.op.split(":")
        self.replying_to = message["r"] if "r" in message else None
        self.data = message["data"] if "data" in message else {}

    async def reply(self, op: str, data: dict | None = None) -> None:
        message = {
            "i": 0,
            "op": op,
            "replying_to": self.id
        }

        if data is not None:
            message["data"] = data
        
        await self.websocket.send(json.dumps(message))

    async def error(self, err: str) -> None:
        print(f"ERR! {err}")
        print(f"ERR! {self.message}")
        await self.reply("error", {
            "message": err
        })
