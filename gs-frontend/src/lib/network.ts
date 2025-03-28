export class ServerError {
    message: string;
    request: Map<string, any>;
    constructor(request: any, message: string) {
        this.message = message;
        this.request = request;
    }
}
export class ConnectionError {
    detail: any;
    constructor(e: any) {
        this.detail = e;
    }
}

export class Network {
    socket: WebSocket | undefined = undefined;
    private waiting_for: Array<{ id: number; callback: any }> = [];
    private curr_id: number = 0;
    constructor() { }
    public connect(): Promise<void | ConnectionError> {
        return new Promise((resolve, _) => {
            try {
                this.socket = new WebSocket("ws://localhost:8237");
            } catch (e) {
                resolve(new ConnectionError(e));
                return;
            }
            this.socket.addEventListener("error", (e) => {
                resolve(new ConnectionError(e));
            });
            this.socket.addEventListener("open", (_) => { });
            this.socket.addEventListener("message", (event) => {
                let obj = JSON.parse(event.data);

                let packet_idx = 0;
                for (const packet of this.waiting_for) {
                    if (packet.id == obj["r"]) {
                        packet.callback(obj);
                        this.waiting_for.splice(packet_idx, 1);
                        break;
                    }
                    packet_idx += 1;
                }
            });
        });
    }

    public request(data: any): Promise<any | ConnectionError | ServerError> {
        return new Promise((resolve, _) => {
            if (this.socket === undefined) {
                resolve(new ConnectionError("Not connected"));
                return;
            }
            this.waiting_for.push({
                id: this.curr_id,
                callback: (received: any) => {
                    if (received["op"] == "error") {
                        resolve(new ServerError(data, received.data.message));
                    } else {
                        resolve(received.data);
                    }
                }
            });
            data["i"] = this.curr_id;
            this.curr_id += 1;
            this.socket.send(JSON.stringify(data));
        });
    }
}
