from packet import Packet
from utils import *
from mavsdk.offboard import (Attitude, OffboardError)

async def protocol_mav(packet: Packet) -> None:
    id, method, replying_to, data = packet.id, packet.method, packet.replying_to, packet.data

    match method:
        case "arm":
            try:
                await packet.drone.action.arm()
            except:
                await packet.error("Failed to arm")
                return
            
            await packet.reply("mav:arm", {
                "success": True
            })
        case "disarm":
            try:
                await packet.drone.action.disarm()
            except:
                await packet.error("Failed to disarm")
                return
            
            await packet.reply("mav:disarm", {
                "success": True
            })
        case "takeoff":
            try:
                await packet.drone.action.takeoff()
            except:
                await packet.error("Failed to take off")
                return
            
            await packet.reply("mav:takeoff", {
                "success": True
            })
        case "land":
            try:
                await packet.drone.action.land()
            except:
                await packet.error("Failed to land")
                return
            
            await packet.reply("mav:land", {
                "success": True
            })
        case "kill":
            try:
                await packet.drone.action.kill()
            except:
                await packet.error("Failed to kill")
                return
            
            await packet.reply("mav:kill", {
                "success": True
            })
        case "reboot":
            try:
                await packet.drone.action.reboot()
            except:
                await packet.error("Failed to reboot")
                return
            
            await packet.reply("mav:reboot", {
                "success": True
            })
        case "offboard_start":
            try:
                await packet.drone.offboard.start()
            except:
                await packet.error("Failed to start offboard mode")
                return
            
            await packet.reply("mav:offboard_start", {
                "success": True
            })
        case "offboard_stop":
            try:
                await packet.drone.offboard.stop()
            except:
                await packet.error("Failed to stop offboard mode")
                return
            
            await packet.reply("mav:offboard_stop", {
                "success": True
            })
        case "set_attitude":
            assert_item_float("roll_deg", data, -180, 180)
            assert_item_float("pitch_deg", data, -180, 180)
            assert_item_float("yaw_deg", data, -180, 180)
            assert_item_float("thrust_value", data, 0, 1)

            if not await packet.drone.offboard.is_active():
                await packet.error("Offboard mode is not active")
                return

            attitude = Attitude(
                roll_deg=data["roll_deg"],
                pitch_deg=data["pitch_deg"],
                yaw_deg=data["yaw_deg"],
                thrust_value=data["thrust_value"]
            )

            try:
                await packet.drone.offboard.set_attitude(attitude)
            except OffboardError as err:
                await packet.error(f"Plane is being disobedient (failed to set attitude). Panic!!!\nReceived: {err}")
                return
            
            await packet.reply("mav:offboard_start", {
                "success": True,
                "data": data,
            })
        case _:
            await packet.error("Invalid method!")
