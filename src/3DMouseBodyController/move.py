import socket, json

cmd = {
    "command": "move_body",
    "name": "Body1",
    "rotate": { "axis":[0,0,1], "angle_deg": 30, "pivot": "center" },
    "translate": { "dx_mm": 10, "dy_mm": 0, "dz_mm": 0 },
    "mode": "feature"  # or "occurrence"
}

payload = (json.dumps(cmd) + "\n").encode("utf-8")
with socket.create_connection(("127.0.0.1", 8080), timeout=2.0) as s:
    s.sendall(payload)
print("Command sent.")