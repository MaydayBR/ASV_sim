import socket
import json

IP = "0.0.0.0"   # listen on all interfaces
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((IP, PORT))

print(f"Listening for UDP telemetry on {IP}:{PORT} ...")

while True:
    data, addr = sock.recvfrom(65535)
    msg = data.decode("utf-8", errors="replace")
    try:
        telem = json.loads(msg)
        print(f"[{addr}] t={telem.get('t_s')} pos=({telem.get('x')},{telem.get('y')}) "
              f"wp={telem.get('wp_reached')}/{telem.get('wp_index')} "
              f"dist={telem.get('dist_to_wp')} gps_drop={telem.get('gps_drop')} "
              f"obs_d={telem.get('nearest_obs_surface_dist')}")
    except json.JSONDecodeError:
        print(f"[{addr}] {msg}")
