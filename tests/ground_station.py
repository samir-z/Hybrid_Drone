import socket, threading, csv, datetime, os

ESP32_IP = "192.168.4.1"
UDP_PORT = 14550
LISTEN_PORT = 14550

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", LISTEN_PORT))
sock.settimeout(0.1)

os.makedirs("logs", exist_ok=True)
log_filename = f"logs/flight_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
log_file = open(log_filename, "w", newline="")
csv_writer = csv.writer(log_file)
csv_writer.writerow(["raw"])
print(f"Logging to {log_filename}")

def rx_thread():
    while True:
        try:
            data, _ = sock.recvfrom(256)
            print(data.decode(errors='replace'), end='')
            csv_writer.writerow([data.decode(errors='replace').strip()])
            log_file.flush()
        except socket.timeout:
            pass

threading.Thread(target=rx_thread, daemon=True).start()

print(f"Connecting to {ESP32_IP}:{UDP_PORT}. Enter commands (Ctrl+C to exit):")
try:
    while True:
        cmd = input()
        sock.sendto((cmd + "\n").encode(), (ESP32_IP, UDP_PORT))
except KeyboardInterrupt:
    log_file.close()
    print(f"\nLog saved to {log_filename}")
    print("\nExiting...")