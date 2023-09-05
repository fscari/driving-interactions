import socket

# Server configuration
host = '131.180.29.49'  # Replace with Windows IP
port = 2002

# Create socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))

# Compute first input
# ...
for i in range(10000):
    print(i)
# Notify server
message = "FIRST_INPUT_COMPUTED"
client_socket.sendall(message.encode())

client_socket.close()
