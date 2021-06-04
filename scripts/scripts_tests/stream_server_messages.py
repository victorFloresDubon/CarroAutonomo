import socket

server_ip = '192.168.0.7'
server_port = 7691

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((server_ip, server_port))

s.listen(5)
print("Esperando por conexiones")
while True:
    cliente_socket, direccion = s.accept()
    print(f"Enviando datos de prueba a {direccion}")
    cliente_socket.send(bytes("D", "utf-8"))
    cliente_socket.send(bytes("R", "utf-8"))
    cliente_socket.send(bytes("P", "utf-8"))
    cliente_socket.send(bytes("F", "utf-8"))