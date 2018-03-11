import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print(s)

server = "145.239.82.215"
port = 80

# request = "GET / HTTP/1.1\nHost: "+server+"\r\n"
request = "GET / HTTP/1.1\r\nHost: dupa.com\r\n\r\n"
s.connect((server, port))
print("connected")
s.send(request.encode())
result = s.recv(4096)

# print(result)

while(len(result) > 0):
    print(result)
    result = s.recv(1024)