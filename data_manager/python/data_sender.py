import socket
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5007
q = []


print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT


sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
def update(a):
    global q

    q = []
    for x in a:
        q.append(x)

def send():
    global sock, q

    MESSAGE = ""

    for x in q:
        MESSAGE += "%f, " %(x)

    # print "Message (send): ", MESSAGE

    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

