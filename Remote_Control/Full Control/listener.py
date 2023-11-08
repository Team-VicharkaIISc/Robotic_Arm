# Echo server program
import socket
rospy.init_node('Robotic_ARM')
pub = rospy.Publisher('Controller_Output', String, queue_size=10)
import rospy
from std_msgs.msg import String

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 50007              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print('Connected by', addr)
while True:
    data = conn.recv(1024)
    if not data: break
    print(data) # Paging Python!
    pub.publish(data)
    # do whatever you need to do with the data
conn.close()
# optionally put a loop here so that you start 
# listening again after the connection closes
