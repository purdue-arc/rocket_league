# C

import asyncio
import websockets

import rospy
from rktl_msgs.msg import ControlEffort

car_num = 0; # How do we get this?

running = True

class WebSocketNode():
  def __init__(self):
    self.throttle = 25565
    self.steering = 25566

  def receive_callback(self, data):
    self.throttle = data.throttle
    self.steering = data.steering

  def node_listener(self):
    rospy.init_node('car_websocket', anonymous=True)
    rospy.Subscriber(f'/cars/car{car_num}/effort', ControlEffort, self.receive_callback)

  async def socket_handler(self, websocket):
    while running:
      packet = bytearray(self.throttle.to_bytes(4, byteorder='little'))
      packet.extend(self.steering.to_bytes(4, byteorder='little'))
      print(packet)
      print(len(packet))
      await websocket.send(packet)

  async def main(self):
    self.node_listener()
    async with websockets.serve(self.socket_handler, "192.168.13.133", 8765):
      await asyncio.Future()



if __name__ == '__main__':
  node = WebSocketNode()
  asyncio.run(node.main())
