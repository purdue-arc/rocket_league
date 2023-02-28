# C

import asyncio
import websockets

import rospy
from rktl_msgs.msg import ControlEffort

car_num = 0; # How do we get this?

running = True

throttle = 25565
steering = 25566

async def socket_handler(websocket):
  while running:
    packet = bytearray(throttle.to_bytes(4, byteorder='little'))
    packet.extend(steering.to_bytes(4, byteorder='little'))
    print(packet)
    print(len(packet))
    await websocket.send(packet)

async def main():
  try:
      async with websockets.serve(socket_handler, "127.0.0.1", 8765):
        await asyncio.Future()
  except asyncio.CancelledError:
      running = False

def receive_callback(data):
    throttle = data.throttle
    steering = data.steering
    print(f"Throttle: \{throttle} Steering: \{steering}")


if __name__ == '__main__':
  print("Initing")
  print("Starting")
  rospy.init_node('car_websocket', anonymous=True)
  rospy.Subscriber(f'/cars/car{car_num}/effort', ControlEffort, receive_callback)
  print("Running")
  loop = asyncio.get_event_loop()
  task = asyncio.run(main())
  for signal in [SIGINT, SIGTERM]:
    loop.add_signal_handler(signal, task.cancel)
  try:
    loop.run_until_complete(task)
  finally:
    loop.close()
  print("Spinning")
  rospy.spin()
