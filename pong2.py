"""Example using zmq with asyncio coroutines"""
# Copyright (c) PyZMQ Developers.
# This example is in the public domain (CC-0)

import time

import zmq
from zmq.asyncio import Context, Poller
import asyncio

url = 'tcp://127.0.0.1:5555'

url2 = 'tcp://127.0.0.1:5556'
ctx = Context.instance()


async def ping():
    """print dots to indicate idleness"""
    while True:
        await asyncio.sleep(0.5)
        print('.')


async def receiver():
    """receive messages with polling"""
    pull = ctx.socket(zmq.PULL)
    pull.connect(url)
    poller = Poller()
    poller.register(pull, zmq.POLLIN)
    while True:
        events = await poller.poll()
        if pull in dict(events):
            print("recving", events)
            msg = await pull.recv_multipart()
            print('recvd', msg)


async def sender():
    """send a message every second"""
    tic = time.time()
    push = ctx.socket(zmq.PUSH)
    push.bind(url2)
    currentmsg = 1
    msg = 'hello'
    while True:
        if currentmsg == 1:
            print("sending 1")
            await push.send_multipart([msg.encode('ascii')])
            await asyncio.sleep(1)
        if currentmsg == 2:
            print("sending 2")
            await push.send_multipart([str(time.time() - tic).encode('ascii')])
            await asyncio.sleep(1)


asyncio.get_event_loop().run_until_complete(asyncio.wait([
    ping(),
    receiver(),
    sender(),
]))
