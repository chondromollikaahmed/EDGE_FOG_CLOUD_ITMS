import websocket
import time 
import random



st = time.time()

ws =websocket.WebSocket()
ws.connect("wss://s8176.blr1.piesocket.com/v3/1?api_key=h60WX0ZPGChIf1cAfJh86GPFyk2DpTMyavihtbaM&notify_self=1")
# ws.connect("wss://s8176.blr1.piesocket.com/v3/1?api_key=h60WX0ZPGChIf1cAfJh86GPFyk2DpTMyavihtbaM")
# print("connected to websocket server")


for i in range(300):
    ws.send(str(i)+" .{'speed': "+str(random.uniform(10,40))+", \
            'distance':"+str(random.uniform(20,50))+" \
            'accelero reading':"+str(random.uniform(30,60))+"}")
    ws.recv()
et = time.time()

elapsed_time = et - st
print('Execution time:', elapsed_time, 'seconds')


ws.close ()