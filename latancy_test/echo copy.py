import websocket
import time 
import random



st = time.time()

ws =websocket.WebSocket()
ws.connect("wss://s8775.nyc1.piesocket.com/v3/1?api_key=7C9up7kPYDMSlgPGDPwwCvBCuyDdS7zbRjfKRB4e")
# ws.connect("wss://s8176.blr1.piesocket.com/v3/1?api_key=h60WX0ZPGChIf1cAfJh86GPFyk2DpTMyavihtbaM")
# print("connected to websocket server")


ws.send(str(i)+" .{'speed': "+str(random.uniform(10,40))+", \
            'distance':"+str(random.uniform(20,50))+" \
            'accelero reading':"+str(random.uniform(30,60))+"}")
et = time.time()

elapsed_time = et - st
print('Execution time:', elapsed_time, 'seconds')


ws.close ()