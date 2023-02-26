import requests
import random
import json
url = "https://s8176.blr1.piesocket.com/api/publish"
i=0
payload = json.dumps({
    "key": "h60WX0ZPGChIf1cAfJh86GPFyk2DpTMyavihtbaM", #Demo key, get yours at https://piesocket.com
    "secret": "pKpFDNVpt7mP3eBexLyplIAr16hfQp66", #Demo secret, get yours at https://piesocket.com
    "roomId": "1",
    "message": { "text": "Hello From Node! "+str((i+1))+" .{'speed': "+str(random.uniform(10,40))+", \
            'distance':"+str(random.uniform(20,50))+" \
            'accelero reading':"+str(random.uniform(30,60))+"}" }
});
print(payload)
headers = {
  'Content-Type': 'application/json'
}
total =0
for i in range(300):
    response = requests.request("POST", url, headers=headers, data = payload)
    total+=response.elapsed.total_seconds()
    i+=1
    
print(response.text.encode('utf8'))
print(total)