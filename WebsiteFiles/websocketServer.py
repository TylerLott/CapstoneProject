import asyncio
import websockets
import sqlite3
import json

async def myfunc(websocket, path):
    db_conn = sqlite3.connect('/home/pi/Desktop/CapstoneProject/database/platform.db')
    last_entry = 0
    while True:
        c = db_conn.cursor()
        c.execute('SELECT * FROM data;')
        for i in c:
            _, actL, senL, actC, senR, actR = i
        info = json.dumps({'type': 'data', 'actL': actL, 'senL': senL, 'actC': actC, 'senR': senR, 'actR': actR})
        await websocket.send(info)
        
        c.execute('SELECT * FROM terminal WHERE entry_num > ?', (last_entry,))
        for i in c:
            last_entry, terminal_out = i
            info = json.dumps({'type':'terminal', 'text':terminal_out})
            await websocket.send(info)
        
	async for message in websocket:
	    data = json.loads(message)
	    self.c.execute('UPDATE user_commands SET type=?, amount=? WHERE row=1;', (data["type"], float(data["amount"])))

        await asyncio.sleep(.1)

start_server = websockets.serve(myfunc, "localhost", 4000)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()


