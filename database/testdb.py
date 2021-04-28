import sqlite3
import random
import time

conn = sqlite3.connect('platform.db')
cur = conn.cursor()

cur.execute('DELETE FROM terminal;')
conn.commit()

ent = 1

while True:
    
    data = random.randint(0,10)
    cur.execute('UPDATE data SET actuatorL=?, sensorL=?, actuatorC=?, sensorR=?, actuatorR=? WHERE row=1;', (data, data-1, 0.5, data, 0.5))
    text = "this is working " + str(ent)
    cur.execute('INSERT INTO terminal VALUES (?, ?);', (ent, text))
    conn.commit()
    ent += 1
    time.sleep(2)


