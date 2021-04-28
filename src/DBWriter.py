import sqlite3


class DBWriter:
    
    def __init__(self, filepath):
        self.conn = sqlite3.connect(filepath)
        self.c = self.conn.cursor()
        self.entry = 1
        self.c.execute('DELETE FROM terminal;')
        self.c.execute('UPDATE data SET actuatorL=?, sensorL=?, actuatorC=?, sensorR=?, actuatorR=? WHERE row=1;', (0, 0, 0, 0, 0))
        self.conn.commit()
        
    def writeData(self, data_arr):
        actL, senL, actC, senR, actR = data_arr
        self.c.execute('UPDATE data SET actuatorL=?, sensorL=?, actuatorC=?, sensorR=?, actuatorR=? WHERE row=1;', (actL, senL, actC, senR, actR))
        self.conn.commit()
    
    def writeTerm(self, text):
        self.c.execute('INSERT INTO terminal VALUES (?, ?);', (self.entry, text))
        self.conn.commit()
        self.entry += 1
        
    def readUserIn(self):
	self.c.execute('SELECT * FROM user_commands;')
	for i in c:
	    _, type, amount = i
	self.c.execute('UPDATE user_commands SET type=?, amount=?', ('none', 0))
	self.conn.commit()
	return type, float(amount)