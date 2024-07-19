class DataLogger:
    def __init__(self, filePath, columnNames):
        self.names = columnNames
        self.file = open(filePath, 'w')
                
        #write header to file
        self.logData(columnNames)
    
    def __del__(self):
        self.file.close()
    
    def logData(self, columns):
        # print(f"Logging {columns} to {self.file.name}", flush=True)
        self.file.write(f"{','.join([str(d) for d in columns])}\n")
    
    def flush(self):
        self.file.flush()
