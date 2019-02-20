import pandas as pd

data = pd.read_csv("Holo_data/2018-12-04-10-47-15-obstacles.csv", low_memory=False) 

print(data.head())