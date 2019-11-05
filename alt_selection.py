import pandas as pd
import numpy as np

df = pd.read_csv('USA-road-d.E.co', sep=" ", header=None, comment='c')
df.columns = ['v', 'id', 'x', 'y']
df.drop(0, axis=0, inplace=True)
df.drop('v', axis=1, inplace=True)

arr = df.to_numpy().astype(int)
arr[:,0] = arr[:,0] - 1

arr[(arr[:,1] < -78463699) & (arr[:,2] > 43370186)][:,0]
arr[(arr[:,1] < -78993699) & (arr[:,2] < 33576748)][:,0]