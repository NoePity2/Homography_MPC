import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos desde el archivo
file_path = 'data_py.txt'

# Leer el archivo como un DataFrame
column_names = ['time', 'h11_p', 'h11_c', 'h12_p', 'h12_c', 'h13_p', 'h13_c', 'h31_p', 'h31_c', 'h32_p', 'h32_c', 'h33_p', 'h33_c', 'v', 'w']
data = pd.read_csv(file_path, delim_whitespace=True, names=column_names)
print(data)

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(1, 1, 1)
colors = ['red','green','blue','magenta','cyan','yellow']
index  = ['h11','h12','h13','h31','h32','h33']
cols   = [1,3,5,7,9,11]
for i,col in enumerate(cols):
   ax.plot(data.iloc[:, col], color=colors[i], label= r'%s predicted'%index[i],linestyle='--')
   ax.plot(data.iloc[:,col+1], color=colors[i], label= r'%s observed'%index[i],linestyle='-')

ax.set_xlabel('Time')
ax.set_ylabel('Homography elements')
ax.legend(loc='best', ncol=3)
ax.grid(True)

print(data.iloc[:, 13])
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(1, 1, 1)
ax.plot(data.iloc[:, 13],label=r"$v \; [\frac{m}{s}$]")
ax.plot(data.iloc[:, 14],label=r" $\omega [\frac{rad}{s}$]")
ax.grid()
ax.legend()
ax.set_xlabel(r"time [$s$]", fontsize=14)
ax.set_ylabel(r"Controls", fontsize=14)

plt.show()
