import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('/home/csl/rdog/test_ws/src/action_log_mujoco.csv', delimiter=',')
t = np.arange(data.shape[0]) * 1/50 

plt.figure(figsize=(14, 16))

plt.subplot(4, 1, 1)
for i in range(3):
    plt.plot(t, data[:, i], label=f'Joint {i}')
plt.title('Joint Positions 1-3')
plt.xlabel('Time [s]')
plt.legend()

plt.subplot(4, 1, 2)
for i in range(3, 6):
    plt.plot(t, data[:, i], label=f'Joint {i}')
plt.title('Joint Positions 4-6')
plt.xlabel('Time [s]')
plt.legend()

plt.subplot(4, 1, 3)
for i in range(6, 9):
    plt.plot(t, data[:, i], label=f'Joint {i}')
plt.title('Joint Positions 7-9')
plt.xlabel('Time [s]')
plt.legend()

plt.subplot(4, 1, 4)
for i in range(9, 12):
    plt.plot(t, data[:, i], label=f'Joint {i}')
plt.title('Joint Positions 10-12')
plt.xlabel('Time [s]')
plt.legend()

plt.ylabel('Action value')
plt.legend(ncol=2)
plt.tight_layout()
plt.show()