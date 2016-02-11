import numpy as np
import matplotlib.pyplot as plt

with open('./fly.txt', 'r') as f:
    lines = f.readlines()
jumper = 0
yaw, pitch, roll = [], [], []
x, y, z = [], [], []
vx,vy,vz = [],[],[]
sx,sy,sz = [],[],[]
svx,svy,svz = [],[],[]
for line in lines[:]:
    line = line.strip('\n')
    line_list = line.split(' ')
    jumper = jumper + 1
    if jumper == 3:
        yaw.append(float(line_list[-5]))
        pitch.append(float(line_list[-3]))
        roll.append(float(line_list[-1]))
        z.append(float(line_list[5]))
        vz.append(float(line_list[7]))
        sz.append(float(line_list[1]))
        svz.append(float(line_list[3]))
    if jumper == 2:
        y.append(float(line_list[5]))
        vy.append(float(line_list[7]))
        sy.append(float(line_list[1]))
        svy.append(float(line_list[3]))
    if jumper == 1:
        x.append(float(line_list[5]))
        vx.append(float(line_list[7]))
        sx.append(float(line_list[1]))
        svx.append(float(line_list[3]))
    if jumper == 3:
        jumper = 0

print np.shape(x),np.shape(y),np.shape(z)
print np.shape(roll),np.shape(yaw),np.shape(pitch)
print 'Data len is %f s'% (len(yaw)/100.0)

print 'The std of x:',np.std(np.array(x[:]),ddof = 1)
print 'The std of y:',np.std(np.array(y[:]),ddof = 1)
# print 'The std of z:',np.std(np.array(z),ddof = 1)

# dt = 0.01
# t = np.arange(0, len(yaw)/100.0, dt)
t = np.arange(0, np.shape(x)[0])

# fig, axs = plt.subplots(5, 1)


# # # axs[0].plot(t, yaw[1000:1800])
# # axs[0].plot(t, yaw[:])
# # # axs[0].set_ylim(-0.15, -0.05)
# # # axs[0].set_xlim(10,25)
# # axs[0].set_xlabel('Time (s)')
# # axs[0].set_ylabel('Yaw (rad)')
# # axs[0].grid(True)

# axs[0].plot(t, roll[800:4500])
# # axs[0].plot(t, roll[:])
# # axs[1].set_xlim(10, 25)
# # axs[1].set_ylim(-0.5, 0.5)
# axs[0].set_xlabel('Time (sec)')
# axs[0].set_ylabel('Roll (rad)')
# axs[0].grid(True)


# axs[1].plot(t, pitch[800:4500])
# # axs[1].plot(t, pitch[:])
# # axs[2].set_ylim(-0.5, 0.5)
# axs[1].set_xlabel('Time (sec)')
# axs[1].set_ylabel('Pitch (rad)')
# axs[1].grid(True)

# axs[2].plot(t, x[800:4500])
# # axs[2].plot(t, x[:])
# # axs[3].set_ylim(-0.5, 0.5)
# axs[2].set_xlabel('Time (sec)')
# axs[2].set_ylabel('X (m)')
# axs[2].grid(True)

# axs[3].plot(t, y[800:4500])
# # axs[3].plot(t, y[:])
# # axs[4].set_ylim(-0.5, 0.5)
# axs[3].set_xlabel('Time (sec)')
# axs[3].set_ylabel('Y (rad)')
# axs[3].grid(True)

# axs[4].plot(t, z[800:4500])
# # axs[4].plot(t, z[:])
# # axs[5].set_ylim(-0.5, 0.5)
# axs[4].set_xlabel('Time (sec)')
# axs[4].set_ylabel('Z (rad)')
# axs[4].grid(True)




plt.figure(figsize=(30,30.5))
plt.subplot(811)
plt.plot(t, roll[:])
plt.ylabel("Roll (rad)") 

plt.subplot(812)
plt.plot(t, pitch[:])
plt.ylabel("Pitch (rad)") 

plt.subplot(813)
plt.plot(t, x[:])
plt.plot(t, sx[:])
plt.ylabel("X Position (m)") 

plt.subplot(814)
plt.plot(t, y[:])
plt.plot(t, sy[:])
plt.ylabel("Y Position (m)") 

plt.subplot(815)
plt.plot(t, z[:])
plt.plot(t, sz[:])
plt.ylabel("Z Position (m)") 

plt.subplot(816)
plt.plot(t, vx[:])
plt.plot(t, svx[:])
plt.ylabel("Z Position (m)") 

plt.subplot(817)
plt.plot(t, vy[:])
plt.plot(t, svy[:])
plt.ylabel("Z Position (m)") 

plt.subplot(818)
plt.plot(t, vz[:])
plt.plot(t, svz[:])
plt.ylabel("Z Position (m)") 


plt.xlabel("Time (Sec)") 


# fig.tight_layout()
plt.savefig("figure.pdf")
# plt.show()