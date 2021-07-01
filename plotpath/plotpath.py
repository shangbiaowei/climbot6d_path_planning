from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

data = numpy.loadtxt('path0.txt')
data1 = numpy.loadtxt('path1.txt')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,0],data[:,1],data[:,2],'.-')
plt.hold('on')
plt.grid('on')
ax.plot(data1[:,0],data1[:,1],data1[:,2],'g-')


# # plot sphere
# # center and radius
# center = [500, 500, 500]
# radius = 250
# # data
# u =numpy.linspace(0, 2 *numpy.pi, 100)
# v =numpy.linspace(0,numpy.pi, 100)
# x = radius *numpy.outer(numpy.cos(u),numpy.sin(v)) + center[0]
# y = radius *numpy.outer(numpy.sin(u),numpy.sin(v)) + center[1]
# z = radius *numpy.outer(numpy.ones(numpy.size(u)),numpy.cos(v)) + center[2]
# # # plot
# # ax.plot_surface(x, y, z, rstride=10, cstride=10, color='g')
# # # wire frame
# ax.plot_wireframe(x, y, z, rstride=10, cstride=10, color='r')

# show
plt.axis('equal')
plt.show()

