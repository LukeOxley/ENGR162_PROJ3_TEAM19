import matplotlib.pyplot as plt
from transforms import *


        

# coord = Translation2d(1 , 1)
# plt.scatter(coord.getX(), coord.getY())
# coord = coord.rotateBy(Rotation2d.fromDegrees(90))
# plt.scatter(coord.getX(), coord.getY())
# plt.scatter(0, 0)
# print(coord)
current = RigidTransform2d(Translation2d(0,0), Rotation2d.fromDegrees(0))

left = 5
right = 5.5
rads = 30.0 * math.pi / 180.0

plt.scatter(current.getTranslation().getX(), current.getTranslation().getY())

print(current)
delta = RigidTransform2d.Delta((left + right)/2, 0, rads)
current = current.transformBy(RigidTransform2d.fromVelocity(delta))
print(current)
print(current.getTranslation().norm())
plt.scatter(current.getTranslation().getX(), current.getTranslation().getY())


current = RigidTransform2d(Translation2d(0,0), Rotation2d.fromDegrees(0))
left = 5
right = 5
rads = 60.0 * math.pi / 180.0
delta = RigidTransform2d.Delta((left + right)/2, 0, rads)
current = current.transformBy(RigidTransform2d.fromVelocity(delta))
print(current)
print(current.getTranslation().norm())
plt.scatter(current.getTranslation().getX(), current.getTranslation().getY())



current = RigidTransform2d(Translation2d(0,0), Rotation2d.fromDegrees(0))
left = 5
right = 5
rads = 90.0 * math.pi / 180.0
delta = RigidTransform2d.Delta((left + right)/2, 0, rads)
current = current.transformBy(RigidTransform2d.fromVelocity(delta))
print(current)
print(current.getTranslation().norm())
plt.scatter(current.getTranslation().getX(), current.getTranslation().getY())

current = RigidTransform2d(Translation2d(0,0), Rotation2d.fromDegrees(0))
left = 5
right = 5
rads = 0.0 * math.pi / 180.0
delta = RigidTransform2d.Delta((left + right)/2, 0, rads)
current = current.transformBy(RigidTransform2d.fromVelocity(delta))
print(current)
print(current.getTranslation().norm())
plt.scatter(current.getTranslation().getX(), current.getTranslation().getY())

#plt.show()

curr = Rotation2d.fromDegrees(45)
start = Rotation2d.fromDegrees(-45)

offset = start.inverse().rotateBy(curr)
print(offset)

curr = RigidTransform2d(Translation2d(-480, 120), Rotation2d(1, 1, True))
delta = RigidTransform2d(Translation2d(-20, 0), Rotation2d(1,0, False))
print(curr)
print(delta)
result = curr.transformBy(delta)
print(result)

start = RigidTransform2d(Translation2d(120, 480), Rotation2d(0, 1, False)).getTranslation()
max_loc = Translation2d(50, 0).rotateBy(Rotation2d(-1, 0, False))

print(max_loc)
end = start.translateBy(max_loc)
print(end)