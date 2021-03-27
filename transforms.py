import math

class RigidTransform2d:

    class Delta:
        def __init__(self, dx, dy, dtheta):
            self.dx = dx
            self.dy = dy
            self.dtheta = dtheta

    def __init__(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation
    def fromTranslation(translation):
        return RigidTransform2d(translation, Rotation2d.fromRadians(0))
    def fromVelocity(delta):
        sin_theta = math.sin(delta.dtheta)
        cos_theta = math.cos(delta.dtheta)
        if (math.fabs(delta.dtheta) < 1.0E-9):
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta
            c = 0.5 * delta.dtheta
        else:
            s = sin_theta / delta.dtheta
            c = (1.0 - cos_theta) / delta.dtheta
        return RigidTransform2d(Translation2d(delta.dx * s - delta.dy * c,
                                              delta.dx * c + delta.dy * s),
                                Rotation2d(cos_theta, sin_theta, False))
    def getTranslation(self):
        return self.translation
    def setTranslation(self, translation):
        self.translation = translation
    def getRotation(self):
        return self.rotation
    def setRotation(self, rotation):
        self.rotation = rotation
    def transformBy(self, other):
        return RigidTransform2d(self.translation.translateBy(other.translation.rotateBy(self.rotation)),
                                self.rotation.rotateBy(other.rotation))
    def inverse(self):
        rotation_inverted = self.rotation.inverse()
        return RigidTransform2d(self.translation.inverse().rotateBy(rotation_inverted), rotation_inverted)
    
    def interpolate(self, other, x):
        if (x <= 0):
            return RigidTransform2d(self.translation, self.rotation)
        elif (x >= 1):
            return RigidTransform2d(other.translation, other.rotation)
        else:
            return RigidTransform2d(self.translation.interpolate(other.translation, x),
                                    self.rotation.interpolate(other.rotation, x))
    def __str__(self):
        return str(self.translation) + ' ' + str(self.rotation)

class Rotation2d:
    def __init__(self, x, y, normalize):
        self.cos_angle = x
        self.sin_angle = y
        if(normalize):
            self.normalize()
    def fromRadians(radians):
        return Rotation2d(math.cos(radians), math.sin(radians), False)
    def fromDegrees(degrees):
        return Rotation2d.fromRadians(degrees * math.pi / 180)
    def normalize(self):
        mag = math.hypot(self.cos_angle, self.sin_angle)
        if (mag > 1E-9):
            self.sin_angle /= mag
            self.cos_angle /= mag
        else:
            self.sin_angle = 0
            self.cos_angle = 1
    def cos(self):
        return self.cos_angle
    def sin(self):
        return self.sin_angle
    def getRadians(self):
        return math.atan2(self.sin_angle, self.cos_angle)
    def getDegrees(self):
        return self.getRadians() * 180 / math.pi
    def rotateBy(self, other_rotation):
        return Rotation2d(self.cos_angle * other_rotation.cos_angle - self.sin_angle * other_rotation.sin_angle,
                          self.cos_angle * other_rotation.sin_angle + self.sin_angle * other_rotation.cos_angle, True)
    def inverse(self):
        return Rotation2d(self.cos_angle, -self.sin_angle, False)
    def interpolate(self, other_rotation, x):
        if (x <= 0):
            return Rotation2d(self.cos_angle, self.sin_angle, False)
        elif (x >= 1):
            return Rotation2d(other_rotation.cos_angle, other_rotation.sin_angle, False)
        angle_diff = self.inverse().rotateBy(other_rotation).getRadians()
        return self.rotateBy(Rotation2d.fromRadians(angle_diff * x))
    def __str__(self):
        return "Degrees: {:.2f}".format(self.getDegrees()) 

class Translation2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return "X: {:.2f}".format(self.x)+" Y: {:.2f}".format(self.y)
    def norm(self):
        return math.hypot(self.x, self.y)
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def setX(self, x):
        self.x = x
    def setY(self, y):
        self.y = y
    def translateBy(self, other_translation):
        return Translation2d(self.x + other_translation.x, 
                             self.y + other_translation.y)
    def rotateBy(self, rotation):
        return Translation2d(self.x * rotation.cos() - self.y * rotation.sin(),
                             self.x * rotation.sin() + self.y * rotation.cos())
    def inverse(self):
        return Translation2d(-self.x, -self.y)
    def interpolate(self, other_translation, x):
        if (x <= 0):
            return Translation2d(self.x, self.y)
        elif (x >= 1):
            return Translation2d(other_translation.x, other_translation.y)
        else:
            return self.extrapolate(other_translation, x)
    def extrapolate(self, other_translation, x):
        return Translation2d(x * (other_translation.x - self.x) + self.x, 
                             x * (other_translation.y - self.y) + self.y)