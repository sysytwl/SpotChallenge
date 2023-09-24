import math

class Kinematics:
    def __init__(self, L1, L2):
        self._L1 = L1
        self._L2 = L2
        self.Position_ = {
            "RightlyJoint0": 0.0,
            "RightlyJoint1": 0.0,
            "RightlyJoint2": 0.0,
            "RightlyJoint3": 0.0,
            "LeftyJoint0": 0.0,
            "LeftyJoint1": 0.0,
            "LeftyJoint2": 0.0,
            "LeftyJoint3": 0.0,
            "Lefty": False,
            "Rightly": False
        }
        self.ForwardKinematics_ = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
        }
        self.transformed_ = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
        }
        self._Lsum = self._L1 + self._L2
        self._Ldiff = abs(self._L1 - self._L2)
        self._Alpha = 0.0
        self._Beta = 0.0
        self._Gamma = 0.0
        self._xy = 0.0

    def InverseKinematics(self, targetX, targetY, targetZ):
        x_y = math.sqrt(targetX * targetX + targetY * targetY)
        xy_z = math.sqrt(x_y * x_y + targetZ * targetZ)
        print(xy_z)

        self.Position_["Lefty"] = False
        self.Position_["Rightly"] = False

        if self._Ldiff <= xy_z <= self._Lsum:
            self._Alpha = math.acos((xy_z * xy_z + self._L1 * self._L1 - self._L2 * self._L2) / (2 * self._L1 * xy_z)) * 180.0 / math.pi
            print(self._Alpha)
            self._Beta = math.acos((self._L1 * self._L1 + self._L2 * self._L2 - xy_z * xy_z) / (2 * self._L1 * self._L2)) * 180.0 / math.pi
            print(self._Beta)
            self._Gamma = math.atan2(targetZ, x_y) * 180.0 / math.pi
            print(self._Gamma)
            print(math.atan(targetZ/x_y)* 180.0 / math.pi)
            self.Position_["LeftyJoint0"] = self.Position_["RightlyJoint0"] = math.atan2(targetY, targetX) * 180.0 / math.pi
            self.Position_["RightlyJoint1"] = self._Gamma - self._Alpha
            self.Position_["RightlyJoint2"] = 180.0 - self._Beta
            self.Position_["LeftyJoint1"] = self._Gamma + self._Alpha
            self.Position_["LeftyJoint2"] = self._Beta - 180.0

            if (0 <= self.Position_["LeftyJoint0"] <= 350 and 0 <= self.Position_["LeftyJoint1"] <= 170 and 0 <= self.Position_["LeftyJoint2"] <= 180):
                self.Position_["Lefty"] = True
            if (0 <= self.Position_["RightlyJoint0"] <= 350 and 0 <= self.Position_["RightlyJoint1"] <= 170 and 0 <= self.Position_["RightlyJoint2"] <= 180):
                self.Position_["Rightly"] = True

    def ForwardKinematics(self, Joint0, Joint1, Joint2):
        print(self._L1 * math.sin(Joint1 / 180.0 * math.pi))
        self.ForwardKinematics_["Z"] = self._L1 * math.sin(Joint1 / 180.0 * math.pi) + self._L2 * math.sin(Joint1 / 180.0 * math.pi + Joint2 / 180.0 * math.pi)
        self._xy = self._L1 * math.cos(Joint1 / 180.0 * math.pi) + self._L2 * math.cos(Joint1 / 180.0 * math.pi + Joint2 / 180.0 * math.pi)
        self.ForwardKinematics_["X"] = self._xy * math.cos(Joint0 / 180.0 * math.pi)
        self.ForwardKinematics_["Y"] = self._xy * math.sin(Joint0 / 180.0 * math.pi)

    def CoordinateTrans(self, originX, originY, originZ, X, Y, Z):
        self.transformed_["X"] = X - originX
        self.transformed_["Y"] = Y - originY
        self.transformed_["Z"] = Z - originZ

# Example usage:
kinematics = Kinematics(274,130)
kinematics.InverseKinematics(101, 0, 285)
print(kinematics.Position_)
kinematics.ForwardKinematics(0, 45, 90)
print(kinematics.ForwardKinematics_)
# kinematics.CoordinateTrans(originX_value, originY_value, originZ_value, X_value, Y_value, Z_value)
