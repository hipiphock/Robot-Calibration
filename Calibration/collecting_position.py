import ctypes


class Point3d(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double)]


class DataRecord(object):
    def __int__(self):
        self.Datafp = None

    def OpenDataFile(self, mod, filename):
        if mod == 'w':
            self.Datafp = open(filename, "wb")

        elif mod == 'r':
            self.Datafp = open(filename, "rb")

        elif mod == 'a':
            self.Datafp = open(filename, "ab")

        else:
            print("Invalid File Open mode!")
            print("-w : write mode")
            print("-r : Read mode")

    def DataInsert(self, robotpose, Position_by_cam):
        if self.Datafp == None:
            print("Use OpenDataFile first!!")

        elif Position_by_cam[2] == -float('Inf') or Position_by_cam[2] == -float('NaN'):
            print("cam Information contain -inf or NaN")

        else:
            temp_robotpose = Point3d(robotpose[0], robotpose[1], robotpose[2])
            temp_camPosition = Point3d(Position_by_cam[0], Position_by_cam[1], Position_by_cam[2])
            a = self.Datafp.write(temp_robotpose)
            print("DataInsert...temp_robotpose_{}".format(a))
            self.Datafp.write(temp_camPosition)
            print("DataInsert...temp_camPosition")

    def CloseData(self):
        self.Datafp.close()
