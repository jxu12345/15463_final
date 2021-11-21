# class for 3-vector objects
class Vec3():
    def __init__(self, x=0., y=0., z=0.):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return '{{{self.x}, {self.y}, {self.z}}}'.format(self=self)

    def __add__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return Vec3(self.x + other, self.y + other, self.z + other)

    def __sub__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            return Vec3(self.x - other, self.y - other, self.z - other)

    def __mul__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)
        else:
            return Vec3(self.x * other, self.y * other, self.z * other)
    
    def __iadd__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return Vec3(self.x + other, self.y + other, self.z + other)

    def __isub__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            return Vec3(self.x - other, self.y - other, self.z - other)

    def __imul__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)
        else:
            return Vec3(self.x * other, self.y * other, self.z * other)

    def __pow__(self, exp):
        if isinstance(exp, float) or isinstance(exp,int):
            return Vec3(self.x ** exp, self.y ** exp, self.z ** exp)
        else:
            raise Exception('Invalid power exponent type')

    def __rmul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return Vec3(other * self.x, other * self.y, other * self.z)  
        else:
            raise Exception('Invalid right multiplication type')

    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)
    
    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        else:
            raise Exception('Vec3 index out of range')
    
    def __setitem__(self, key, data):
        if key == 0:
            self.x = data
        elif key == 1:
            self.y = data
        elif key == 2:
            self.z = data
        else:
            raise Exception('Vec3 index out of range')

    def __abs__(self):
        return Vec3(abs(self.x), abs(self.y), abs(self.z))

    def __round__(self):
        return Vec3(round(self.x), round(self.y), round(self.z))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __ne__(self, other):
        return self.x != other.x or self.y != other.y or self.z != other.z

    def to_int(self):
        return Vec3(int(self.x), int(self.y), int(self.z))

    def to_float(self):
        return Vec3(float(self.x), float(self.y), float(self.z))
    

def vec3_from_string(str1, str2, str3):
    x = float(str1)
    y = float(str2)
    z = float(str3)
    return Vec3(x, y, z)

def interpolate(lower, upper, time):
    return time * upper + (1-time) * lower

def print_list(vec_list):
    for i in range(len(vec_list)):
        print(str(vec_list[i]))