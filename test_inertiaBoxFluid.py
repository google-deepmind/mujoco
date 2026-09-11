# test_inertiaBoxFluid.py

def mjd_inertiaBoxFluid(mass, x, y, z):
    Ixx = mass * (y*y + z*z) / 12.0
    Iyy = mass * (x*x + z*z) / 12.0
    Izz = mass * (x*x + y*y) / 12.0
    return Ixx, Iyy, Izz

# Example test
mass = 2.0
x, y, z = 1.0, 2.0, 3.0

Ixx, Iyy, Izz = mjd_inertiaBoxFluid(mass, x, y, z)
print(f"Ixx={Ixx:.4f}, Iyy={Iyy:.4f}, Izz={Izz:.4f}")
