import numpy as np

LBS2KG = 0.453592
INCHES2METERS = 0.0254

def inertia_circular_cylinder(mass, radius, height):
    Ixx = mass * (3*radius**2 + height**2) / 12.0
    Iyy = mass * (3*radius**2 + height**2) / 12.0
    Izz = mass * radius**2 / 2.0

    return Ixx, Iyy, Izz
#The center axis of the cylindrical hull is aligned with the xaxis
##################SET PARAMS################################
hull_mass_lb = 3.0
hull_radius_in = 1.8
hull_length_in = 9.05

#model the thruster as a cylinder
thruster_mass_lb =  0.0573202
thruster_radius_in = 0.551181
thruster_height_in = 1.02362

############################################################

hull_mass_kg = LBS2KG * hull_mass_lb
print("Hull Mass (kg):          %E kg" %hull_mass_kg)

hull_radius_m = INCHES2METERS * hull_radius_in
hull_length_m = INCHES2METERS * hull_length_in

print("Hull Radius (m):         %E m" % hull_radius_m)
print("Hull Length (m):         %E m" % hull_length_m)

#calculate the inertia of the hull (assuming uniform density)
hull_Izz, hull_Iyy, hull_Ixx = inertia_circular_cylinder(hull_mass_kg, hull_radius_m, hull_length_m)
hull_inertial_matrix = np.array([hull_Ixx, 0, 0, 0, hull_Iyy, 0, 0, 0, hull_Izz]).reshape((3, 3))
print("\nHull Principal Inertias (kg * m^2):")
print(" Ixx:                    %E kg * m^2" % hull_Ixx)
print(" Iyy:                    %E kg * m^2" % hull_Iyy)
print(" Izz:                    %E kg * m^2" % hull_Izz)

thruster_mass_kg = LBS2KG * thruster_mass_lb
print("\nThruster Mass (kg):      %E kg" % thruster_mass_kg)

#calc thruster parameters
thruster_radius_m = INCHES2METERS * thruster_radius_in
thruster_height_m = INCHES2METERS * thruster_height_in
print("Thruster Radius (m):     %E m" % thruster_radius_m)
print("Thruster height (m):     %E m" % thruster_height_m)

thruster_Ixx, thruster_Iyy, thruster_Izz = inertia_circular_cylinder(thruster_mass_kg, thruster_radius_m, thruster_height_m)

print("\nThruster Principal Inertias (kg * m^2):")
print(" Ixx:                    %E kg * m^2" % thruster_Ixx)
print(" Iyy:                    %E kg * m^2" % thruster_Iyy)
print(" Izz:                    %E kg * m^2" % thruster_Izz)
