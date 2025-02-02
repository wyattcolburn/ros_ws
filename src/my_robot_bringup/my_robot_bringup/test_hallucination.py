import math

height = 2
width = 1

num_readings = 640

angle_increment = 360 / num_readings

# Compute angles correctly
target_theta_height = math.atan(height / width)  # Angle for height
target_theta_width = math.atan(width / height)   # Angle for width

# Compute the number of scans corresponding to height and width
scans_per_half_height = math.degrees(target_theta_height) / angle_increment
scans_per_half_width = math.degrees(target_theta_width) / angle_increment

# Total number of scans for a rectangular structure
total_scans = 2 * scans_per_half_height + 2 * scans_per_half_width

# Debug prints
print(f"Scans per half height: {scans_per_half_height:.2f}")
print(f"Scans per half width: {scans_per_half_width:.2f}")
print(f"Total scans: {total_scans:.2f}")

corner_list = [0]
for i in range(8): 
    if i in (1,2,5,6):
        corner_list.append(corner_list[-1]+scans_per_half_width)
    else:
        corner_list.append(corner_list[-1] + scans_per_half_height)
    print(corner_list)
 
