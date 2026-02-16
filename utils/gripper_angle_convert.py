import numpy as np

# 주어진 값
r = 15.0  # mm
l = 40.0  # mm
x = 26.586  # mm

# θ 구하기
cos_theta = (r**2 + x**2 - l**2) / (2 * r * x)
theta = np.arccos(cos_theta)
theta_deg = np.degrees(theta)

# φ 구하기
cos_phi = (x - r * np.cos(theta)) / l
phi = np.arccos(cos_phi)
phi_deg = np.degrees(phi)

print(f"θ = {theta_deg:.15f}°")
print(f"φ = {phi_deg:.15f}°")