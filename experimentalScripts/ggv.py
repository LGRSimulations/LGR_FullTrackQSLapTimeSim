import numpy as np
import matplotlib.pyplot as plt

mu = 1.2
Fz = 1000  # Normal load (N), example value

theta = np.linspace(0, 2 * np.pi, 200)
Fx = mu * Fz * np.sin(theta)  # Longitudinal force (braking/traction)
Fy = mu * Fz * np.cos(theta)  # Lateral force (cornering)

plt.figure(figsize=(7,7))
plt.plot(Fy, Fx, label='Friction Circle')

# Example: plot slip angle and slip ratio points (placeholder logic)
slip_angles = np.linspace(-15, 15, 10)  # degrees
slip_ratios = np.linspace(-0.2, 0.2, 10)
for sa in slip_angles:
    for sr in slip_ratios:
        # Placeholder: map slip angle/ratio to force as a fraction of max
        Fy_value = mu * Fz * np.cos(np.radians(sa)) * (1 - abs(sr)/0.2)
        Fx_value = mu * Fz * np.sin(np.radians(sa)) * (1 - abs(sr)/0.2)
        plt.scatter(Fy_value, Fx_value, c='r', s=30, alpha=0.6, label='Slip Points' if (sa==slip_angles[0] and sr==slip_ratios[0]) else "")

plt.axhline(0, color='gray', linestyle='--')
plt.axvline(0, color='gray', linestyle='--')
plt.xlabel('Lateral Force (N)')
plt.ylabel('Longitudinal Force (N)')
plt.title('Friction Circle with Lateral/Longitudinal Forces')
plt.grid()
plt.axis('equal')
plt.legend()
plt.show()