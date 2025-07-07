import numpy as np
import matplotlib.pyplot as plt

# Parameters
gravity = 9.81 
dry_mass = 20000  # Mass of the rocket in kg
duration = 480
dt = 0.1  # Time step in seconds
burn_time = 141
rho = 1.09  # Air density at 4000 ft in kg/m^3
Cd = 0.5  # Drag coefficient
A = 1.0  # Cross-sectional area in m^2
landing_thrust_on = False
fuel_mass_initial = 55000
exhaust_velocity = 4400  # m/s
landing_fuel_reserve = 5000  # Increased reserve for landing
throttle_ratio = 0.75  # Throttle ascent to 75%
max_decel = 5 * gravity
buffer = 5500  # Increased buffer for earlier landing burn

# Initial conditions
position = np.array([0.0, 0.0])
velocity = np.array([0.0, 0.0])
fuel_burn_rate = fuel_mass_initial / burn_time
fuel_mass = fuel_mass_initial
mass = dry_mass + fuel_mass
max_thrust = fuel_burn_rate * exhaust_velocity

# Logs for plotting
time_log = []
x_log = []
y_log = []
vx_log = []
vy_log = []
acc_log = []
mass_log = []

# Simulation loop
time = 0
while time < duration:

    # Thrust angle control
    if time < 5:
        thrust_angle_deg = 90
    elif time < 10:
        thrust_angle_deg = 75
    elif time < 15:
        thrust_angle_deg = 60
    else:
        thrust_angle_deg = 45

    # Trigger landing burn
    required_stop = (velocity[1]**2) / (2 * max_decel)
    if not landing_thrust_on and velocity[1] < -5 and position[1] <= required_stop + buffer: 
        print(f"[DEBUG] Triggering landing burn at altitude {position[1]:.2f} m, velocity {velocity[1]:.2f} m/s")
        landing_thrust_on = True
        thrust_angle_deg = 90

    # Calculate thrust
    if landing_thrust_on and fuel_mass > 0:
        if position[1] > 0:
            target_velocity = -1
            error = velocity[1] - target_velocity
            Kp = 1.21
            desired_decel = min(Kp * abs(error), max_decel)
        else:
            desired_decel = 0

        current_thrust = min(desired_decel * mass, max_thrust)
        required_fuel = current_thrust / exhaust_velocity
        fuel_used = min(required_fuel * dt, fuel_mass)
        fuel_mass = max(0, fuel_mass - fuel_used)
        mass = dry_mass + fuel_mass
        if fuel_mass <= 0:
            current_thrust = 0

    elif time < burn_time and fuel_mass > landing_fuel_reserve:
        current_thrust = throttle_ratio * fuel_burn_rate * exhaust_velocity
        required_fuel = current_thrust / exhaust_velocity
        fuel_used = min(required_fuel * dt, fuel_mass)
        fuel_mass = max(0, fuel_mass - fuel_used)
        mass = dry_mass + fuel_mass
    else:
        current_thrust = 0

    # Forces
    gravity_force = np.array([0, -mass * gravity])
    thrust_angle_rad = np.radians(thrust_angle_deg)
    thrust_force = np.array([current_thrust * np.cos(thrust_angle_rad),
                             current_thrust * np.sin(thrust_angle_rad)])
    
    speed = np.linalg.norm(velocity)
    drag_force = -0.5 * rho * speed**2 * Cd * A * (velocity / speed) if speed > 0 else np.array([0.0, 0.0])
    net_force = thrust_force + gravity_force + drag_force
    acceleration = net_force / mass

    # Integrate
    velocity += acceleration * dt
    position += velocity * dt

    # Ground check
    if position[1] <= 0 and abs(velocity[1]) < 1.0 and time > 0:
        print("Safe landing achieved!")
        break
    elif position[1] <= 0 and time > 0:
        print(f"Fuel left: {fuel_mass:.2f} kg")
        print(f"Altitude: {position[1]:.2f} m")
        print(f"Velocity: {velocity[1]:.2f} m/s")
        print("Crash landing! Final vertical speed:", velocity[1])
        break

    # Logs
    time_log.append(time)
    x_log.append(position[0])
    y_log.append(position[1])
    vx_log.append(velocity[0])
    vy_log.append(velocity[1])
    acc_log.append(np.linalg.norm(acceleration))
    mass_log.append(mass)

    time += dt

# Plotting
plt.figure(figsize=(12, 10))

plt.subplot(3, 1, 1)
plt.plot(time_log, y_log, color='blue')
plt.title('Rocket Flight Profile with Drag')
plt.ylabel('Altitude (m)')
plt.grid(True)

plt.subplot(3, 1, 2)
velocity_magnitude = [np.sqrt(vx**2 + vy**2) for vx, vy in zip(vx_log, vy_log)]
plt.plot(time_log, velocity_magnitude, color='orange')
plt.ylabel('Velocity (m/s)')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(time_log, acc_log, color='green')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.grid(True)

plt.figure()
plt.plot(time_log, mass_log)
plt.title('Rocket Mass Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Mass (kg)')
plt.grid(True)

plt.tight_layout()
plt.show()
