# Satellite Attitude Dynamics Simulation

MATLAB simulation of satellite orientation and rotational stability in Low Earth Orbit using Euler angle dynamics and gravitational effects.

## What It Models
- Satellite orientation changes (roll, pitch, yaw)
- Rotational stability under different conditions
- Gravitational gradient torques
- Momentum wheel desaturation operations
- Angular momentum conservation

---

## Scenarios

### 1. Stable Satellite
**Conditions:** Zero rotation, no external torques  
**Result:** Perfect stability - all angles remain at 0 radians  
**Application:** Baseline for attitude control system performance

---

### 2. Tumbling Satellite
**Conditions:** Initial rotation (0.1 rad) with angular velocities (p=0.05, q=0.03, r=0.02 rad/s)  
**Result:** Constant spin rates, linear angle increase  
**Application:** Loss of attitude control - demonstrates momentum conservation during wheel failures

---

### 3. Gravity Gradient Stabilization
**Conditions:** Small disturbance, gravity torques enabled  
**Result:** Oscillatory "rocking" motion around equilibrium  
**Application:** Passive stabilization - predicts pointing accuracy without active control

Gravity Gradient - Euler Angles: <img width="1389" height="900" alt="Euler_Fig1_gravgrad" src="https://github.com/user-attachments/assets/4e2f3f9e-5cd7-472e-b078-a366dba6ca3e" />

Gravity Gradient - Angular Rates:<img width="1379" height="900" alt="Euler_Fig2_gravgrad" src="https://github.com/user-attachments/assets/85fca631-c882-4bf8-99a1-5262293247c4" />

---

### 4. Reaction Wheel Desaturation
**Conditions:** High angular momentum (p=0.1, q=0.08, r=0.05 rad/s), damping torques  
**Result:** Exponential decay of rotation rates over 30 seconds  

Reaction Wheel - Euler Angles:<img width="1379" height="900" alt="Euler_Fig_RXNWheel" src="https://github.com/user-attachments/assets/6d4d7ce5-dbc5-4ab6-ac2b-a0c6a41894c1" />
Reaction Wheel - Angular Rates:<img width="1393" height="900" alt="Angular_Fig_RXNWheel" src="https://github.com/user-attachments/assets/05206f3f-290d-425c-8fcd-f0c0275f9079" />



Real-World Application: This directly connects to Substack indicator #1 - Momentum Wheel Saturation Creep. When momentum wheels become saturated from accumulated disturbances, operators must dump angular momentum using thrusters. This simulation shows the desaturation maneuver profile.
Business Value: Predicting how fast momentum builds up (and how often desaturation is needed) directly impacts:

Propellant budget (each desaturation burns fuel)
Mission lifetime (run out of propellant = mission over)
Operational planning (when to schedule maneuvers)


Technical Details
Physics Engine: Euler's equations of motion for rigid body rotation
Governing Equations:
phi_dot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
theta_dot = q*cos(phi) - r*sin(phi)
psi_dot = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta)
Numerical Integration: MATLAB's ode45 (Runge-Kutta 4th/5th order adaptive method)
Satellite Parameters:

Mass: 1000 kg
Moments of inertia: I_xx=1000, I_yy=1500, I_zz=800 kg·m²
Orbit: 500 km altitude circular LEO

---

## Why This Matters
Attitude control failures cause ~15% of satellite mission losses. This simulation:
- Predicts momentum wheel saturation rates
- Designs control budgets (propellant allocation)
- Validates stability margins
- Plans desaturation maneuver sequences

---

## Usage
```matlab
% Run any scenario
main_stable
main_tumbling
main_gravity_gradient
main_reaction_wheel
```
**Requirements:** MATLAB (base installation only)
