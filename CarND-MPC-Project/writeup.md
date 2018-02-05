
**MPC (Model predictive control)**

The goals / steps of this project are the following:
* Design a MPC algorithm that guides a vehicle to drive along the road
* Reflect on the details of MPC, and describe a latency solution

---

### Reflection

### 1. describes their model in detail. This includes the state, actuators and update equations.

the state includes {x, y, psi, v, cte, epsi}
actuators include {delta [steering angle], a [throttle] }
update equations following kinematic model
    x1 = x0 + v0 * cos(psi0) * dt
    y1 = y0 + v0 * sin(psi0) * dt
    psi1 = psi0 + v0 / Lf * delta0 * dt
    v1 = v0 + a * dt
    cte1 = f(x0) - y0 + v0 * sin(psi0) * dt
    epsi1 = epsi_desi0 - epsi0 + v0 / Lf * delta0 * dt 
the subscript 0,1 stands for timestamp t0 and t1, respectively. 
f(x0) is the expected y value given x0, f is the polynomial formula. 
epsi_desi0 is the expected psi angle, which is computed as atan(f'(x0))
Lf is the length from front to CoG 

### 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried. 

Following the Quize in MPC, similarly I set N = 25, dt = 0.05 second, which works well. 

### 3. A polynomial is fitted to waypoints.
I use 3rd order polynomial to fit the waypoints, whichs return the coefficients of the fitted curve. The details are shown in the code main.cpp (Line 122)

### 4. implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency
Use the previous acceleration and steer to predict the state (x, y, psi, v) of the vehicle after about 100 millisecond latency. The latency is also predicted using the latency between the previous msg and the current msg. 
Then using the predicted state as input to the MPC algorithm to compute acceleration and steer values.
Details are shown between Lines 152 - 165 in the main.cpp
