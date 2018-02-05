
**PID Controller**

The goals / steps of this project are the following:
* Design a PID controller that guides a vehicle to drive along the road
* Designa a process to tune the coefficients of P,I,D terms
* Reflect on the effects of P,I,D terms, and describe the process to tune these coefficients. 

[//]: # (Image References)

[image1]: ./screen_snap/twiddle.jpg "twiddle"

---

### Reflection

### 1. describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

P-Propotional term, correcting the cte error by its current quantity or magnitude. However, when cte approches to zero, the speed of the system maynot approach zeor, intead could remain a certain value, causing overshooting. If the weight of P is too large, overshooting frequently occurs, making the vehicle fluctuate. Therefore, I initially set weight of P (Kp) as 1.0, that seems to be a proper intial value for later parameter tuning.

I-Integral term, correcting the cte error by acclumation of cte. This couteracts the acclumated residual error after correcting by Propotional term. Since the accumulated value could be very large, the weight of I (Ki) must be small. In this case, initially Ki is 0.001. 

D-Derivative term, correcting the cte error by the change of cte. This couteracts the fluctuation effect by exerting a control force propotional to the rate of cte error change. This forces the speed of the system approach to zero as cte reduces, therefore combat overshooting. Initally, the weight of the I (Kd) is set 1.0. 

### 2. Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

I tune the P, I, D coefficients using twiddle. The code contains in twiddle.cpp and tune_param.cpp, which are designed to find out the Kp, Ki, and Kd. One thing needs to mention here, since when the vehicle is stuck at location nearby the track, making the accumulated cte small, results in a bad solution. So I added another speed error term, shown in the line 42 of the code twiddle.cpp. Specically, it adds penalty if the vehicle's speed is less than a reference speed (10mph). By penalizing small speed, twiddle can find out better parameters, which makes the vehicle drive along the road. 

I run this code for several times by gradually improving the number of steps (N) used to compute cte, making it a corase-to-fine process. 

At first run, initially, Kp = 1.0, Ki = 0.01, Kd = 1.0, dp = 1.0, 0.005, 1.0, N = 100, and the final output Kp = 1.0, Ki = 0.01, Kd = 1.8. This parameter enables the vehicle travel to the curve, but dirft away when it enters into the curve. 

By obversing that the Ki is still too large to make the vehicle steers heavily, I mannually decrease it as Ki = 0.001.

In the second run, initially Kp = 1.0, Ki = 0.001, Kd = 1.8; dp = 1.0, 0.0005, 1.0, N = 450, and the final output Kp = 0.9, Ki = 0.00205, Kd = 4.448;

In the third run, initially Kp = 0.9, Ki = 0.00205, Kd = 4.448; dp = 0.4, 0.001, 2., N = 650, and the final output Kp = 0.9 , Ki = 0.00105, Kd = 4.448;

However, with these parameters, the vehicle experience heavy swing in the end part of the road. So I decrease Kp = 0.4, and it works finally.

The screen snap of the result of third run is shown below: 


![alt_text][image1]
