# CarND-Model Predictive Controller
Term 2 - Project 5 - Self-Driving Car Engineer Nanodegree Program

---

## The Model

* The model predictive controller implements a Dynamic model to predict the path of the vehicle considering the vehicle dynamics 

* The state variables used are x, y, psi, v, cte and epsi

* The actuators used are delta for steering angle and acceleration for throttle.

* The lower and upper bounds are set for variable as well as constraints

* The constraints are based upon the update equations. 

```c++
fg[1 + x_start + t] = x1 - (x0 + v0 *CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 *CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta0 * dt); // change delta sign
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 *CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt); // change delta sign
```

------

## Timestep Length and Elapsed Duration (N & dt)

* The No of timesteps N is chosen as 10 based on trail and error and the timestep length dt is taken as 0.1s or 100ms
* The total duration of prediction is therefore `T = N * dt = 1s`

------

## Polynomial Fitting and MPC Preprocessing

* A 3rd degree polynomial `ax^3 +bx^2+cx+d` is used for polynomial fitting

* In the pre-processing, the waypoints in map coordinates are converted to vehicle coordinates so that the calculations are made easy and referenced from the vehicle using

  ```c++
  double dX = ptsx[i] - px;
  double dY = ptsy[i] - py;
  double minus_psi = 0.0 - psi;
  ptsx[i] = dX *cos( minus_psi ) - dY *sin( minus_psi );
  ptsy[i] = dX *sin( minus_psi ) + dY *cos( minus_psi );
  ```

## Model Predictive Control with Latency

* The Cost function play a key role in optimizing the route of the vehicle. The following equations were

  used to formulate the cost function

  ```c++
  fg[0] += 3500* CppAD::pow(vars[cte_start + t]-ref_cte, 2);
  fg[0] += 3500* CppAD::pow(vars[epsi_start + t]-ref_epsi, 2);
  fg[0] += 1* CppAD::pow(vars[v_start + t] - ref_v, 2);
  fg[0] += 5* CppAD::pow(vars[delta_start + t], 2);
  fg[0] += 5* CppAD::pow(vars[a_start + t], 2);
  fg[0] += 700* CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2); 
  fg[0] += 200* CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += 10* CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
  ```

* To account for Latency, the actuators are activated after the one timestep of 100ms which is equal to the latency of the vehicle

  ```c++
  if (t > 1) {   // to account for latency
  AD<double> a0 = vars[a_start + t - 2];
  AD<double> delta0 = vars[delta_start + t - 2];
  }
  ```


