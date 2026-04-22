


# Data driven control with error compensation in continuum robot

This repository contains the simulation code and schemes for the paper submission related to **Data driven control with error compensation is modified by model free adaptive control** applied to **Continuum robot** systems.

The repository is created for the purpose of anonymous peer review.

---

##  Requirements

- This repository is supplementary material fro paper **A Backbone-Free Continuum Robot Using Data Driven Control Scheme with Error Compensation**
- MATLAB R2025a or later

---

## Numerical model

The numerical calculation in this code is based on a nonlinear discrete-time MIMO system with two inputs and two outputs:

- input vector

  $$
  u(k)=\begin{bmatrix}u_1(k)\\u_2(k)\end{bmatrix}\in\mathbb{R}^2
  $$

- output vector

  $$
  y(k)=\begin{bmatrix}y_1(k)\\y_2(k)\end{bmatrix}\in\mathbb{R}^2
  $$

The system update equations are

$$
y_1(k+1)&=
\frac{2.5\,y_1(k)y_1(k-1)}
{1+y_1^2(k)+y_2^2(k-1)+y_1^2(k-2)}
+0.7\sin\!\left(\frac{y_1(k)+y_1(k-1)}{2}\right)
\cos\!\left(\frac{y_1(k)+y_1(k-1)}{2}\right) \\

&+0.09\,u_1(k)u_2(k-1)
+1.2\,u_1(k)
+1.6\,u_1(k-2)
+0.5\,u_2(k)
$$

and

$$
y_2(k+1)=
\frac{5\,y_2(k)y_2(k-1)}
{1+y_2^2(k)+y_1^2(k-1)+y_2^2(k-2)}
+u_2(k)
+1.1\,u_2(k-1)
+1.4\,u_2(k-2)
+0.5\,u_1(k)
$$

This numerical model has the following characteristics:

1. **Nonlinearity**: both output channels contain nonlinear rational terms.
2. **Coupling**: the evolution of each output depends on both inputs and also on the other output channel.
3. **Dynamic memory**: the model contains delayed outputs and delayed inputs.
4. **MIMO structure**: the system is not two independent single-input single-output subsystems, but one coupled 2-by-2 discrete-time plant.

Therefore, this code is a numerical benchmark for testing data-driven methods on a nonlinear MIMO system with coupling, memory, and time-varying local input-output behavior.

---

## Simulation setting

The main numerical setting in the script is:

The reference signal is a two-dimensional time-varying trajectory:

$$
y_d(k)=
\begin{bmatrix}
y_{d,1}(k) \\
y_{d,2}(k)
\end{bmatrix}
$$

with

$$
y_{d,1}(k)=5\sin\!\left(\frac{k}{50}\right)+2\cos\!\left(\frac{k}{20}\right)
$$

$$
y_{d,2}(k)=2\sin\!\left(\frac{k}{50}\right)+5\cos\!\left(\frac{k}{20}\right)
$$

This reference is used to evaluate the tracking ability of the numerical schemes under the same nonlinear MIMO plant.

---

## Numerical results

<img src="images\image-20260422191935853.png" alt="image-20260422191935853" style="zoom: 33%;" />

<p align="center">y<sub>1</sub> tracking performances</p>

<img src="images\image-20260422192014575.png" alt="image-20260422192014575" style="zoom: 33%;" />

<p align="center">y<sub>2</sub> tracking performances</p>

<img src="images\image-20260422192129257.png" alt="image-20260422192129257" style="zoom: 33%;" />

<p align="center">Control input u<sub>1</sub></p>

<img src="images\image-20260422192137008.png" alt="image-20260422192137008" style="zoom: 33%;" />

<p align="center">Control input u<sub>2</sub></p>

## Continuum robot system other experimental result

<img src="images\Figure_2.png" alt="Figure_2" style="zoom:50%;" />

<p align="center">y<sub>1</sub> tracking performances in continuum robots</p>

## File

- `Data_driven_control_with_error_Compensation.m`: main MATLAB script only for **the numerical example**

---

## Notes

- This repository is shared **only for the purpose of peer review**.

---

