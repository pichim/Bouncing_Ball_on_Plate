# Ball-on-Wheel Model with Vertical Degree of Freedom

## Overview

This document summarizes the extended **3-DOF ball-on-wheel model** obtained by augmenting the original 2-DOF Maple model with an additional vertical translation coordinate `z(t)`.

The generalized coordinates are

$$
q = \begin{bmatrix} x \\ \phi \\ z \end{bmatrix},
\qquad
\dot q = \begin{bmatrix} \dot x \\ \dot\phi \\ \dot z \end{bmatrix},
\qquad
\ddot q = \begin{bmatrix} \ddot x \\ \ddot\phi \\ \ddot z \end{bmatrix}.
$$

The generalized input vector is

$$
\tau_q = \begin{bmatrix} 0 \\ \tau \\ F_z \end{bmatrix},
$$

where

- $x(t)$: ball position along the wheel / rocker,
- $\phi(t)$: rocker angle,
- $z(t)$: vertical displacement of the rotary-joint center, positive upward,
- $\tau$: torque acting on the rotary joint,
- $F_z$: external force acting upward on the joint center.

---

## Assumptions

1. The original 2-DOF geometry is

   $$
   r_1 = x \cos\phi - (R+L)\sin\phi,
   $$

   $$
   r_2 = x \sin\phi + (R+L)\cos\phi.
   $$

2. The vertical degree of freedom is added by translating the full mechanism upward by $z(t)$, so the extended geometry becomes

   $$
   r_1 = x \cos\phi - (R+L)\sin\phi,
   $$

   $$
   r_2 = z + x \sin\phi + (R+L)\cos\phi.
   $$

3. The ball rolling term is kept in the same form as in the original model:

   $$
   \frac{J_k}{2}\left(\dot\phi - \frac{\dot x}{R}\right)^2.
   $$

4. The rocker/body is described by

   - rotational inertia $J$ about the rotary joint,
   - mass $M$,
   - center-of-mass distance $L_s$ from the rotary joint.

5. Gravity acts downward with magnitude $g$, and $z$ is positive upward.

6. No damping, friction, or actuator dynamics are included in this model.

---

## Extended 3-DOF Geometry

The ball-center coordinates are

$$
r_1 = x \cos\phi - (R+L)\sin\phi,
$$

$$
r_2 = z + x \sin\phi + (R+L)\cos\phi.
$$

Their time derivatives are

$$
\dot r_1 = \dot x \cos\phi - \dot\phi\bigl(x\sin\phi + (R+L)\cos\phi\bigr),
$$

$$
\dot r_2 = \dot z + \dot x \sin\phi + \dot\phi\bigl(x\cos\phi - (R+L)\sin\phi\bigr).
$$

The body center of mass is modeled as

$$
r_S = \begin{bmatrix}
- L_s \sin\phi \\
 z + L_s \cos\phi
\end{bmatrix}.
$$

---

## Kinetic Energy

The total kinetic energy is

$$
T = T_{\text{ball,trans}} + T_{\text{ball,rot}} + T_{\text{body}}.
$$

### 1. Ball translational kinetic energy

$$
T_{\text{ball,trans}} = \frac{M_k}{2}\left(\dot r_1^2 + \dot r_2^2\right).
$$

Expanded:

$$
T_{\text{ball,trans}} = \frac{M_k}{2}\Big[
\dot x^2 + \dot z^2 + \bigl(x^2 + (R+L)^2\bigr)\dot\phi^2
- 2(R+L)\dot x\dot\phi
+ 2\dot z\dot x\sin\phi
+ 2\dot z\dot\phi\bigl(x\cos\phi - (R+L)\sin\phi\bigr)
\Big].
$$

### 2. Ball rotational kinetic energy

$$
T_{\text{ball,rot}} = \frac{J_k}{2}\left(\dot\phi - \frac{\dot x}{R}\right)^2.
$$

### 3. Rocker/body kinetic energy

$$
T_{\text{body}} = \frac{J}{2}\dot\phi^2 + \frac{M}{2}\dot z^2 - M L_s \sin\phi\, \dot z\dot\phi.
$$

### Total kinetic energy

$$
T = \frac{M_k}{2}\left(\dot r_1^2 + \dot r_2^2\right)
+ \frac{J_k}{2}\left(\dot\phi - \frac{\dot x}{R}\right)^2
+ \frac{J}{2}\dot\phi^2
+ \frac{M}{2}\dot z^2
- M L_s \sin\phi\, \dot z\dot\phi.
$$

---

## Potential Energy

The total potential energy is

$$
U = U_{\text{ball}} + U_{\text{body}}.
$$

### 1. Ball potential energy

$$
U_{\text{ball}} = M_k g\, r_2 = M_k g\Big(z + x\sin\phi + (R+L)\cos\phi\Big).
$$

### 2. Rocker/body potential energy

$$
U_{\text{body}} = M g\bigl(z + L_s\cos\phi\bigr).
$$

### Total potential energy

$$
U = M_k g\Big(z + x\sin\phi + (R+L)\cos\phi\Big) + M g\bigl(z + L_s\cos\phi\bigr).
$$

Equivalently,

$$
U = (M_k + M)gz + M_k g\Big(x\sin\phi + (R+L)\cos\phi\Big) + M L_s g \cos\phi.
$$

---

## Lagrangian

$$
\mathcal{L} = T - U.
$$

The equations of motion follow from

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot q_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = (\tau_q)_i.
$$

---

## Nonlinear Differential Equations

The three nonlinear equations of motion are

$$
\left(M_k + \frac{J_k}{R^2}\right)\ddot x
+ \left(-M_k(R+L) - \frac{J_k}{R}\right)\ddot\phi
+ M_k\sin\phi\,\ddot z
- M_k x \dot\phi^2
+ M_k g\sin\phi
= 0,
$$

$$
\left(-M_k(R+L) - \frac{J_k}{R}\right)\ddot x
+ \left(J + J_k + M_k x^2 + M_k(R+L)^2\right)\ddot\phi
+ \left(M_k x\cos\phi - \bigl(M_k(R+L) + M L_s\bigr)\sin\phi\right)\ddot z
+ 2 M_k x \dot\phi \dot x
+ M_k g x\cos\phi
- \bigl(M_k(R+L) + M L_s\bigr)g\sin\phi
= \tau,
$$

$$
M_k\sin\phi\,\ddot x
+ \left(M_k x\cos\phi - \bigl(M_k(R+L) + M L_s\bigr)\sin\phi\right)\ddot\phi
+ (M + M_k)\ddot z
+ 2 M_k\cos\phi\,\dot\phi\dot x
- \left(M_k x\sin\phi + \bigl(M_k(R+L) + M L_s\bigr)\cos\phi\right)\dot\phi^2
+ (M + M_k)g
= F_z.
$$

---

## Matrix Form

The model can be written as

$$
M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q) + D\,\dot q = \tau_q.
$$

Since this model does **not** include damping, the damping matrix is

$$
D = \mathbf{0}_{3\times 3}.
$$

### Mass matrix

$$
M(q) =
\begin{bmatrix}
M_k + \dfrac{J_k}{R^2}
& -M_k(R+L) - \dfrac{J_k}{R}
& M_k\sin\phi \\
-M_k(R+L) - \dfrac{J_k}{R}
& J + J_k + M_k x^2 + M_k(R+L)^2
& M_k x\cos\phi - \bigl(M_k(R+L) + M L_s\bigr)\sin\phi \\
M_k\sin\phi
& M_k x\cos\phi - \bigl(M_k(R+L) + M L_s\bigr)\sin\phi
& M + M_k
\end{bmatrix}.
$$

### Coriolis / centrifugal matrix

One valid choice of $C(q,\dot q)$ satisfying $C(q,\dot q)\dot q$ = velocity-dependent nonlinear terms is

$$
C(q,\dot q) =
\begin{bmatrix}
0
& -M_k x\dot\phi
& 0 \\
M_k x\dot\phi
& M_k x\dot x
& 0 \\
M_k\dot\phi\cos\phi
& M_k\dot x\cos\phi - \dot\phi\Big(M_k x\sin\phi + \bigl(M_k(R+L) + M L_s\bigr)\cos\phi\Big)
& 0
\end{bmatrix}.
$$

Then

$$
C(q,\dot q)\dot q =
\begin{bmatrix}
- M_k x \dot\phi^2 \\
2 M_k x \dot\phi\dot x \\
2 M_k\cos\phi\,\dot\phi\dot x
- \left(M_k x\sin\phi + \bigl(M_k(R+L) + M L_s\bigr)\cos\phi\right)\dot\phi^2
\end{bmatrix}.
$$

### Gravity vector

$$
G(q) =
\begin{bmatrix}
M_k g\sin\phi \\
M_k g x\cos\phi - \bigl(M_k(R+L) + M L_s\bigr)g\sin\phi \\
(M + M_k)g
\end{bmatrix}.
$$

---

## Final Compact Form

With

$$
q = \begin{bmatrix} x \\ \phi \\ z \end{bmatrix},
\qquad
\tau_q = \begin{bmatrix} 0 \\ \tau \\ F_z \end{bmatrix},
$$

the final model is

$$
M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q) + D\,\dot q = \tau_q,
$$

with $D = 0$ for the current undamped model.
