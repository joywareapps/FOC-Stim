# FOC-Stim Firmware Analysis: 4-Phase Signal Generation & Model Convergence

## 1. Hardware Overview
The FOC-Stim hardware utilizes a **Star Configuration** for its 4-phase output stage:
*   **4 Drivers:** Independent H-bridge/Half-bridge drivers.
*   **4 Audio Transformers:** Used for galvanic isolation and voltage step-up.
*   **Common Neutral:** The secondary coils of all 4 transformers meet at a single shared point.
*   **Return Path:** Current from any electrode (A, B, C, or D) must return through the other three. The system must satisfy Kirchhoff's Current Law: $i_A + i_B + i_C + i_D = 0$.

---

## 2. 4-Phase Algorithm: From 3D Space to 2D Complex Plane

The core challenge is mapping a 3D position (alpha, beta, gamma) to 4 electrode currents while maintaining the zero-sum constraint.

### 2.1 Basis Projection
The firmware defines 4 basis vectors in 3D space ($\hat{b}_1 \dots \hat{b}_4$). For any target position $\vec{pos}$, it calculates desired RMS amplitudes ($a_1 \dots a_4$) for each electrode. This is a geometric projection that ensures the resulting "vector sum" of the felt sensation matches the intended direction.

### 2.2 The "Infinitely Many Ways" (Degrees of Freedom)
Because we are using AC signals (represented as complex numbers $p_1 \dots p_4$), the constraint $\sum p_k = 0$ allows for extra degrees of freedom even when amplitudes $|p_k| = a_k$ are fixed:
1.  **Internal Phase Rotation:** All 4 points can be rotated by an angle $	heta$ on the complex plane without changing the RMS amplitudes or the zero-sum balance.
2.  **Internal Summing Point ($p$):** The algorithm groups electrodes into pairs $(1,2)$ and $(3,4)$. It picks an arbitrary point $p$ such that $p_1+p_2=p$ and $p_3+p_4=-p$. There is a valid range for $p$, and picking any value within that range changes the phase relationship *between* the pairs.

---

## 3. The Impedance Model (Convergence)

The device maintains an internal estimate of the complex impedance ($Z_k$) for each electrode. This is critical for:
1.  **Safety:** Detecting disconnected or shorted electrodes.
2.  **Precision:** Ensuring the actual current matches the user's requested intensity.

### 3.1 Adaptive Estimation (MRAC)
The firmware uses a **Model Reference Adaptive Control** approach. During each pulse:
*   It measures $i_{meas, k}$ and $v_{meas, k}$.
*   It calculates error relative to the model $i_{cmd, k}$.
*   It updates $Z_k$ using a gradient descent update:
    *   **Magnitude update:** Correlates error with signal amplitude.
    *   **Angle update:** Correlates error with signal derivative (frequency/phase).

---

## 4. The Problem: Randomness vs. Smoothness

### 4.1 Persistent Excitation
To accurately estimate 4 independent complex impedances, the signal must be "sufficiently exciting." If the phase relationship between electrodes is static, the model may only see a subset of the possible current paths (e.g., current mostly flowing A $	o$ B and C $	o$ D). This causes the model to "stall" or drift on the unexcited paths.

### 4.2 The Developer's Findings
*   **Randomization helps convergence:** By rotating the "random state" (the start angle $	heta$ and polarity), all paths are eventually exercised, and the model converges better.
*   **Current flow is perceptible:** Even if RMS amplitudes are constant, changing the phase relationship (how current splits between return paths) is felt by the user. 
*   **Discrete jumps are jarring:** Switching the random state every pulse or every $N$ pulses creates a felt "click" or "glitch" because the internal current distribution jumps instantly.

---

## 5. Proposed Improvements

### A. Slow Continuous Rotation (Phase Dithering)
Instead of jumping to a new random angle $	heta$ every $N$ pulses, apply a very slow, continuous rotation (e.g., $	heta(t) = \omega_{slow} \cdot t$).
*   **Effect:** Current paths shift continuously and smoothly.
*   **Result:** Persistent excitation for the model without discrete jars. Might feel like a slow "swirl."

### B. Randomized $p$-midpoint
In `fourphase_math.cpp`, $p$ is currently hardcoded as the midpoint of the valid range. We could slowly oscillate this $p$ within its valid range.
*   **Effect:** Changes the phase relationship between electrode pairs $(1,2)$ and $(3,4)$ smoothly.

### C. Cross-faded State Transitions
If discrete states are preferred, the transition between State A and State B should be cross-faded over several hundred milliseconds (many pulses) rather than a single step.

### D. Sub-perceptual Excitation
Add very small, high-frequency "noise" to the phase relationships that is below the human tactile threshold but sufficient to keep the model's gradient descent active.

---

## 6. Next Steps for Discussion
1.  **Analyze `fourphase_math_2.cpp`:** It seems to implement a more complex interpolation involving "PointCoords" (A, B, C, D, AB, AC, etc.). We should check if this version already attempts to address some of these flow issues.
2.  **Simulation:** Test if continuous rotation ($	heta$) provides sufficient model convergence in a simulated star-load environment.
