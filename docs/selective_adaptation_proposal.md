# Proposal: Selective Adaptation & Confidence-Weighted Updates for FOC-Stim

## 1. Context: The Observability Problem
The FOC-Stim device utilizes a Model Reference Adaptive Control (MRAC) system to estimate the complex impedance of each electrode ($Z_1 \dots Z_4$). 

To achieve full model convergence (evaluating all 4 impedances), the input signal must provide **Persistent Excitation (PE)**. In the context of the 4-phase star configuration, this means the current must flow through enough linearly independent paths to distinguish between the individual transformers and the load.

### Current Challenges:
1.  **Divergence at Low Power:** When the commanded signal is small, the error signal is dominated by measurement noise. The model "learns" this noise, causing estimates to drift into nonsensical ranges (e.g., negative resistance).
2.  **Unexcited Paths:** If a pattern only involves current flow between $Z_1$ and $Z_2$, the estimates for $Z_3$ and $Z_4$ "float" and drift over time due to sensor DC offsets and mathematical integration wind-up.
3.  **Jarring Randomness:** The current "quick hack" of jumping to a new random phase state every $N$ pulses provides the necessary excitation but creates perceptible, jarring transitions for the user.

---

## 2. Proposed Solution: Confidence-Weighted Adaptation

Instead of treating every measurement equally, we assign a **Confidence Score** ($\Psi$) to each update step. This score represents how "informative" the current pulse is for a specific electrode.

### 2.1 The Confidence Metric
The confidence for electrode $k$ should be proportional to the energy of the commanded signal for that electrode during the measurement window.

$$\Psi_k = 	ext{clamp}\left( \frac{|i_{cmd, k}| - I_{noise\_floor}}{I_{reliable}}, 0, 1 ight)$$

*   $I_{noise\_floor}$: A threshold below which measurements are considered pure noise.
*   $I_{reliable}$: The current level at which we have 100% confidence in the sensor reading.

### 2.2 Weighted Gradient Descent
The standard update step in `fourphase_model.cpp` is:
$$Z_k = Z_k + \gamma \cdot 	ext{Error}_k$$

We modify this to use the confidence score as a multiplier for the learning rate ($\gamma$):
$$Z_k = Z_k + (\gamma \cdot \Psi_k) \cdot 	ext{Error}_k$$

**Result:** 
*   If an electrode is inactive ($i_{cmd} \approx 0$), its impedance estimate is effectively **locked**. 
*   The model only "learns" when there is actual signal to learn from.

---

## 3. Integration with Electrode-Power Math (e1, e2, e3, e4)

With the deprecation of Alpha/Beta/Gamma in favor of direct electrode power ($e_1 \dots e_4$), the confidence-weighted approach becomes even more natural. Each $e_k$ directly informs the confidence $\Psi_k$ for its corresponding impedance $Z_k$.

### Refined Algorithm:
1.  **Smooth Phase Dithering:** Apply a very slow, continuous rotation to the `start_angle` ($	heta$). This ensures that over a window of several seconds, all return paths are exercised.
2.  **Selective Adaptation:** Apply the confidence-weighted update. This ensures that during the dithering process, we only update the parts of the model that are currently "in focus."

---

## 4. Implementation Details (C++ Pseudocode)

In `FourphaseModel::perform_one_update_step()`:

```cpp
// Existing error calculation
float err1 = context[i].i1_meas - context[i].i1_cmd;

// NEW: Confidence calculation based on commanded signal energy
// ignores noise floor and prevents drift during inactivity
float energy1 = context[i].i1_cmd * context[i].i1_cmd;
float confidence1 = energy1 > NOISE_THRESHOLD_SQUARED ? 1.0f : 0.0f; 

// Apply weighted update
magnitude_error1 += gamma1 * confidence1 * (context[i].i1_cmd * err1);
angle_error1 += gamma2 * confidence1 * (dx1 * err1);
```

---

## 5. Expected Benefits
1.  **Elimination of "Noise-Drift":** Resistance values will remain stable even when the app is idling or playing at very low volumes.
2.  **Perceptual Smoothness:** By combining this with continuous phase rotation, we can achieve model convergence without discrete "jumps" in the current flow.
3.  **Robustness:** The model becomes much more resistant to "zap" events caused by one electrode estimate drifting to an extreme value while it was not being used.
