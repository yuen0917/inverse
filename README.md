# Tikhonov-Regularized Pseudo-Inverse (RTF)

Hardware and software implementation of the **regularized pseudo-inverse** (Tikhonov-regularized Moore–Penrose pseudo-inverse) for the RTF (Room / Acoustic Transfer Function) matrix, used in acoustic beamforming and inverse problems.

## Introduction

For each frequency bin, the algorithm computes:

$$
\text{pinv}_\lambda(A) = (A^H A + \lambda I)^{-1} A^H
$$

where $A$ is the RTF matrix (sources × microphones), $A^H$ is its conjugate transpose, and $\lambda$ is the regularization parameter. The result is applied to form beamformer weights or to solve regularized least-squares problems in the frequency domain.

The hardware (`inverse_top.v`) implements this for a **2×2** complex $G = A^H A + \lambda I$: it builds $G$, computes $\det(G)$, $1/\det$, $G^{-1}$, and then $G^{-1} A^H$ (per frequency, per microphone), writing the result to BRAM.

## Files

| File | Description |
|------|-------------|
| **inverse_top.v** | Top-level RTL: FSM that reads RTF data from BRAM, computes $G = A^H A + \lambda I$, inverts the 2×2 $G$, computes $G^{-1} A^H$, and writes results to BRAM. Interfaces to an external divider IP for $1/\det$. |
| **inverse_top_tb.v** | Verilog testbench for `inverse_top`: drives BRAM-style inputs, checks `done` / `all_freq_finish`, and can dump results for comparison with the software model. |
| **inverse_software.py** | Python software model that mirrors the RTL (fixed-width, two’s-complement) to verify hardware. Uses the same BRAM pattern as `inverse_top_tb.v` and reproduces the FSM steps (G accumulation, det, inv_det, inv_G, result elements). |
| **inverse_test.ipynb** | Jupyter notebook: builds free-field RTF matrix `Af` from geometry (DOA, mic positions, sample rate), then computes the regularized pseudo-inverse per frequency (`pinvA = inv(RTF^H @ RTF + λI) @ RTF^H`) with NumPy for reference / comparison. |
| **hermitian.v** | Helper module: reads complex data from BRAM and writes its conjugate (Hermitian transpose in vector form). Can be used to prepare $A^H$ from $A$ if data is stored as $A$. |
| **divider_test.v** | Test/example for the divider IP used by `inverse_top` (dividend/divisor in, quotient out). |
| **divider_top_tb.v** | Testbench for the divider top wrapper. |
| **inverse_software_output.txt** | Sample or reference output from `inverse_software.py` (optional). |

## Top Module Structure

The testbenches instantiate Vivado block-design wrappers that tie RTL to the divider IP:

- **inverse_top_tb.v**
  Instantiates **design_2**, which contains:
  - Vivado **Divider** IP (AXI-Stream or equivalent interface)
  - **inverse_top.v** (drives dividend/divisor, consumes quotient)

- **divider_top_tb.v**
  Instantiates **design_1**, which contains:
  - Vivado **Divider** IP
  - **divider_test.v** (test wrapper around the divider)

```text
inverse_top_tb
  └── design_2
        ├── Vivado Divider IP
        └── inverse_top.v

divider_top_tb
  └── design_1
        ├── Vivado Divider IP
        └── divider_test.v
```

## State Machine (inverse_top.v)

The RTL uses a single FSM. Each frequency bin is processed in one pass; the flow repeats for all `FREQ_NUM` bins.

| State | Name | Description |
|-------|------|-------------|
| **S_IDLE** (0) | Wait start | Waits for `start` (after `LATENCY` delay). When start is seen, clears counters, `sor0`/`sor1` temp arrays, G accumulators, inverse-G, det, and result registers. |
| **S_RD** (1) | Read BRAM | Reads one complex sample from BRAM (`af_bram_rd_real` / `af_bram_rd_imag`). If reading first source row (`flag_rd_sor1==0`): stores into `sor0_temp`, accumulates `g11_real_acc` = Σ\|sor0\|². If reading second row: stores into `sor1_temp`, accumulates `g22_real_acc` = Σ\|sor1\|² and `g12_real_acc`, `g12_imag_acc` = Σ sor0*·sor1 (cross terms). Repeats until `rd_cnt == PER_FREQ - 1` (all MIC_NUM×SOR_NUM samples for this frequency read). |
| **S_UPDATE_RD_ADDR** (2) | Update read address | Increments `sor_cnt` (mic index); when `sor_cnt` wraps after MIC_NUM, toggles `flag_rd_sor1`. Advances `bram_rd_addr` by `BRAM_RD_INCREASE`. Next state is always `S_RD`. |
| **S_PLUS** (3) | Add λI to G | Adds `LAMBDA` to `g11_real_acc` and `g22_real_acc`, completing $G = A^H A + \lambda I$. Resets `rd_cnt`, `sor_cnt`, `flag_rd_sor1` for later use; advances read address once. |
| **S_CALDET1** (4) | det first term | Computes `det = g11_real_acc * g22_real_acc` (first term of $\det(G) = g_{11}g_{22} - \|g_{12}\|^2$). |
| **S_CALDET2** (5) | det second term | Subtracts $\|g_{12}\|^2$: `det = det - (g12_real_acc_sqr + g12_imag_acc_sqr)`, giving the full $\det(G)$. |
| **S_INVDET** (6) | Set divider inputs | Drives divider: `s_axis_divisor_tdata = det`, `s_axis_dividend_tdata = 1` (to compute $1/\det$). Valid signals are still 0. |
| **S_SETDIV** (7) | Start divider | Asserts `s_axis_dividend_tvalid` and `s_axis_divisor_tvalid` to hand off the division to the Vivado Divider IP. |
| **S_WAITDIV** (8) | Wait divider | Deasserts valid; waits for `m_axis_dout_tvalid`. When valid, latches quotient into `inv_det_q` (integer part) and `inv_det_f` (fractional part), then goes to `S_CALINVG`. |
| **S_CALINVG** (9) | Inverse of G | Computes $G^{-1} = (1/\det)\cdot\text{adj}(G)$: `inv_g11_real = g22*inv_det`, `inv_g12_real/imag = -g12*inv_det`, `inv_g22_real = g11*inv_det`. |
| **S_CALRESULT** (10) | Result elements | Computes one output sample of $G^{-1} A^H$: multiplies `inv_g*` by `sor0_temp`/`sor1_temp` (by row: `result_row1` selects row 1 or 2), fills `result_real_element0/1/2` and `result_imag_element0/1/2`. |
| **S_WR** (11) | Write BRAM | Enables BRAM write: sums real/imag elements into `result_bram_wr_real` / `result_bram_wr_imag` and writes to current `bram_wr_addr`. Increments `wr_cnt` until `PER_FREQ` samples for this frequency are written. |
| **S_UPDATE_WR_ADDR** (12) | Update write address | Increments `sor_cnt`; when it wraps after MIC_NUM, toggles `result_row1`. Advances `bram_wr_addr` by `BRAM_WR_INCREASE`. Next state is `S_CALRESULT` (compute next output sample, then write again). |
| **S_DONE** (13) | Done | Sets `done = 1`; updates `freq_sample_cnt` and `all_freq_finish` when all `FREQ_NUM` bins are done. Resets `wr_cnt`, `sor_cnt`, `result_row1`; advances `bram_wr_addr` for the next run. Returns to `S_IDLE`. |

Flow summary for one frequency: **S_IDLE → S_RD ⇄ S_UPDATE_RD_ADDR** (until all RTF samples read) **→ S_PLUS → S_CALDET1 → S_CALDET2 → S_INVDET → S_SETDIV → S_WAITDIV** (until divider done) **→ S_CALINVG → S_CALRESULT → S_WR ⇄ S_UPDATE_WR_ADDR** (until all result samples written) **→ S_DONE → S_IDLE**. Then the same sequence runs for the next frequency until `freq_sample_cnt` has covered all `FREQ_NUM` bins.

## Parameters (inverse_top.v)

- **MIC_NUM**, **SOR_NUM**, **FREQ_NUM**: number of mics, sources, and frequency bins.
- **DATA_WIDTH**: bit width of RTF real/imag (e.g. 16).
- **LAMBDA**: regularization $\lambda$ (fixed-point, same scale as $G$).
- **BRAM**: address widths, base, and step for read/write BRAM.
- **DIVOUT / DIVISOR / DIVIDEND**: widths for the divider interface (quotient and fractional part).

## How to Run

1. **Software reference (floating-point)**
   Open `inverse_test.ipynb` and run all cells to get `Af` and `pinvA` per frequency.

2. **Software model (RTL-equivalent)**
   Run `inverse_software.py` (e.g. `python inverse_software.py`). It uses the same BRAM pattern as the testbench and prints or saves results for comparison with RTL simulation.

3. **RTL simulation**
   Simulate `inverse_top_tb.v` with your Verilog simulator (e.g. instantiate `design_2` or your top that contains `inverse_top` + divider). Compare outputs with `inverse_software.py` or with exported results from the notebook.

## Dependencies

- **inverse_test.ipynb**: NumPy, Jupyter.
- **inverse_software.py**: Python 3 with standard library (no NumPy required for the fixed-point model).
- **RTL**: Verilog simulator; divider IP must match the interface and width parameters used in `inverse_top.v`.
