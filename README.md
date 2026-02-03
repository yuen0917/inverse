# Tikhonov-Regularized Pseudo-Inverse (RTF)

Hardware and software implementation of the **regularized pseudo-inverse** (Tikhonov-regularized Moore–Penrose pseudo-inverse) for the RTF (Room / Acoustic Transfer Function) matrix, used in acoustic beamforming and inverse problems.

## Introduction

For each frequency bin, the algorithm computes:

$$
\text{pinv}_\lambda(A) = (A^H A + \lambda I)^{-1} A^H
$$

where $A$ is the RTF matrix (sources × microphones), $A^H$ is its conjugate transpose, and $\lambda$ is the regularization parameter. The result is applied to form beamformer weights or to solve regularized least-squares problems in the frequency domain.

The hardware (`RTF_top.v`) implements this for a **2×2** complex $G = A^H A + \lambda I$: it builds $G$, computes $\det(G)$, $1/\det$, $G^{-1}$, and then $G^{-1} A^H$ (per frequency, per microphone), writing the result to BRAM.

## Files

| File | Description |
|------|-------------|
| **RTF_top.v** | Top-level RTL: FSM that reads RTF data from BRAM, computes $G = A^H A + \lambda I$, inverts the 2×2 $G$, computes $G^{-1} A^H$, and writes results to BRAM. Interfaces to an external divider IP for $1/\det$. |
| **RTF_top_tb.v** | Verilog testbench for `RTF_top`: drives BRAM-style inputs, checks `done` / `all_freq_finish`, and can dump results for comparison with the software model. |
| **RTF_software.py** | Python software model that mirrors the RTL (fixed-width, two’s-complement) to verify hardware. Uses the same BRAM pattern as `RTF_top_tb.v` and reproduces the FSM steps (G accumulation, det, inv_det, inv_G, result elements). |
| **inverse_test.ipynb** | Jupyter notebook: builds free-field RTF matrix `Af` from geometry (DOA, mic positions, sample rate), then computes the regularized pseudo-inverse per frequency (`pinvA = inv(RTF^H @ RTF + λI) @ RTF^H`) with NumPy for reference / comparison. |
| **hermitian.v** | Helper module: reads complex data from BRAM and writes its conjugate (Hermitian transpose in vector form). Can be used to prepare $A^H$ from $A$ if data is stored as $A$. |
| **divider_test.v** | Test/example for the divider IP used by `RTF_top` (dividend/divisor in, quotient out). |
| **divider_top_tb.v** | Testbench for the divider top wrapper. |
| **RTF_software_output.txt** | Sample or reference output from `RTF_software.py` (optional). |

## Top Module Structure

The testbenches instantiate Vivado block-design wrappers that tie RTL to the divider IP:

- **RTF_top_tb.v**
  Instantiates **design_2**, which contains:
  - Vivado **Divider** IP (AXI-Stream or equivalent interface)
  - **RTF_top.v** (drives dividend/divisor, consumes quotient)

- **divider_top_tb.v**
  Instantiates **design_1**, which contains:
  - Vivado **Divider** IP
  - **divider_test.v** (test wrapper around the divider)

```text
RTF_top_tb
  └── design_2
        ├── Vivado Divider IP
        └── RTF_top.v

divider_top_tb
  └── design_1
        ├── Vivado Divider IP
        └── divider_test.v
```

## State Machine (RTF_top.v)

The RTL uses a single FSM. Each frequency bin is processed in one pass; the flow repeats for all `FREQ_NUM` bins.

| State | Name | Description |
|-------|------|-------------|
| **S_IDLE** (0) | Wait start | Waits for `start` (after `LATENCY` delay). When start is seen, clears counters, `sor0`/`sor1` temp arrays, G accumulators, inverse-G, det, and result registers. |
| **S_RD** (1) | Read BRAM | Reads one complex sample from BRAM (`af_bram_rd_real` / `af_bram_rd_imag`). If reading first source row (`flag_rd_sor1==0`): stores into `sor0_temp`, accumulates `g11_real_acc` = $Σ\|sor0\|²$. If reading second row: stores into `sor1_temp`, accumulates `g22_real_acc` = $Σ\|sor1\|²$ and `g12_real_acc`, `g12_imag_acc` = $Σ sor0*·sor1$ (cross terms). Repeats until `rd_cnt == PER_FREQ - 1` (all MIC_NUM×SOR_NUM samples for this frequency read). |
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

## Parameters (RTF_top.v)

- **MIC_NUM**, **SOR_NUM**, **FREQ_NUM**: number of mics, sources, and frequency bins.
- **DATA_WIDTH**: bit width of RTF real/imag (e.g. 16).
- **LAMBDA**: regularization $\lambda$ (fixed-point, same scale as $G$).
- **BRAM**: address widths, base, and step for read/write BRAM.
- **DIVOUT / DIVISOR / DIVIDEND**: widths for the divider interface (quotient and fractional part).

## Bit-Width Growth (RTF_top.v)

To match hardware and software exactly, it is useful to track how the bit-width grows along the pipeline for one complex sample (real/imag both use the same width):

- **Input from BRAM**
  - `af_bram_rd_real`, `af_bram_rd_imag`: **DATA_WIDTH = 16** bits (signed).
  - `sor0_temp_*[i]`, `sor1_temp_*[i]`: **16-bit** signed registers (just latched copies of the inputs).

- **G accumulation: $G = A^H A + \lambda I$**
  - `g11_real_acc`, `g12_real_acc`, `g12_imag_acc`, `g22_real_acc`:
    - Width: **DATA_WIDTH*2 = 32 bits** signed.
    - Reason: each term is a sum of products like $a \cdot a$ or $a \cdot b$, where each operand is 16-bit; the product is up to 32-bit, and accumulation stays in 32 bits (two's complement wrap).
  - `g12_real_acc_sqr`, `g12_imag_acc_sqr`:
    - Wires: **32-bit** products of 32×32, but truncated/wrapped back to 32 bits (same as hardware multiplication result assigned into 32-bit wire).
  - `det`:
    - Width: **32 bits** signed.
    - Computed as: `det = g11_real_acc * g22_real_acc - (g12_real_acc_sqr + g12_imag_acc_sqr)` (all in 32-bit wrap).

- **Divider interface (for $1/\det$)**
  - `s_axis_divisor_tdata`: **32 bits** (same as `det`).
  - `s_axis_dividend_tdata`: **32 bits** (we drive it with constant 1).
  - Divider output `m_axis_dout_tdata`:
    - Width: **DIVOUT_TDATA_WIDTH = 64 bits** signed.
    - Parameter `DIVOUT_F_WIDTH = 32`. Because the MSB is the sign bit, the **effective fractional resolution is `DIVOUT_F_WIDTH - 1 = 31` bits**, i.e. Q1.31 format.
  - Internally split in RTL:
    - `inv_det_q`: **32-bit** signed integer part.
    - `inv_det_f`: **31-bit** fractional part (plus sign handling via `inv_det_q`).
    - Recombined as:  
      `inv_det = ($signed(inv_det_q) <<< (DIVOUT_F_WIDTH - 1)) + $signed(inv_det_f);`  
      → overall scale is $2^{31}$ (Q1.31).

- **Inverse matrix elements $G^{-1}$**
  - `inv_g11_real`, `inv_g12_real`, `inv_g12_imag`, `inv_g22_real`:
    - Width: **DATA_WIDTH*4 = 64 bits** signed.
    - Each is a product of a 32-bit `g*` term and the 64-bit (Q1.31) `inv_det`, with the result truncated/wrapped into 64 bits.

- **Result elements and final output**
  - `result_real_element0/1/2`, `result_imag_element0/1/2`:
    - Width: **DATA_WIDTH*4 = 64 bits** signed.
    - Each is a product of one `inv_g*` (64-bit) with a 16-bit `sor*` element, truncated/wrapped back to 64 bits.
  - `result_bram_wr_real`, `result_bram_wr_imag`:
    - Width: **DATA_WIDTH*4 = 64 bits** signed.
    - Computed as the sum of three 64-bit partial results (for each row), truncated/wrapped into 64 bits.

So, in short, the **real/imag path** grows as:

- **16-bit input** (BRAM, `sor*`) → **32-bit** accumulators (`g*`, `det`) →  
- **64-bit** fixed-point reciprocal (`inv_det`) → **64-bit** inverse matrix elements (`inv_g*`, per-element products) →  
- **64-bit output** written back to BRAM (`result_bram_wr_*`).

## How to Run

1. **Software reference (floating-point)**
   Open `inverse_test.ipynb` and run all cells to get `Af` and `pinvA` per frequency.

2. **Software model (RTL-equivalent)**
   Run `RTF_software.py` (e.g. `python RTF_software.py`). It uses the same BRAM pattern as the testbench and prints or saves results for comparison with RTL simulation.

3. **RTL simulation**
   Simulate `RTF_top_tb.v` with your Verilog simulator (e.g. instantiate `design_2` or your top that contains `RTF_top` + divider). Compare outputs with `RTF_software.py` or with exported results from the notebook.

## Dependencies

- **inverse_test.ipynb**: NumPy, Jupyter.
- **RTF_software.py**: Python 3 with standard library (no NumPy required for the fixed-point model).
- **RTL**: Verilog simulator; divider IP must match the interface and width parameters used in `RTF_top.v`.
