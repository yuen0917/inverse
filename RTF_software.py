"""
Software model for `RTF_top` to verify hardware results.

All arithmetic is done with explicit fixed bit widths and two's‑complement
wrapping, to mimic the Verilog implementation as closely as possible.

Simplification for now:
    - Treat divider output as pure integer: inv_det = 1 / det (truncated)
      without fractional bits.
    - If det == 0, skip division and keep inv_det = 0.

Input pattern:
    Same as `RTF_top_tb.v`:
        bram_rd_mem_real[i] = (i % 1000) - 500
        bram_rd_mem_imag[i] = ((i + 1) % 1000) - 500
    where
        i = freq * (MIC_NUM * SOR_NUM) + sor * MIC_NUM + mic
"""

from __future__ import annotations
from datetime import datetime
import numpy as np

# ---------------------------------------------------------------------------
# Parameters (mirror RTF_top.v)
# ---------------------------------------------------------------------------

MIC_NUM = 8
SOR_NUM = 2
FREQ_NUM = 257

DATA_WIDTH = 16
ACC_WIDTH = DATA_WIDTH * 4          # g11, g12, g22 accumulation (32)
INV_G_WIDTH = DATA_WIDTH * 8        # inv_g* and result elements (96)
DET_WIDTH = DATA_WIDTH * 4          # det width (64), maps to DIVISOR_TDATA_WIDTH
INPUT_FRAC_BITS = 14
INPUT_Q_FORMAT = "s2.14"

# Divider configuration (mirror inverse_top.v)
DIVOUT_TDATA_WIDTH = 64
DIVOUT_F_WIDTH = 55                 # 48 fractional bits after divider
DIVIDEND_TDATA_WIDTH = 32
DIVISOR_TDATA_WIDTH = 32

# Regularization lambda in same scale as G (Q4.28 when INPUT_FRAC_BITS=14).
# 0.01 in Q4.28 = round(0.01 * 2^(2*INPUT_FRAC_BITS)) = 2,684,355.
LAMBDA = 2684355

# Float comparison (set False to skip)
ENABLE_FLOAT_COMPARE = True

PER_FREQ = MIC_NUM * SOR_NUM
TOTAL_NUM = MIC_NUM * SOR_NUM * FREQ_NUM

REAL_INPUT_PATH = "./RTF_input_Real.txt"
IMAG_INPUT_PATH = "./RTF_input_Imag.txt"

BRAM_REAL: list[int] | None = None
BRAM_IMAG: list[int] | None = None


# ---------------------------------------------------------------------------
# Two's‑complement helpers
# ---------------------------------------------------------------------------

def to_sint(value: int, bits: int) -> int:
    """Convert integer to signed two's‑complement with given bit width."""
    mask = (1 << bits) - 1
    value &= mask
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


def add_tc(a: int, b: int, bits: int) -> int:
    """Two's‑complement add with wrapping."""
    return to_sint(a + b, bits)


def sub_tc(a: int, b: int, bits: int) -> int:
    """Two's‑complement subtract with wrapping."""
    return to_sint(a - b, bits)


def mul_tc(a: int, b: int, bits: int) -> int:
    """Two's‑complement multiply with wrapping to `bits`."""
    return to_sint(a * b, bits)


def div_tc(numer: int, denom: int, bits: int) -> int:
    """
    Integer division numer / denom with truncation toward zero,
    wrapped to two's‑complement `bits`.
    If denom == 0, return 0 (matches a safe‑guard behavior).
    """
    if denom == 0:
        return 0
    # Python's // truncates toward negative infinity, so correct sign manually.
    if numer * denom >= 0:
        q = abs(numer) // abs(denom)
    else:
        q = - (abs(numer) // abs(denom))
    return to_sint(q, bits)


def fixed_div_1_over_det(det: int) -> int:
    """
    Fixed‑point version of 1/det with DIVOUT_F_WIDTH fractional bits.

    The ideal fixed‑point value would be:
        inv_det_raw = round((1 << DIVOUT_F_WIDTH) / det)
    encoded on DIVOUT_TDATA_WIDTH bits (signed two's‑complement).

    This mimics a divider that outputs a signed Q(DIVOUT_TDATA_WIDTH-DIVOUT_F_WIDTH).DIVOUT_F_WIDTH
    fixed‑point number without extra scaling.
    """
    # numerator is 1.0 in fixed‑point with DIVOUT_F_WIDTH fractional bits
    numer = 1 << DIVOUT_F_WIDTH
    return div_tc(numer, det, DIVOUT_TDATA_WIDTH)


# ---------------------------------------------------------------------------
# Testbench‑style BRAM pattern
# ---------------------------------------------------------------------------

def load_bram_inputs(
    real_path: str, imag_path: str, expected_len: int
) -> tuple[list[int], list[int]]:
    """Load BRAM inputs from text files (one integer per line)."""
    with open(real_path, "r", encoding="utf-8") as f_real:
        real_vals = [int(line.strip()) for line in f_real if line.strip()]
    with open(imag_path, "r", encoding="utf-8") as f_imag:
        imag_vals = [int(line.strip()) for line in f_imag if line.strip()]

    if len(real_vals) < expected_len or len(imag_vals) < expected_len:
        raise ValueError(
            "Input length mismatch: "
            f"real={len(real_vals)}, imag={len(imag_vals)}, "
            f"expected={expected_len}"
        )

    # Keep only the first expected_len samples if files are longer.
    return real_vals[:expected_len], imag_vals[:expected_len]


def tb_bram_value(index: int) -> tuple[int, int]:
    """
    Reproduce the BRAM initialization used in RTF_top_tb.v:

        bram_rd_mem_real[i] = (i % 1000) - 500
        bram_rd_mem_imag[i] = ((i + 1) % 1000) - 500

    Returned values are clipped/wrapped to DATA_WIDTH bits.
    """
    if BRAM_REAL is not None and BRAM_IMAG is not None:
        real = BRAM_REAL[index]
        imag = BRAM_IMAG[index]
    else:
        real = (index % 1000) - 500
        imag = ((index + 1) % 1000) - 500
    return to_sint(real, DATA_WIDTH), to_sint(imag, DATA_WIDTH)


def build_af_for_freq(freq: int) -> tuple[list[complex], list[complex]]:
    """
    Build sor0 and sor1 arrays (length MIC_NUM) for a given frequency index,
    using the same indexing as the hardware BRAM.
    """
    sor0 = []
    sor1 = []
    base = freq * PER_FREQ
    for mic in range(MIC_NUM):
        idx0 = base + 0 * MIC_NUM + mic  # sor0
        idx1 = base + 1 * MIC_NUM + mic  # sor1
        r0, i0 = tb_bram_value(idx0)
        r1, i1 = tb_bram_value(idx1)
        sor0.append(complex(r0, i0))
        sor1.append(complex(r1, i1))
    return sor0, sor1


# ---------------------------------------------------------------------------
# Core algorithm: mirror RTF_top FSM computations (combinational view)
# ---------------------------------------------------------------------------

def compute_g_matrix(
    sor0: list[complex], sor1: list[complex]
) -> tuple[int, int, int, int, int, int]:
    """
    Compute g11_real_acc, g12_real_acc, g12_imag_acc, g22_real_acc
    with the same accumulation pattern and widths as hardware.

    Returns:
        (g11_before_lambda, g12_real_acc, g12_imag_acc, g22_before_lambda,
         g11_after_lambda,  g22_after_lambda)
    """
    g11_real_acc = 0
    g12_real_acc = 0
    g12_imag_acc = 0
    g22_real_acc = 0

    for mic in range(MIC_NUM):
        a0 = sor0[mic]
        a1 = sor1[mic]

        # sor0 contribution to g11
        g11_term = int(a0.real) * int(a0.real) + int(a0.imag) * int(a0.imag)
        g11_real_acc = add_tc(g11_real_acc, g11_term, ACC_WIDTH)

        # sor1 contribution to g22
        g22_term = int(a1.real) * int(a1.real) + int(a1.imag) * int(a1.imag)
        g22_real_acc = add_tc(g22_real_acc, g22_term, ACC_WIDTH)

        # g12 accumulation: conj(sor0) * sor1
        g12_r_term = int(a0.real) * int(a1.real) + int(a0.imag) * int(a1.imag)
        g12_i_term = int(a0.real) * int(a1.imag) - int(a0.imag) * int(a1.real)
        g12_real_acc = add_tc(g12_real_acc, g12_r_term, ACC_WIDTH)
        g12_imag_acc = add_tc(g12_imag_acc, g12_i_term, ACC_WIDTH)

    # Record AH * A(F) result (before adding lambda on diagonal)
    g11_before = g11_real_acc
    g22_before = g22_real_acc

    # PLUS state: add lambda to diagonal terms (G + lambda*I)
    g11_after = add_tc(g11_real_acc, LAMBDA, ACC_WIDTH)
    g22_after = add_tc(g22_real_acc, LAMBDA, ACC_WIDTH)

    return g11_before, g12_real_acc, g12_imag_acc, g22_before, g11_after, g22_after


def compute_inv_g(g11: int, g12_r: int, g12_i: int, g22: int) -> tuple[int, int, int, int]:
    """
    Mirror S_CALDET1, S_CALDET2, S_INVDET, S_CALINVG.
    All arguments are ACC_WIDTH‑bit signed.
    """
    # det = g11 * g22  (stored in 32‑bit det register in hardware)
    det_mul = mul_tc(g11, g22, DET_WIDTH)

    # subtract |g12|^2
    g12_r_sqr = mul_tc(g12_r, g12_r, DET_WIDTH)
    g12_i_sqr = mul_tc(g12_i, g12_i, DET_WIDTH)
    det_sub = add_tc(g12_r_sqr, g12_i_sqr, DET_WIDTH)
    det = sub_tc(det_mul, det_sub, DET_WIDTH)

    # inv_det ≈ 1 / det in fixed‑point, with DIVOUT_F_WIDTH fractional bits
    inv_det_fp = fixed_div_1_over_det(det)  # 64‑bit fixed‑point (Q32.32‑like)

    # In hardware they multiply inv_det (divider output) with 32‑bit g* and
    # keep the result in 48‑bit registers. Here we mimic that by doing the
    # multiply and wrapping to INV_G_WIDTH bits; the common 2^DIVOUT_F_WIDTH
    # scale factor is kept implicitly (same as RTL).
    inv_g11_real = mul_tc(g22,    inv_det_fp, INV_G_WIDTH)
    inv_g12_real = mul_tc(-g12_r, inv_det_fp, INV_G_WIDTH)
    inv_g12_imag = mul_tc(-g12_i, inv_det_fp, INV_G_WIDTH)
    inv_g22_real = mul_tc(g11,    inv_det_fp, INV_G_WIDTH)

    return inv_g11_real, inv_g12_real, inv_g12_imag, inv_g22_real


def compute_outputs_for_freq(freq: int) -> dict:
    """
    Compute the 16 complex outputs (row0 for MIC_NUM, then row1 for MIC_NUM)
    for a given frequency index, with the same per‑mic formulas as S_CALRESULT/S_WR.

    Returns a dictionary with:
        - 'sor0', 'sor1': input vectors (list of complex)
        - 'g_before': (g11, g12_real, g12_imag, g22) before adding lambda
        - 'g_after' : (g11, g12_real, g12_imag, g22) after adding lambda
        - 'inv_g'   : (inv_g11_real, inv_g12_real, inv_g12_imag, inv_g22_real)
        - 'outputs' : list of 16 (real, imag) tuples, wrapped to INV_G_WIDTH bits.
                      Order: mic 0..7 (row0), then mic 0..7 (row1).
    """
    sor0, sor1 = build_af_for_freq(freq)
    (
        g11_before,
        g12_r,
        g12_i,
        g22_before,
        g11_after,
        g22_after,
    ) = compute_g_matrix(sor0, sor1)
    inv_g11, inv_g12_r, inv_g12_i, inv_g22 = compute_inv_g(
        g11_after, g12_r, g12_i, g22_after
    )

    # Also compute det and inv_det (divider output) for debug/verification.
    det_mul = mul_tc(g11_after, g22_after, DET_WIDTH)
    g12_r_sqr = mul_tc(g12_r, g12_r, DET_WIDTH)
    g12_i_sqr = mul_tc(g12_i, g12_i, DET_WIDTH)
    det_sub = add_tc(g12_r_sqr, g12_i_sqr, DET_WIDTH)
    det = sub_tc(det_mul, det_sub, DET_WIDTH)
    inv_det_fp = fixed_div_1_over_det(det)

    results: list[tuple[int, int]] = []

    # First row (result_row1 == 0 in hardware)
    for mic in range(MIC_NUM):
        a0 = sor0[mic]
        a1 = sor1[mic]
        r0 = mul_tc(inv_g11, int(a0.real), INV_G_WIDTH)
        r1 = mul_tc(inv_g12_r, int(a1.real), INV_G_WIDTH)
        r2 = mul_tc(inv_g12_i, int(a1.imag), INV_G_WIDTH)
        i0 = mul_tc(-inv_g11, int(a0.imag), INV_G_WIDTH)
        i1 = mul_tc(-inv_g12_r, int(a1.imag), INV_G_WIDTH)
        i2 = mul_tc(inv_g12_i, int(a1.real), INV_G_WIDTH)
        real_out = add_tc(add_tc(r0, r1, INV_G_WIDTH), r2, INV_G_WIDTH)
        imag_out = add_tc(add_tc(i0, i1, INV_G_WIDTH), i2, INV_G_WIDTH)
        results.append((real_out, imag_out))

    # Second row (result_row1 == 1 in hardware)
    for mic in range(MIC_NUM):
        a0 = sor0[mic]
        a1 = sor1[mic]
        r0 = mul_tc(inv_g12_r, int(a0.real), INV_G_WIDTH)
        r1 = mul_tc(-inv_g12_i, int(a0.imag), INV_G_WIDTH)
        r2 = mul_tc(inv_g22, int(a1.real), INV_G_WIDTH)
        i0 = mul_tc(-inv_g12_r, int(a0.imag), INV_G_WIDTH)
        i1 = mul_tc(-inv_g12_i, int(a0.real), INV_G_WIDTH)
        i2 = mul_tc(-inv_g22, int(a1.imag), INV_G_WIDTH)
        real_out = add_tc(add_tc(r0, r1, INV_G_WIDTH), r2, INV_G_WIDTH)
        imag_out = add_tc(add_tc(i0, i1, INV_G_WIDTH), i2, INV_G_WIDTH)
        results.append((real_out, imag_out))

    return {
        "sor0": sor0,
        "sor1": sor1,
        "g_before": (g11_before, g12_r, g12_i, g22_before),
        "g_after": (g11_after, g12_r, g12_i, g22_after),
        "det": {
            "det_mul": det_mul,
            "det_sub": det_sub,
            "det": det,
            "inv_det_fp": inv_det_fp,
        },
        "inv_g": (inv_g11, inv_g12_r, inv_g12_i, inv_g22),
        "outputs": results,
    }


def compute_float_pinv_for_freq(freq: int) -> np.ndarray:
    """
    Compute floating-point pinvA = inv(A^H A + lambda*I) * A^H for comparison.
    Inputs use Q-format scaling to float domain.
    """
    sor0, sor1 = build_af_for_freq(freq)
    scale_in = 2 ** INPUT_FRAC_BITS
    a0 = np.array([complex(int(x.real), int(x.imag)) for x in sor0], dtype=np.complex128) / scale_in
    a1 = np.array([complex(int(x.real), int(x.imag)) for x in sor1], dtype=np.complex128) / scale_in
    A = np.stack([a0, a1], axis=1)  # (MIC_NUM, SOR_NUM)
    G = A.conj().T @ A
    lambda_float = LAMBDA / (2 ** (2 * INPUT_FRAC_BITS))
    G_reg = G + lambda_float * np.eye(SOR_NUM, dtype=np.complex128)
    inv_g = np.linalg.inv(G_reg)
    pinv_a = inv_g @ A.conj().T  # (SOR_NUM, MIC_NUM)
    return pinv_a


def compute_float_stages_for_freq(freq: int) -> dict:
    """
    Compute floating-point stages for comparison:
    G (before/after lambda), det, inv_det, inv(G), and outputs (pinvA).
    """
    sor0, sor1 = build_af_for_freq(freq)
    scale_in = 2 ** INPUT_FRAC_BITS
    a0 = np.array([complex(int(x.real), int(x.imag)) for x in sor0], dtype=np.complex128) / scale_in
    a1 = np.array([complex(int(x.real), int(x.imag)) for x in sor1], dtype=np.complex128) / scale_in
    A = np.stack([a0, a1], axis=1)  # (MIC_NUM, SOR_NUM)
    G = A.conj().T @ A
    lambda_float = LAMBDA / (2 ** (2 * INPUT_FRAC_BITS))
    G_reg = G + lambda_float * np.eye(SOR_NUM, dtype=np.complex128)
    det = np.linalg.det(G_reg)
    inv_g = np.linalg.inv(G_reg)
    inv_det = 1.0 / det
    pinv_a = inv_g @ A.conj().T  # (SOR_NUM, MIC_NUM)
    return {
        "G_before": G,
        "G_after": G_reg,
        "det": det,
        "inv_det": inv_det,
        "inv_g": inv_g,
        "pinv_a": pinv_a,
    }


def main() -> None:
    """
    Compute and print software results for the first few freqs,
    so you can compare with RTL simulation at the corresponding addresses.
    Also writes the same output to RTF_software_output.txt.
    """
    global BRAM_REAL, BRAM_IMAG
    out_path = "./RTF_software_output.txt"
    max_freq = min(257, FREQ_NUM)

    def log(msg: str = "") -> None:
        print(msg)
        f.write(msg + "\n")

    def fmt_c(z: complex) -> str:
        return f"{z.real:+.6e}{z.imag:+.6e}j"

    now = datetime.now()
    timestamp_str = now.strftime("%Y-%m-%d %H:%M:%S")

    with open(out_path, "w", encoding="utf-8") as f:
        try:
            BRAM_REAL, BRAM_IMAG = load_bram_inputs(
                REAL_INPUT_PATH, IMAG_INPUT_PATH, TOTAL_NUM
            )
        except FileNotFoundError:
            BRAM_REAL, BRAM_IMAG = None, None
            log("[warn] Input files not found, using testbench pattern.")
        except ValueError as exc:
            BRAM_REAL, BRAM_IMAG = None, None
            log(f"[warn] {exc}; using testbench pattern.")

        f.write(f"Generated: {timestamp_str}\n")
        f.write(
            f"Input Q format: {INPUT_Q_FORMAT} (float = val / 2^{INPUT_FRAC_BITS})\n\n"
        )
        for freq in range(max_freq):
            log("=" * 60)
            log(f"[freq {freq}] Inputs (sor0, sor1):")
            info = compute_outputs_for_freq(freq)

            # Print input vectors
            for mic in range(MIC_NUM):
                a0 = info["sor0"][mic]
                a1 = info["sor1"][mic]
                log(
                    f"  mic={mic:2d}  sor0=({int(a0.real):6d}, {int(a0.imag):6d})"
                    f"  sor1=({int(a1.real):6d}, {int(a1.imag):6d})"
                )

            # Print G = A^H A (before lambda)
            g11_b, g12_r, g12_i, g22_b = info["g_before"]
            log("\n  G = A^H * A (before lambda):")
            log(f"    g11 = {g11_b:12d}")
            log(f"    g12 = ({g12_r:12d}, {g12_i:12d})")
            log(f"    g22 = {g22_b:12d}")

            # Print G + lambda*I
            g11_a, g12_r_a, g12_i_a, g22_a = info["g_after"]
            log("\n  G + lambda*I:")
            log(f"    g11 = {g11_a:12d}")
            log(f"    g12 = ({g12_r_a:12d}, {g12_i_a:12d})")
            log(f"    g22 = {g22_a:12d}")

            # Print det and divider output (inv_det) for checking
            det_info = info["det"]
            det_mul = det_info["det_mul"]
            det_sub = det_info["det_sub"]
            det = det_info["det"]
            inv_det_fp = det_info["inv_det_fp"]
            log("\n  det check (all wrapped to hardware widths):")
            log(f"    det_mul = g11*g22           = {det_mul:12d}")
            log(f"    det_sub = |g12|^2 (r^2+i^2) = {det_sub:12d}")
            log(f"    det     = det_mul - det_sub = {det:12d}")
            log(f"    inv_det (divider, Q*.{DIVOUT_F_WIDTH}) = {inv_det_fp:16d}")
            if det == 0:
                log("    [warn] det == 0 -> inv_det is forced to 0 by model")

            # Print inverse matrix elements
            inv_g11, inv_g12_r, inv_g12_i, inv_g22 = info["inv_g"]
            log("\n  inv(G) elements (fixed-point, scaled):")
            log(f"    inv_g11_real = {inv_g11:16d}")
            log(f"    inv_g12      = ({inv_g12_r:16d}, {inv_g12_i:16d})")
            log(f"    inv_g22_real = {inv_g22:16d}")

            # Print final outputs
            log("\n  Outputs (row0 then row1, 16 per freq):")
            for idx, (r, i) in enumerate(info["outputs"]):
                log(f"    out_idx={idx:2d}  real={r:16d}  imag={i:16d}")

            # Float comparison against fixed outputs (scaled back to float domain)
            if ENABLE_FLOAT_COMPARE:
                float_info = compute_float_stages_for_freq(freq)
                pinv_a = float_info["pinv_a"]
                scale_g = 2 ** (2 * INPUT_FRAC_BITS)
                scale_det = 2 ** (4 * INPUT_FRAC_BITS)
                scale_inv_det = 2 ** (4 * INPUT_FRAC_BITS - DIVOUT_F_WIDTH)
                scale_inv_g = 2 ** (DIVOUT_F_WIDTH - 2 * INPUT_FRAC_BITS)
                scale_out = 2 ** (DIVOUT_F_WIDTH - INPUT_FRAC_BITS)

                log("\n  Float vs fixed stages (float / fixed / error):")
                g_before_f = float_info["G_before"]
                g_after_f = float_info["G_after"]
                g_before_fx = np.array(
                    [
                        [complex(g11_b, 0), complex(g12_r, g12_i)],
                        [complex(g12_r, -g12_i), complex(g22_b, 0)],
                    ],
                    dtype=np.complex128,
                ) / scale_g
                g_after_fx = np.array(
                    [
                        [complex(g11_a, 0), complex(g12_r_a, g12_i_a)],
                        [complex(g12_r_a, -g12_i_a), complex(g22_a, 0)],
                    ],
                    dtype=np.complex128,
                ) / scale_g

                for r in range(2):
                    for c in range(2):
                        err = abs(g_before_f[r, c] - g_before_fx[r, c])
                        log(
                            "    G_before[%d,%d] float=%s  fixed=%s  err=%.3e"
                            % (r, c, fmt_c(g_before_f[r, c]), fmt_c(g_before_fx[r, c]), err)
                        )
                for r in range(2):
                    for c in range(2):
                        err = abs(g_after_f[r, c] - g_after_fx[r, c])
                        log(
                            "    G_after [%d,%d] float=%s  fixed=%s  err=%.3e"
                            % (r, c, fmt_c(g_after_f[r, c]), fmt_c(g_after_fx[r, c]), err)
                        )

                det_f = float_info["det"]
                det_fx = det / scale_det
                det_err = abs(det_f - det_fx)
                log(
                    "    det        float=%s  fixed=%s  err=%.3e"
                    % (fmt_c(det_f), fmt_c(det_fx), det_err)
                )

                inv_det_f = float_info["inv_det"]
                inv_det_fx = inv_det_fp * scale_inv_det
                inv_det_err = abs(inv_det_f - inv_det_fx)
                log(
                    "    inv_det    float=%s  fixed=%s  err=%.3e"
                    % (fmt_c(inv_det_f), fmt_c(inv_det_fx), inv_det_err)
                )

                inv_g_f = float_info["inv_g"]
                inv_g_fx = np.array(
                    [
                        [complex(inv_g11, 0), complex(inv_g12_r, inv_g12_i)],
                        [complex(inv_g12_r, -inv_g12_i), complex(inv_g22, 0)],
                    ],
                    dtype=np.complex128,
                ) / scale_inv_g
                for r in range(2):
                    for c in range(2):
                        err = abs(inv_g_f[r, c] - inv_g_fx[r, c])
                        log(
                            "    inv(G)[%d,%d] float=%s  fixed=%s  err=%.3e"
                            % (r, c, fmt_c(inv_g_f[r, c]), fmt_c(inv_g_fx[r, c]), err)
                        )

                log("\n  Float vs fixed comparison (pinvA):")
                for mic in range(MIC_NUM):
                    fp_row0 = pinv_a[0, mic]
                    fp_row1 = pinv_a[1, mic]
                    fx_row0 = info["outputs"][mic]
                    fx_row1 = info["outputs"][MIC_NUM + mic]
                    fx_row0_c = complex(fx_row0[0] / scale_out, fx_row0[1] / scale_out)
                    fx_row1_c = complex(fx_row1[0] / scale_out, fx_row1[1] / scale_out)
                    err0 = abs(fp_row0 - fx_row0_c)
                    err1 = abs(fp_row1 - fx_row1_c)
                    log(
                        "    mic=%2d  row0 float=%s  fixed=%s  err=%.3e"
                        "  row1 float=%s  fixed=%s  err=%.3e"
                        % (
                            mic,
                            fmt_c(fp_row0), fmt_c(fx_row0_c), err0,
                            fmt_c(fp_row1), fmt_c(fx_row1_c), err1,
                        )
                    )

    print(f"\nOutput also written to {out_path}")


if __name__ == "__main__":
    main()

