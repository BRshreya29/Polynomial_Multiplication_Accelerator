# Polynomial_Multiplication_Accelerator  

**Target FPGA:** Xilinx **Artix‑7 XC7A100T**  
**Toolchain:** **Vivado 2023.2** (RTL design, simulation, synthesis, implementation & timing analysis)  
**Scheduling:** **Finite State Machine (FSM)** drives *pre-processing → NTT → pointwise mult → INTT → PWM serialization* pipeline.

---

## 1) Description

This project implements a polynomial multiplication datapath (typical for lattice/PQC style flows) on an Artix‑7 FPGA. A centralized FSM (`top_polynomial_fsm`) orchestrates:

1. **Pre-load** external inputs  
2. **Forward NTT** across 4 stages  
3. **Re-load** internal results  
4. **Inverse NTT (INTT)**  
5. **Parallel‑In Serial‑Out (PISO) PWM** output

The design was written, simulated, synthesized and timing-closed in **Vivado 2023.2**. The README explains how to build, simulate, constrain, and probe the design, and documents the FSM, interfaces, and key signals.

---

## 2) FSM Overview

### States

| Name        | Code | Purpose                                                                 | Exit Condition / Transition Trigger                      |
|-------------|------|-------------------------------------------------------------------------|----------------------------------------------------------|
| `IDLE`      | 0    | Reset/hold                                                              | Go to `PRE_LOAD`                                         |
| `PRE_LOAD`  | 1    | Load external `in1`, `in2` into PPU                                     | `full == 1` → `NTT_0`                                    |
| `NTT_0`     | 2    | Forward NTT stage 0                                                     | `counter == 112` → `NTT_1`                               |
| `NTT_1`     | 3    | Forward NTT stage 1                                                     | `counter == 112` → `NTT_2`                               |
| `NTT_2`     | 4    | Forward NTT stage 2                                                     | `counter == 112` → `NTT_3`                               |
| `NTT_3`     | 5    | Forward NTT stage 3                                                     | `counter == 112 && sendr` → `PRE_LOAD_2`                 |
| `PRE_LOAD_2`| 6    | Re-load internal `he/ho` into PPU, switch to INTT mode                  | `sendr == 1` → `INTT_0`                                  |
| `INTT_0`    | 7    | Inverse NTT stage 0                                                     | `counter == 96` → `INTT_1`                               |
| `INTT_1`    | 8    | Inverse NTT stage 1 + final serialization (PWM)                         | `send == 1` → `DONE`                                     |
| `DONE`      | 9    | Final output valid, assert `done`                                       | Stay in `DONE`                                           |

---

## 3) I/O (top_polynomial_fsm)

```verilog
module top_polynomial_fsm (
    input  clk,
    input  rst,
    input  int,              // user-controlled input enable
    input  [11:0] in1, in2,  // external polynomial samples
    output [11:0] final_output,
    output done
);
```

**Clock/Reset**  
```verilog
// Clock: assumed 100 MHz on Artix-7 (edit XDC if different)
input clk;

// Reset: async posedge reset to FSM and submodules
input rst;
```
**Data**
```verilog
// External polynomial inputs consumed during PRE_LOAD
input [11:0] in1, in2;

// Final serialized output from piso_pwm after INTT
output [11:0] final_output;

// Asserted high when the entire pipeline completes
output done;
```
---

## 4) Summary 

This project was developed in interest of research in Post-Quantum Cryptographic Algorithm ML-KEM.
