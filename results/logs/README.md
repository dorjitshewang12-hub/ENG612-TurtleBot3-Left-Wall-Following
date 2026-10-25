# Real-robot run logs (TurtleBot3 Burger, 25 Oct 2026)

Terminal output from each run on the lab arena, in the order they were run.

| File | Controller | What happened |
|---|---|---|
| 01_baseline_first_real_run.txt | Baseline | First real run. Tracks 0.27-0.35 m, 4 corners. Ctrl-C stop failed (traceback). |
| 02_dual_v1_filter_lag.txt | Dual (early) | Heavy filtering and slew limit caused lag: 0.36 m after corners, overshoot to 0.25 m. |
| 03_dual_v2_arcing_into_wall.txt | Dual + speed PID | Speed PID added. Robot arced into the left wall after corners (single side beam + higher Kp). |
| 04_dual_v3_two_beam_fixed.txt | Dual + two-beam fix | Two-beam wall angle estimate and front trigger 0.35 m. Tracks 0.28-0.34 m. Run with kp:=1.2. |
| 05_dual_v3_default_kp_1.2.txt | Dual | Same as 04 with kp = 1.2 as the default. Confirms repeatable behaviour. |
| 06_baseline_ctrl_c_fixed.txt | Baseline | Ctrl-C fix confirmed (clean exit). Tracks 0.28-0.32 m. |
| 07_baseline_final_with_error.txt | Baseline | Final comparison run, error logged. RMS 1.28 cm. |
| 08_dual_final_with_error.txt | Dual | Final comparison run, error logged. RMS 1.37 cm. |

Analysis of runs 07 and 08: `../wall_following_results.xlsx`.
