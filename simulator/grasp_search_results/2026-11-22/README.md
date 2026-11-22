# Grasp search, 2026-11-22 (simulator only)

Cross-entropy random search (`simulator/grasp_search.py`): 236 generations, 8,822 scored trials, 49 mm blocks with 6 mm / 4 deg noise.
`summary.md` / `final.json` compare the settings on fresh blocks (score 0..4). Best group ("elite mean"): lifted 11 of 13, placed 7 of 13,
mean score 3.08, against 1.33 and 1 of 21 placed for the settings used before. Small sample, and the simulated jaws are not the real jaws:
treat it as the shape of the answer and verify on the real arm. The settings are stored as profiles in
`lauzkach_eye_robot_control/grasp_profiles.json` (`current` = what was used before, `sim_elite` = this search).
The full per-trial log (22,000+ lines) is not committed; rerun `python -m simulator.grasp_search` to regenerate.
