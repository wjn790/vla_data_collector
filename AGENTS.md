# SVT Policy Client Invariant

Every real-robot LingBot-VLA inference YAML must pin the WJN deployment contract
ID, exact model path, robot config, normalization SHA-256, robot-config SHA-256,
and training-config SHA-256. The executor must compare all values with server
metadata before publishing a policy action. A missing or mismatched value is a
hard startup failure. Never update only the checkpoint or only the robot config.
