# OSGAR Development Mandates

When debugging or implementing features based on robot logs, ALWAYS follow this "Standard Development Procedure":

1.  **Local Replay Verification:** Run `osgar.replay` on the robot's log locally. Verify that the behavior (outputs/prints) matches what was seen on the robot. This confirms the environment is correct and the issue is reproducible.
2.  **Analysis & Debugging:** Add extra debug prints or visualizations to the code to pinpoint the root cause.
3.  **Implementation:** Apply the fix or new logic.
4.  **Verification of Change (Expect Failure):** Run `osgar.replay` *without* the `-F` (force) flag. The replay **SHOULD FAIL** (or show significant "unexpected input/output" differences) because the robot's behavior has changed. If it doesn't fail, investigate why the code change isn't active or why behavior stayed the same.
5.  **Force Replay for Validation:** Run `osgar.replay -F` to process the old log's inputs through the *new* logic and validate that the updated behavior matches the desired solution.
