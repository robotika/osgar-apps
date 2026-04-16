# Standard Development Procedure for OSGAR Apps

This project follows a strict verification loop when using log data for development:

1.  **Replay Verification**: Verify local replay matches robot behavior.
2.  **Debug**: Analyze via prints/visuals.
3.  **Implement**: Apply code changes.
4.  **Verify Change**: Run `osgar.replay` (expect failure/difference).
5.  **Force Validate**: Run `osgar.replay -F` to see new behavior on old inputs.
