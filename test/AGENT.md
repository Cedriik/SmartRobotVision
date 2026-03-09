# AGENT.md

## Goal
Build the program step-by-step in ~/Robot/test with reliable rollback and change tracking.

## Rules
1. Before editing an existing code file, copy the current version to ~/Robot/Rollback/.
2. Keep active development files in ~/Robot/test unless instructed otherwise.
3. After every build/change session, update ~/Robot/CONTINUITY.md.
4. Keep entries short: date-time, files changed, summary, rollback note.

## Rollback Quick Steps
- List backups: ls -la ~/Robot/Rollback
- Restore file: cp ~/Robot/Rollback/<filename> ~/Robot/test/<filename>
- Verify: python3 ~/Robot/test/<filename>
