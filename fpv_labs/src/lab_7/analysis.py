"""Lab 7: flight indicators and plots from the QGroundControl logs."""

import sys
from pathlib import Path

from tools.analysis import analyze

# QGC: Application Load Save Path -> <repo>/logs/lab_7
LOG_DIRECTORY = Path(__file__).resolve().parents[3] / "logs" / "lab_7"

if __name__ == "__main__":
    sys.exit(analyze(LOG_DIRECTORY))
