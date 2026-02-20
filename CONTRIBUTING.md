# Contributing to Aqua Keeper AI

Thank you for considering contributing to Aqua Keeper AI! This document explains the development workflow, coding standards, and how to submit changes.

---

## Table of Contents

1. [Development Setup](#development-setup)
2. [Project Layout](#project-layout)
3. [Coding Standards](#coding-standards)
4. [Testing](#testing)
5. [Pull Request Process](#pull-request-process)
6. [Issue Reporting](#issue-reporting)

---

## Development Setup

```bash
# Clone the repository
git clone https://github.com/your-org/aqua-keeper-ai.git
cd aqua-keeper-ai

# Create a virtual environment
python -m venv .venv
source .venv/bin/activate      # Linux / macOS
# .venv\Scripts\activate       # Windows

# Install dependencies
pip install -r requirements.txt

# Verify installation
python -m pytest tests/ -v
python scripts/diagnostics.py
```

### Requirements

- Python 3.9 or later
- A POSIX-compatible operating system (Linux recommended; macOS for development)
- Hardware dependencies (RPi.GPIO, adafruit libraries) are optional — the codebase runs with the `mock` backend on any platform

---

## Project Layout

```
aqua-keeper-ai/
├── configs/              # YAML configuration files
├── diagrams/             # Architecture and design diagrams
├── scripts/              # CLI tools (training, simulation, diagnostics)
├── src/
│   ├── control/          # Controller, actuator, coverage strategy
│   ├── pipeline/         # End-to-end main loop
│   ├── utils/            # Config loader, logger, benchmark profiler
│   └── vision/           # Detector, calibration, Kalman tracker
├── tests/
│   ├── unit/             # Fast unit tests (no hardware required)
│   └── integration/      # Integration tests (synthetic data)
├── ARCHITECTURE.md       # System design and diagrams
├── CONTROL.md            # Control system details
├── CONTRIBUTING.md       # This file
├── HARDWARE.md           # BOM, assembly, wiring
├── OPERATIONS.md         # Deployment and monitoring
├── TESTING.md            # Testing strategy and procedures
├── VISION.md             # Vision pipeline and training
└── README.md             # Project overview and quick start
```

---

## Coding Standards

### Python Style

- Follow **PEP 8** with a maximum line length of 100 characters.
- Use **type hints** for all function signatures.
- Use **docstrings** (Google or NumPy style) for public classes and methods.
- Import order: stdlib → third-party → local (separated by blank lines).

### Naming

- Modules and packages: `snake_case`
- Classes: `PascalCase`
- Functions and variables: `snake_case`
- Constants: `UPPER_SNAKE_CASE`

### File Headers

Every Python source file should start with a module docstring describing its purpose.

### Commits

- Use present-tense imperative mood: "Add Kalman tracker", not "Added Kalman tracker".
- Keep the first line under 72 characters.
- Reference issue numbers where applicable: "Fix #42 — correct PID anti-windup".

---

## Testing

### Running Tests

```bash
# All tests
python -m pytest tests/ -v

# Unit tests only
python -m pytest tests/unit/ -v

# Integration tests only
python -m pytest tests/integration/ -v

# With coverage (if pytest-cov is installed)
python -m pytest tests/ --cov=src --cov-report=term-missing
```

### Writing Tests

- Place unit tests in `tests/unit/test_<module>.py`.
- Place integration tests in `tests/integration/test_<feature>.py`.
- Use descriptive test names: `test_velocity_limiting`, not `test_1`.
- Tests must not require hardware — use the `mock` backend for motor tests.
- Keep tests fast (< 1 s each; the full suite should run in < 5 s).

### Test Structure

```python
class TestClassName:
    def test_specific_behaviour(self):
        # Arrange
        obj = ClassName(param=value)
        # Act
        result = obj.method(input)
        # Assert
        assert result == expected
```

---

## Pull Request Process

1. **Fork** the repository and create a feature branch from `main`.
2. Make your changes with clear, atomic commits.
3. Ensure all tests pass: `python -m pytest tests/ -v`
4. Update documentation if your change affects public APIs or configuration.
5. Submit a pull request with:
   - A clear title and description.
   - Reference to any related issue.
   - A summary of what changed and why.
6. Address review feedback promptly.

---

## Issue Reporting

When reporting a bug, please include:

1. **Platform** — hardware (Pi 4, Jetson Nano, etc.) and OS version.
2. **Python version** — output of `python --version`.
3. **Steps to reproduce** — minimal commands to trigger the issue.
4. **Expected behaviour** — what should happen.
5. **Actual behaviour** — what actually happens (include error messages / logs).
6. **Configuration** — relevant sections of `configs/default.yaml` (redact secrets).

For feature requests, describe the use case and the expected outcome.
