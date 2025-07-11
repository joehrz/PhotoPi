#!/usr/bin/env python3
# PhotoPi/run_tests.py
"""
Test runner script for PhotoPi project.
Provides convenient testing commands and coverage reporting.
"""

import sys
import subprocess
import argparse
from pathlib import Path


def run_command(cmd, description=""):
    """Run a command and handle errors."""
    if description:
        print(f"\n{description}")
        print("-" * 50)
    
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    
    if result.returncode != 0:
        print(f"Command failed with return code {result.returncode}")
        sys.exit(result.returncode)
    
    return result


def main():
    parser = argparse.ArgumentParser(description="PhotoPi test runner")
    parser.add_argument(
        "--coverage", 
        action="store_true", 
        help="Run tests with coverage reporting"
    )
    parser.add_argument(
        "--unit-only", 
        action="store_true", 
        help="Run only unit tests (exclude integration/hardware tests)"
    )
    parser.add_argument(
        "--verbose", "-v", 
        action="store_true", 
        help="Verbose output"
    )
    parser.add_argument(
        "--fast", 
        action="store_true", 
        help="Skip slow tests"
    )
    parser.add_argument(
        "--pattern", 
        type=str, 
        help="Run tests matching pattern"
    )
    
    args = parser.parse_args()
    
    # Base pytest command
    cmd = ["python", "-m", "pytest"]
    
    # Add test directory
    cmd.append("tests/")
    
    # Add coverage if requested
    if args.coverage:
        cmd.extend([
            "--cov=photopack",
            "--cov-report=html",
            "--cov-report=term-missing",
            "--cov-branch"
        ])
    
    # Add verbosity
    if args.verbose:
        cmd.append("-v")
    
    # Filter tests
    if args.unit_only:
        cmd.extend(["-m", "not integration and not hardware"])
    
    if args.fast:
        cmd.extend(["-m", "not slow"])
    
    if args.pattern:
        cmd.extend(["-k", args.pattern])
    
    # Run tests
    run_command(cmd, "Running PhotoPi Tests")
    
    # If coverage was requested, show where the report is
    if args.coverage:
        print("\nCoverage report generated in htmlcov/index.html")


if __name__ == "__main__":
    main()