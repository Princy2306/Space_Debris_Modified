import subprocess


def run_program(inputs):
    result = subprocess.run(
        ["./space_debris"],
        input=inputs,
        text=True,
        capture_output=True
    )
    return result.stdout


def test_program_runs():
    output = run_program("0\n")
    assert "DYNAMIC SPACE DEBRIS COLLISION DETECTION" in output
    assert "Goodbye." in output


def test_risk_thresholds():
    output = run_program("2\n0\n")
    assert "CRITICAL" in output
    assert "HIGH" in output
    assert "MEDIUM" in output
    assert "LOW" in output


def test_builtin_scenario():
    output = run_program("3\n1\n0\n")
    assert "Normal route" in output


if __name__ == "__main__":
    test_program_runs()
    test_risk_thresholds()
    test_builtin_scenario()
    print("All tests passed!")