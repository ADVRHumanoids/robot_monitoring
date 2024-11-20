import pytest

def pytest_addoption(parser):
    parser.addoption("--yaml_file", action="store", default="ecat_repl/test/test_repl.yaml")

@pytest.fixture
def yaml_file(request):
    return request.config.getoption("--yaml_file")
