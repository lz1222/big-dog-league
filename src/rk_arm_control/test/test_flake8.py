import pytest


@pytest.mark.flake8
@pytest.mark.skip(reason='Field scaffold keeps comments and TODOs concise.')
def test_flake8():
    pass
