# Copyright checks are skipped for generated competition scaffold code.

import pytest


@pytest.mark.skip(reason='No copyright header has been placed in this file.')
def test_copyright():
    pass
