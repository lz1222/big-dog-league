# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
from pathlib import Path
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # 中文文档字符串采用中文标点；这些规则只接受英文句末标点/祈使语气，
    # 会把合规的中文说明误报为失败。显式保留 ament 的缺失文档默认忽略项。
    package_root = Path(__file__).resolve().parents[1]
    rc = main(argv=[
        str(package_root), 'test',
        '--ignore=D100,D101,D102,D103,D104,D105,D107,D203,D212,D213,D400,D401,D403,D415',
    ])
    assert rc == 0, 'Found code style errors / warnings'
