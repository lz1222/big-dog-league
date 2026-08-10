"""被动 Sport API 观察器的无硬件安全回归测试。"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_api_observer_all_filter_remains_passive():
    """api_id=0 仅放宽订阅过滤，不得引入任何 SDK 控制对象。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_sport_api_observer.cpp').read_text(
        encoding='utf-8'
    )
    assert 'observed_api_id_ != 0 && identity.api_id() != observed_api_id_' in source
    assert 'const int64_t api_id = argc >= 4 ? std::stoll(argv[3]) : 0;' in source
    assert 'api_id; 0=all' in source
    assert 'SportClient' not in source
    assert 'ChannelPublisher' not in source


def test_api_observer_marks_all_filter_explicitly():
    """汇总输出必须区分全量观察与具体 API，避免证据解释歧义。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_sport_api_observer.cpp').read_text(
        encoding='utf-8'
    )
    assert '" api_filter="' in source
    assert 'std::cout << "ALL"' in source


def test_api_observer_accepts_only_safe_service_names():
    """服务名只能决定订阅路径，不能允许任意 topic 注入。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_sport_api_observer.cpp').read_text(
        encoding='utf-8'
    )
    assert 'bool IsSafeServiceName' in source
    assert "std::isalnum(character) != 0 || character == '_'" in source
    assert 'ApiTopic(service_name, "request")' in source
    assert 'ApiTopic(service_name, "response")' in source
