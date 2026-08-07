#!/bin/bash
# 正式关闭链共用的 estop CLI 诊断：保留失败即失败的安全语义，记录每一步证据。

rk_estop_now_ms() {
    # Python 3.8 可用，避免依赖不同系统对 date %N 的实现差异。
    python3 -c 'import time; print(int(time.time() * 1000))'
}

rk_estop_log_stream() {
    local stage="$1"
    local operation="$2"
    local stream_name="$3"
    local stream_file="$4"
    local line

    if [ ! -s "$stream_file" ]; then
        printf 'ESTOP_DIAG stage=%s operation=%s %s=<empty>\n' \
            "$stage" "$operation" "$stream_name"
        return 0
    fi
    while IFS= read -r line || [ -n "$line" ]; do
        printf 'ESTOP_DIAG stage=%s operation=%s %s=%s\n' \
            "$stage" "$operation" "$stream_name" "$line"
    done < "$stream_file"
}

rk_estop_response_classification() {
    # CLI 输出并非稳定 JSON；仅接受明确的 success 布尔字段，避免误判成功。
    python3 - "$1" <<'PY'
import re
import sys

with open(sys.argv[1], 'r', encoding='utf-8', errors='replace') as stream:
    response = stream.read()
matches = re.findall(r'(?im)\bsuccess\s*(?:=|:)\s*(true|false)\b', response)
if not matches:
    print('MALFORMED_RESPONSE')
elif matches[-1].lower() == 'true':
    print('SUCCESS')
else:
    print('SERVICE_REJECTED')
PY
}

rk_call_mux_estop() {
    # 统一记录 service type 与 SetBool 调用，调用方仅在 SUCCESS 时继续关闭链。
    local stage="$1"
    local service_name="${ESTOP_SERVICE:-/safety/estop}"
    local start_ms
    local end_ms
    local operation_start_ms
    local operation_end_ms
    local type_stdout
    local type_stderr
    local call_stdout
    local call_stderr
    local type_exit
    local call_exit
    local service_type
    local classification

    start_ms="$(rk_estop_now_ms)"
    type_stdout="$(mktemp "${TMPDIR:-/tmp}/rk_estop_type_stdout.XXXXXX")" || return 1
    type_stderr="$(mktemp "${TMPDIR:-/tmp}/rk_estop_type_stderr.XXXXXX")" || {
        rm -f "$type_stdout"
        return 1
    }
    call_stdout="$(mktemp "${TMPDIR:-/tmp}/rk_estop_call_stdout.XXXXXX")" || {
        rm -f "$type_stdout" "$type_stderr"
        return 1
    }
    call_stderr="$(mktemp "${TMPDIR:-/tmp}/rk_estop_call_stderr.XXXXXX")" || {
        rm -f "$type_stdout" "$type_stderr" "$call_stdout"
        return 1
    }

    operation_start_ms="$(rk_estop_now_ms)"
    timeout 3s ros2 service type "$service_name" >"$type_stdout" 2>"$type_stderr"
    type_exit=$?
    operation_end_ms="$(rk_estop_now_ms)"
    printf 'ESTOP_DIAG stage=%s service=%s operation=service_type start_ms=%s end_ms=%s duration_ms=%s exit_code=%s\n' \
        "$stage" "$service_name" "$operation_start_ms" "$operation_end_ms" \
        "$((operation_end_ms - operation_start_ms))" "$type_exit"
    rk_estop_log_stream "$stage" service_type stdout "$type_stdout"
    rk_estop_log_stream "$stage" service_type stderr "$type_stderr"

    service_type="$(cat "$type_stdout")"
    if [ "$type_exit" -eq 124 ]; then
        classification=TIMEOUT
    elif [ "$type_exit" -ne 0 ]; then
        classification=CLI_ERROR
    elif [ "$service_type" != 'std_srvs/srv/SetBool' ]; then
        classification=SERVICE_TYPE_UNAVAILABLE
    else
        classification=''
    fi
    if [ -n "$classification" ]; then
        end_ms="$(rk_estop_now_ms)"
        printf 'ESTOP_DIAG stage=%s service=%s start_ms=%s end_ms=%s duration_ms=%s classification=%s\n' \
            "$stage" "$service_name" "$start_ms" "$end_ms" \
            "$((end_ms - start_ms))" "$classification"
        rm -f "$type_stdout" "$type_stderr" "$call_stdout" "$call_stderr"
        return 1
    fi

    operation_start_ms="$(rk_estop_now_ms)"
    timeout 5s ros2 service call "$service_name" std_srvs/srv/SetBool \
        '{data: true}' >"$call_stdout" 2>"$call_stderr"
    call_exit=$?
    operation_end_ms="$(rk_estop_now_ms)"
    printf 'ESTOP_DIAG stage=%s service=%s operation=service_call start_ms=%s end_ms=%s duration_ms=%s exit_code=%s\n' \
        "$stage" "$service_name" "$operation_start_ms" "$operation_end_ms" \
        "$((operation_end_ms - operation_start_ms))" "$call_exit"
    rk_estop_log_stream "$stage" service_call stdout "$call_stdout"
    rk_estop_log_stream "$stage" service_call stderr "$call_stderr"

    if [ "$call_exit" -eq 124 ]; then
        classification=TIMEOUT
    elif [ "$call_exit" -ne 0 ]; then
        classification=CLI_ERROR
    else
        classification="$(rk_estop_response_classification "$call_stdout")"
    fi
    end_ms="$(rk_estop_now_ms)"
    printf 'ESTOP_DIAG stage=%s service=%s start_ms=%s end_ms=%s duration_ms=%s classification=%s\n' \
        "$stage" "$service_name" "$start_ms" "$end_ms" \
        "$((end_ms - start_ms))" "$classification"
    rm -f "$type_stdout" "$type_stderr" "$call_stdout" "$call_stderr"
    [ "$classification" = SUCCESS ]
}
