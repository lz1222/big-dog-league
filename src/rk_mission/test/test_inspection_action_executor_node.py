#!/usr/bin/env python3

"""Foxy B1 修复回归测试：inspector_action_executor_node 无 _rclpy_pybind11 导入。"""

import importlib
import sys
import unittest


# ---- 测试目标：模块级兼容元组 ----

class InspectionActionExecutorImportTest(unittest.TestCase):
    """验证 B1 修复后模块在有无 _rclpy_pybind11 的情况下都能成功导入。"""

    def test_module_imports_without_pybind11_module(self):
        """Foxy 上 _rclpy_pybind11 不存在，模块仍必须可导入。"""
        self.assertNotIn(
            'rclpy._rclpy_pybind11', sys.modules,
            'test precondition: _rclpy_pybind11 must be absent on Foxy',
        )
        try:
            import rk_mission.inspection_action_executor_node as mod
        except (ImportError, ModuleNotFoundError) as error:
            self.fail(
                'inspection_action_executor_node must import without '
                'rclpy._rclpy_pybind11 on Foxy: {}'.format(error)
            )
        self.assertTrue(hasattr(mod, '_SHUTDOWN_SIGNALS'))

    def test_shutdown_signals_is_tuple(self):
        """_SHUTDOWN_SIGNALS 必须是 tuple 以支持 + 操作符。"""
        import rk_mission.inspection_action_executor_node as mod
        self.assertIsInstance(mod._SHUTDOWN_SIGNALS, tuple)

    def test_shutdown_signals_does_not_contain_business_exceptions(self):
        """关闭信号元组不应包含普通 RuntimeError 或 Exception。"""
        import rk_mission.inspection_action_executor_node as mod
        self.assertNotIn(RuntimeError, mod._SHUTDOWN_SIGNALS)
        self.assertNotIn(Exception, mod._SHUTDOWN_SIGNALS)
        self.assertNotIn(ValueError, mod._SHUTDOWN_SIGNALS)

    def test_shutdown_signals_allow_keyboard_interrupt_combination(self):
        """验证 _SHUTDOWN_SIGNALS 可与 KeyboardInterrupt 正确组合。"""
        import rk_mission.inspection_action_executor_node as mod
        combined = (KeyboardInterrupt,) + mod._SHUTDOWN_SIGNALS
        self.assertIsInstance(combined, tuple)
        self.assertIn(KeyboardInterrupt, combined)

    def test_reimport_is_idempotent(self):
        """重复导入不改变 _SHUTDOWN_SIGNALS 的值。"""
        import rk_mission.inspection_action_executor_node as mod
        first = mod._SHUTDOWN_SIGNALS
        importlib.reload(mod)
        second = mod._SHUTDOWN_SIGNALS
        self.assertEqual(first, second)

    def test_rclpy_pybind11_not_in_sys_modules_after_import(self):
        """导入后 _rclpy_pybind11 不应被意外注册到 sys.modules。"""
        import rk_mission.inspection_action_executor_node  # noqa: F401,F811
        self.assertNotIn(
            'rclpy._rclpy_pybind11', sys.modules,
            'rclpy._rclpy_pybind11 should not be present in sys.modules on Foxy',
        )


if __name__ == '__main__':
    unittest.main()
