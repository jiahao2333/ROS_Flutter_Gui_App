import sys
import os
import unittest
from unittest.mock import MagicMock, patch
import asyncio

# 如果不存在则 Mock websockets
sys.modules['websockets'] = MagicMock()
sys.modules['websockets.exceptions'] = MagicMock()

# 现在导入 backend
# 将当前目录添加到路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import backend

class TestROS2Backend(unittest.IsolatedAsyncioTestCase):
    
    def setUp(self):
        backend.bg_processes = {
            "slam": None,
            "navigation": None,
            "rosbridge": None
        }
        # Mock shutil.which 返回 False (模拟未安装 ros2，或使用环境变量)
        # 我们设置环境变量 MOCK_ROS2 以允许命令“成功”
        os.environ["MOCK_ROS2"] = "1"

    @patch('backend.subprocess.Popen')
    @patch('backend.os.killpg')
    async def test_start_mapping(self, mock_kill, mock_popen):
        # 设置 mock 进程
        mock_proc = MagicMock()
        mock_proc.pid = 12345
        mock_popen.return_value = mock_proc

        success, msg = await backend.handle_start_mapping()
        
        self.assertTrue(success)
        self.assertEqual(msg, "success")
        
        # 验证 2 次启动
        self.assertEqual(mock_popen.call_count, 2)
        args_list = [call.args[0] for call in mock_popen.call_args_list]
        self.assertTrue(any("rviz_slam.launch.py" in cmd for cmd in args_list))
        self.assertTrue(any("rosbridge_websocket_launch.xml" in cmd for cmd in args_list))

    @patch('backend.subprocess.Popen')
    async def test_start_navigation(self, mock_popen):
        mock_proc = MagicMock()
        mock_proc.pid = 12345
        mock_popen.return_value = mock_proc

        success, msg = await backend.handle_start_navigation("my_map")
        
        self.assertTrue(success)
        self.assertTrue(any("navigation.launch.py" in x and 'map:="my_map"' in x for x in [c[0][0] for c in mock_popen.call_args_list]))

    @patch('backend.subprocess.run')
    async def test_save_map(self, mock_run):
        mock_run.return_value.returncode = 0
        
        success, msg = await backend.handle_save_map("new_map")
        
        self.assertTrue(success)
        # 检查命令
        cmd = mock_run.call_args[0][0]
        self.assertIn("map_saver_cli", cmd)
        self.assertIn('-f "new_map"', cmd)

    @patch('backend.os.listdir')
    @patch('backend.os.path.exists')
    async def test_get_maps(self, mock_exists, mock_listdir):
        mock_exists.return_value = True
        mock_listdir.return_value = ["map_1.pgm", "map_1.yaml", "house.png", "office.yaml"]
        
        success, maps, msg = await backend.handle_get_maps()
        
        self.assertTrue(success)
        self.assertIn("map_1", maps)
        self.assertIn("office", maps)
        # house.png 没有 .yaml 或 .pgm 扩展名
        # 我们的代码只做了 splitext 检查
        # 验证代码逻辑: "if ext in ['.yaml', '.pgm']"
        # 所以 house.png (.png) 应该被排除
        self.assertNotIn("house", maps)

if __name__ == '__main__':
    unittest.main()
