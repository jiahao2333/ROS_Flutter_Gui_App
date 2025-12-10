import asyncio
import json
import logging
import os
import subprocess
import signal
import sys
import websockets
import shutil

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 常量
PORT = 8999
MAPS_DIR = os.path.expanduser("~/ros2_ws/src/slam/maps")

# 全局进程句柄
bg_processes = {
    "slam": None,
    "navigation": None,
    "rosbridge": None,
    "web_video_server": None

}

def kill_process(name):
    """如果进程存在，则终止后台进程。"""
    proc = bg_processes.get(name)
    if proc:
        logger.info(f"正在停止 {name} 进程 (PID: {proc.pid})...")
        try:
            # 发送 SIGINT (Ctrl+C) 到进程组，以确保子进程（如 launch 文件启动的进程）也被终止
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning(f"{name} 未能优雅退出，正在使用 SIGKILL 强制终止。")
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass # 进程已结束
        bg_processes[name] = None
    else:
        logger.debug(f"没有运行中的 {name} 进程需要停止。")

def run_stop_script():
    """运行 ~/.stop_ros.sh 脚本以清理 ROS 环境"""
    script_path = os.path.expanduser("~/.stop_ros.sh")
    if os.path.exists(script_path):
        logger.info(f"正在执行停止脚本: {script_path}")
        try:
            # 使用 bash 执行脚本
            subprocess.run(f"bash {script_path}", shell=True)
        except Exception as e:
            logger.error(f"执行停止脚本失败: {e}")
    else:
        logger.warning(f"未找到停止脚本: {script_path}")

def cleanup_all():
    """停止所有运行中的后台进程。"""
    logger.info("正在清理所有进程...")
    
    # 首先执行停止脚本
    run_stop_script()
    
    for name in list(bg_processes.keys()):
        kill_process(name)

def run_command_bg(command, name):
    """在后台运行命令并存储进程句柄。"""
    kill_process(name) # 确保之前的实例已停止
    
    logger.info(f"正在启动 {name}: {command}")
    
    # 检查 ROS2 环境或是否为 dry run
    if not shutil.which("ros2") and not os.environ.get("MOCK_ROS2"):
         logger.warning("未找到 ROS2。命令本应为: " + command)
         # 如果在非 ROS 系统上且没有 MOCK_ROS2 环境变量，我们可能会失败或仅记录日志。
         # 但用户要求代码在小车上运行。
         # 我们假设它在小车上运行，但在 preexec_fn 中使用 os.setsid 以便进行进程组管理。
    
    try:
        # 使用 shell=True 直接解释命令字符串
        # preexec_fn=os.setsid 对于向整个进程组发送信号至关重要（launch 文件会生成子进程）
        proc = subprocess.Popen(
            command, 
            shell=True, 
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL, # 或者 subprocess.PIPE 如果需要记录输出
            stderr=subprocess.DEVNULL
        )
        bg_processes[name] = proc
        logger.info(f"已启动 {name}，PID 为 {proc.pid}")
        return True
    except Exception as e:
        logger.error(f"启动 {name} 失败: {e}")
        return False

async def handle_start_mapping():
    cleanup_all()
    
    # 1. 启动 SLAM
    cmd_slam = "ros2 launch slam slam.launch.py"
    if not run_command_bg(cmd_slam, "slam"):
        return False, "启动 SLAM 失败"
        
    # 2. 启动 ROSBridge
    cmd_bridge = "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    if not run_command_bg(cmd_bridge, "rosbridge"):
        return False, "启动 ROSBridge 失败"
    
    # 3。启动web_video_server
    cmd_bridge = "ros2 run web_video_server web_video_server"
    if not run_command_bg(cmd_bridge, "web_video_server"):
        return False, "启动 web_video_server 失败"
        
    return True, "success"

async def handle_start_navigation(map_name):
    if not map_name:
        return False, "导航需要指定 map_name"
        
    cleanup_all()
    
    # 1. 启动导航
    cmd_nav = f'ros2 launch navigation navigation.launch.py map:="{map_name}"'
    if not run_command_bg(cmd_nav, "navigation"):
        return False, "启动导航失败"
        
    # 2. 启动 ROSBridge
    cmd_bridge = "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    if not run_command_bg(cmd_bridge, "rosbridge"):
        return False, "启动 ROSBridge 失败"
    
    # 3。启动web_video_server
    cmd_bridge = "ros2 run web_video_server web_video_server"
    if not run_command_bg(cmd_bridge, "web_video_server"):
        return False, "启动 web_video_server 失败"

    return True, "success"

async def handle_save_map(map_name):
    if not map_name:
        return False, "保存地图需要指定 map_name"
        
    # 确保目录存在（可选，但推荐）
    # os.makedirs(MAPS_DIR, exist_ok=True) 
    
    cmd_save = f'cd {MAPS_DIR} && ros2 run nav2_map_server map_saver_cli -f "{map_name}" --ros-args -p map_subscribe_transient_local:=true'
    
    logger.info(f"正在保存地图: {cmd_save}")
    
    try:
        # 同步运行
        process = subprocess.run(cmd_save, shell=True, check=True)
        return True, "map saved successfully"
    except subprocess.CalledProcessError as e:
        logger.error(f"地图保存失败: {e}")
        return False, f"保存地图失败: 命令退出码 {e.returncode}"
    except Exception as e:
         logger.error(f"地图保存错误: {e}")
         return False, f"保存地图失败: {str(e)}"

async def handle_get_maps():
    try:
        # 检查目录是否存在
        if not os.path.exists(MAPS_DIR):
             return True, [], "未找到地图目录，返回空列表"

        files = os.listdir(MAPS_DIR)
        maps = set()
        for f in files:
            # 以 map_ 开头？用户说 "map_0.1.pgm", "map_home.yaml"
            # 逻辑：只取文件名，不包含后缀。
            # 过滤逻辑：用户说："只需要取map_0.1 map_home这些 不包含后缀"
            # 且存在 "map_0.1.pgm" 和 "map_0.1.yaml"。
            
            # 简单方法：splitext
            name, ext = os.path.splitext(f)
            # 是否过滤非地图文件？目前假设所有有效文件都是地图相关的。
            # 也可以通过扩展名过滤以策安全。
            if ext in ['.yaml', '.pgm']:
                maps.add(name)
        
        return True, list(maps), "success"
    except Exception as e:
        logger.error(f"获取地图列表错误: {e}")
        return False, [], f"获取地图列表错误: {str(e)}"

async def handler(websocket): # 为了兼容较新版本的 websockets 移除了 'path' 参数，或者直接使用 'websocket'
    remote_ip = websocket.remote_address[0]
    logger.info(f"来自 {remote_ip} 的新连接")
    
    try:
        async for message in websocket:
            logger.info(f"收到消息: {message}")
            try:
                data = json.loads(message)
                command = data.get("command")
                
                response = {"code": 1, "message": "未知指令"}
                
                if command == "start_mapping":
                    success, msg = await handle_start_mapping()
                    response = {"code": 0 if success else 1, "message": msg}
                    
                elif command == "start_navigation":
                    map_name = data.get("map_name")
                    success, msg = await handle_start_navigation(map_name)
                    response = {"code": 0 if success else 1, "message": msg}
                    
                elif command == "save_map":
                    map_name = data.get("map_name")
                    success, msg = await handle_save_map(map_name)
                    response = {"code": 0 if success else 1, "message": msg}
                    
                elif command == "get_maps":
                    success, maps, msg = await handle_get_maps()
                    if success:
                         response = {"code": 0, "message": msg, "maps": maps}
                    else:
                         response = {"code": 1, "message": msg}

                elif command == "stop_all":
                    cleanup_all()
                    response = {"code": 0, "message": "所有进程已停止"}
                
                await websocket.send(json.dumps(response))
                
            except json.JSONDecodeError:
                await websocket.send(json.dumps({"code": 1, "message": "无效的 JSON"}))
            except Exception as e:
                logger.error(f"处理消息错误: {e}")
                await websocket.send(json.dumps({"code": 1, "message": f"内部错误: {str(e)}"}))
                
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"{remote_ip} 的连接已关闭")
    finally:
        pass

async def main():
    logger.info(f"正在启动 ROS2 后端服务器，端口 {PORT}")
    
    # 设置信号处理以便优雅停机
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(shutdown(sig, loop)))
        
    async with websockets.serve(handler, "0.0.0.0", PORT):
        await asyncio.Future()  # 永久运行

async def shutdown(sig, loop):
    logger.info(f"收到退出信号 {sig.name}...")
    cleanup_all()
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    logger.info(f"正在取消 {len(tasks)} 个未完成任务")
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass # 信号处理程序已处理清理退出
