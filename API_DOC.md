# 机器人控制 API 文档

**协议**: WebSocket
**端口**: 8999

## 概述
此 API 用于在不同功能模式（建图、导航）之间切换机器人。它运行在机器人的辅助 WebSocket 服务器上。

## 指令

### 开启建图模式
**请求**:
```json
{
  "command": "start_mapping"
}
```

**响应**:
```json
{
  "code": 0,
  "message": "success"
}
```


### 开启导航模式
**请求**:
```json
{
  "command": "start_navigation",
  "map_name": "map_name"
}
```

**响应**:
```json
{
  "code": 0,
  "message": "success"
}
```

### 停止所有进程
**请求**:
```json
{
  "command": "stop_all"
}
```

**响应**:
```json
{
  "code": 0,
  "message": "所有进程已停止"
}
```

### 保存地图
**请求**:
```json
{
  "command": "save_map",
  "map_name": "map_name"
}
```

**响应**:
```json
{
  "code": 0,
  "message": "map saved successfully"
}
```

### 获取地图列表
**请求**:
```json
{
  "command": "get_maps"
}
```

**响应**:
```json
{
  "code": 0,
  "message": "success",
  "maps": ["map1", "map2", "map3"]
}
```

## 错误处理
如果指令执行失败，`code` 将不为 0。

**错误示例**:
```json
{
  "code": 1,
  "message": "Failed to start mapping: Process already running"
}
```
