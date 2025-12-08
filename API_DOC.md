# Robot Control API Documentation

**Protocol**: WebSocket
**Port**: 8999

## Overview
This API is used to switch the robot between different functional modes (Mapping, Navigation). It runs on a secondary WebSocket server on the robot.

## Commands

### Start Mapping Mode
**Request**:
```json
{
  "command": "start_mapping"
}
```

**Response**:
```json
{
  "code": 0,
  "message": "success"
}
```

### Start Navigation Mode
**Request**:
```json
{
  "command": "start_navigation"
}
```

**Response**:
```json
{
  "code": 0,
  "message": "success"
}
```

## Error Handling
If the command fails, the `code` will be non-zero.

**Example Error**:
```json
{
  "code": 1,
  "message": "Failed to start mapping: Process already running"
}
```
