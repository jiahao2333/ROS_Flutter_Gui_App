import 'dart:async';
import 'dart:convert';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as status;

class RobotControlService {
  static final RobotControlService _instance = RobotControlService._internal();

  factory RobotControlService() {
    return _instance;
  }

  RobotControlService._internal();

  Future<Map<String, dynamic>> _sendCommand(
      Map<String, dynamic> command) async {
    final ip = globalSetting.robotIp;
    final uri = Uri.parse("ws://$ip:8999");
    WebSocketChannel? channel;

    final completer = Completer<Map<String, dynamic>>();

    try {
      channel = WebSocketChannel.connect(uri);

      // Listen FIRST before sending to ensure we don't miss the message,
      // although connect() returns a channel that might not be connected yet,
      // the stream subscription should handle buffering.

      final subscription = channel.stream.listen(
        (message) {
          try {
            final data = jsonDecode(message);
            if (!completer.isCompleted) {
              completer.complete(data);
            }
          } catch (e) {
            if (!completer.isCompleted) {
              completer.completeError("Invalid response format: $message");
            }
          } finally {
            channel?.sink.close(status.normalClosure);
          }
        },
        onError: (error) {
          if (!completer.isCompleted) {
            completer.completeError("Connection error: $error");
          }
        },
        onDone: () {
          if (!completer.isCompleted) {
            // If connection closed without message
            completer.completeError("Connection closed without response");
          }
        },
      );

      channel.sink.add(jsonEncode(command));

      // Timeout logic
      return await completer.future.timeout(
        const Duration(seconds: 5),
        onTimeout: () {
          subscription.cancel();
          channel?.sink.close(status.goingAway);
          throw TimeoutException("Request timed out");
        },
      );
    } catch (e) {
      if (channel != null) {
        channel.sink.close(status.internalServerError);
      }
      rethrow;
    }
  }

  Future<void> startMapping() async {
    final response = await _sendCommand({"command": "start_mapping"});
    if (response['code'] != 0) {
      throw Exception(response['message'] ?? "Unknown error");
    }
  }

  Future<void> startNavigation(String mapName) async {
    final response = await _sendCommand({
      "command": "start_navigation",
      "map_name": mapName,
    });
    if (response['code'] != 0) {
      throw Exception(response['message'] ?? "Unknown error");
    }
  }

  Future<void> saveMap(String mapName) async {
    final response = await _sendCommand({
      "command": "save_map",
      "map_name": mapName,
    });
    if (response['code'] != 0) {
      throw Exception(response['message'] ?? "Unknown error");
    }
  }

  Future<List<String>> getMapList() async {
    final response = await _sendCommand({"command": "get_maps"});
    if (response['code'] != 0) {
      throw Exception(response['message'] ?? "Unknown error");
    }
    final maps = response['maps'];
    if (maps is List) {
      return maps.map((e) => e.toString()).toList();
    } else {
      return [];
    }
  }
}
