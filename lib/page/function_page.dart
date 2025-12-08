import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as status;
import 'package:toastification/toastification.dart';

class FunctionPage extends StatefulWidget {
  const FunctionPage({Key? key}) : super(key: key);

  @override
  _FunctionPageState createState() => _FunctionPageState();
}

class _FunctionPageState extends State<FunctionPage> {
  bool _isLoading = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(AppLocalizations.of(context)?.connect_robot ??
            'Function Selection'),
        centerTitle: true,
      ),
      body: Center(
        child: Padding(
          padding: const EdgeInsets.all(24.0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              _buildFunctionButton(
                context,
                title: "Enter Mapping Mode", // TODO: Add localization
                icon: Icons.map_outlined,
                onPressed: () => _handleModeCommand("start_mapping"),
              ),
              const SizedBox(height: 24),
              _buildFunctionButton(
                context,
                title: "Enter Navigation Mode", // TODO: Add localization
                icon: Icons.navigation_outlined,
                onPressed: () => _handleModeCommand("start_navigation"),
              ),
              const SizedBox(height: 24),
              _buildFunctionButton(
                context,
                title: "Original Map", // TODO: Add localization
                icon: Icons.map,
                onPressed: () {
                  Navigator.pushNamed(context, "/map");
                },
                isPrimary: false,
              ),
            ],
          ),
        ),
      ),
      // Loading overlay
      floatingActionButton: _isLoading
          ? Container(
              color: Colors.black54,
              child: const Center(
                child: CircularProgressIndicator(),
              ),
            )
          : null,
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
    );
  }

  Widget _buildFunctionButton(
    BuildContext context, {
    required String title,
    required IconData icon,
    required VoidCallback onPressed,
    bool isPrimary = true,
  }) {
    final colorScheme = Theme.of(context).colorScheme;
    return SizedBox(
      height: 80,
      child: ElevatedButton(
        onPressed: _isLoading ? null : onPressed,
        style: ElevatedButton.styleFrom(
          backgroundColor:
              isPrimary ? colorScheme.primary : colorScheme.secondary,
          foregroundColor:
              isPrimary ? colorScheme.onPrimary : colorScheme.onSecondary,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
          ),
          elevation: 4,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, size: 32),
            const SizedBox(width: 16),
            Text(
              title,
              style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _handleModeCommand(String command) async {
    setState(() => _isLoading = true);

    final ip = globalSetting.robotIp;
    // Port for the control service is 8999
    final uri = Uri.parse("ws://$ip:8999");

    WebSocketChannel? channel;

    try {
      channel = WebSocketChannel.connect(uri);

      // Wait for connection to be ready? WebSocketChannel.connect is synchronous in creation but async in connection.
      // We'll just send and listen.

      final request = jsonEncode({"command": command});
      channel.sink.add(request);

      // Listen for first response
      final completer = Completer<void>();

      final subscription = channel.stream.listen(
        (message) {
          try {
            final data = jsonDecode(message);
            if (data['code'] == 0) {
              _showSuccessDialog();
            } else {
              _showError("Operation failed: ${data['message']}");
            }
          } catch (e) {
            _showError("Invalid response format: $message");
          } finally {
            completer.complete();
            channel?.sink.close(status.normalClosure);
          }
        },
        onError: (error) {
          _showError("Connection error: $error");
          completer.complete();
        },
        onDone: () {
          if (!completer.isCompleted) completer.complete();
        },
      );

      // Timeout after 5 seconds
      await completer.future.timeout(
        const Duration(seconds: 5),
        onTimeout: () {
          subscription.cancel();
          channel?.sink.close(status.goingAway);
          _showError("Request timed out");
        },
      );
    } catch (e) {
      _showError("Failed to connect: $e");
    } finally {
      setState(() => _isLoading = false);
    }
  }

  void _showSuccessDialog() {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        title: const Text("Execution Successful"),
        content: const Text("Would you like to enter the map?"),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.pop(context); // Close dialog
            },
            child: const Text("No"),
          ),
          TextButton(
            onPressed: () {
              Navigator.pop(context); // Close dialog
              Navigator.pushNamed(context, "/map");
            },
            child: const Text("Yes"),
          ),
        ],
      ),
    );
  }

  void _showError(String message) {
    if (!mounted) return;
    toastification.show(
      context: context,
      type: ToastificationType.error,
      style: ToastificationStyle.fillColored,
      title: Text("Error"),
      description: Text(message),
      autoCloseDuration: const Duration(seconds: 4),
    );
  }
}
