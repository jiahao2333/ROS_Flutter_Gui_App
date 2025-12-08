import 'dart:async';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/service/robot_control_service.dart';
import 'package:toastification/toastification.dart';

class FunctionPage extends StatefulWidget {
  const FunctionPage({Key? key}) : super(key: key);

  @override
  _FunctionPageState createState() => _FunctionPageState();
}

class _FunctionPageState extends State<FunctionPage> {
  bool _isLoading = false;
  final _service = RobotControlService();

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
                onPressed: _handleStartMapping,
              ),
              const SizedBox(height: 24),
              _buildFunctionButton(
                context,
                title: "Enter Navigation Mode", // TODO: Add localization
                icon: Icons.navigation_outlined,
                onPressed: _handleStartNavigation,
              ),
              const SizedBox(height: 24),
              _buildFunctionButton(
                context,
                title: "Original Map", // TODO: Add localization
                icon: Icons.map,
                onPressed: () {
                  Navigator.pushNamed(context, "/map",
                      arguments: {'isMapping': false});
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

  Future<void> _handleStartMapping() async {
    setState(() => _isLoading = true);
    try {
      await _service.startMapping();
      if (!mounted) return;
      _showSuccessDialog(isMapping: true);
    } catch (e) {
      _showError("Failed to start mapping: $e");
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  Future<void> _handleStartNavigation() async {
    setState(() => _isLoading = true);
    try {
      final maps = await _service.getMapList();
      if (!mounted) return;
      // Show map selection dialog
      if (maps.isEmpty) {
        _showError("No maps available");
        return;
      }
      _showMapSelectionDialog(maps);
    } catch (e) {
      _showError("Failed to get map list: $e");
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  void _showMapSelectionDialog(List<String> maps) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Select Map"),
        content: SizedBox(
          width: double.maxFinite,
          child: ListView.builder(
            shrinkWrap: true,
            itemCount: maps.length,
            itemBuilder: (context, index) {
              return ListTile(
                title: Text(maps[index]),
                onTap: () {
                  Navigator.pop(context); // Close selection dialog
                  _confirmStartNavigation(maps[index]);
                },
              );
            },
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text("Cancel"),
          ),
        ],
      ),
    );
  }

  Future<void> _confirmStartNavigation(String mapName) async {
    setState(() => _isLoading = true);
    try {
      await _service.startNavigation(mapName);
      if (!mounted) return;
      _showSuccessDialog(isMapping: false);
    } catch (e) {
      _showError("Failed to start navigation: $e");
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  void _showSuccessDialog({required bool isMapping}) {
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
              Navigator.pushNamed(context, "/map",
                  arguments: {'isMapping': isMapping});
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
      title: const Text("Error"),
      description: Text(message),
      autoCloseDuration: const Duration(seconds: 4),
    );
  }
}
