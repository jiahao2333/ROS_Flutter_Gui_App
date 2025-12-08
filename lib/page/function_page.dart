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
            AppLocalizations.of(context)?.function_selection ??
            'Function Selection'),
        centerTitle: true,
        actions: [
          IconButton(
            icon: const Icon(Icons.stop_circle_outlined, color: Colors.red),
            tooltip: AppLocalizations.of(context)!.stop,
            onPressed: _handleStopAll,
          ),
        ],
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
                title: AppLocalizations.of(context)!
                    .enter_mapping_mode, // TODO: Add localization
                icon: Icons.map_outlined,
                onPressed: _handleStartMapping,
              ),
              const SizedBox(height: 24),
              _buildFunctionButton(
                context,
                title: AppLocalizations.of(context)!
                    .enter_navigation_mode, // TODO: Add localization
                icon: Icons.navigation_outlined,
                onPressed: _handleStartNavigation,
              ),
              const SizedBox(height: 24),
              _buildFunctionButton(
                context,
                title: AppLocalizations.of(context)!
                    .original_map, // TODO: Add localization
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
      _showError(
          "${AppLocalizations.of(context)!.failed_to_start_mapping}: $e");
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
        _showError(AppLocalizations.of(context)!.no_maps_available);
        return;
      }
      _showMapSelectionDialog(maps);
    } catch (e) {
      _showError("${AppLocalizations.of(context)!.failed_to_get_map_list}: $e");
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  void _showMapSelectionDialog(List<String> maps) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(AppLocalizations.of(context)!.select_map),
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
      _showError(
          "${AppLocalizations.of(context)!.failed_to_start_navigation}: $e");
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  void _showSuccessDialog({required bool isMapping}) {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        title: Text(AppLocalizations.of(context)!.execution_successful),
        content: Text(AppLocalizations.of(context)!.enter_map_confirm),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.pop(context); // Close dialog
            },
            child: Text(AppLocalizations.of(context)!.cancel),
          ),
          TextButton(
            onPressed: () {
              Navigator.pop(context); // Close dialog
              Navigator.pushNamed(context, "/map",
                  arguments: {'isMapping': isMapping});
            },
            child: Text(AppLocalizations.of(context)!.ok),
          ),
        ],
      ),
    );
  }

  Future<void> _handleStopAll() async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(AppLocalizations.of(context)!.stop_all_processes),
        content: Text(AppLocalizations.of(context)!.stop_all_processes_confirm),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: Text(AppLocalizations.of(context)!.cancel),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            child: Text(AppLocalizations.of(context)!.stop,
                style: TextStyle(color: Colors.red)),
          ),
        ],
      ),
    );

    if (confirmed == true) {
      setState(() => _isLoading = true);
      try {
        await _service.stopAll();
        if (!mounted) return;
        toastification.show(
          context: context,
          type: ToastificationType.success,
          title: Text(AppLocalizations.of(context)!.success),
          description:
              Text(AppLocalizations.of(context)!.all_processes_stopped),
          autoCloseDuration: const Duration(seconds: 3),
        );
      } catch (e) {
        _showError(
            "${AppLocalizations.of(context)!.failed_to_stop_processes}: $e");
      } finally {
        if (mounted) setState(() => _isLoading = false);
      }
    }
  }

  void _showError(String message) {
    if (!mounted) return;
    toastification.show(
      context: context,
      type: ToastificationType.error,
      style: ToastificationStyle.fillColored,
      title: Text(AppLocalizations.of(context)!.error),
      description: Text(message),
      autoCloseDuration: const Duration(seconds: 4),
    );
  }
}
