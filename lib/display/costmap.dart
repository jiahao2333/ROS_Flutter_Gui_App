import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flame/components.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

class CostMapComponent extends PositionComponent {
  final double opacity;
  final bool isGlobal;
  final RosChannel? rosChannel;

  ui.Image? _cachedImage;
  OccupancyMap? _lastProcessedMap;
  double _lastOpacity = 0.3;
  bool _isProcessing = false;

  CostMapComponent({
    this.opacity = 0.2,
    this.isGlobal = false,
    this.rosChannel,
  }) {
    _lastOpacity = opacity;
  }

  @override
  void onRemove() {
    _cachedImage?.dispose();
    _cachedImage = null; // 设置为null防止后续绘制
    _lastProcessedMap = null;
    super.onRemove();
  }

  Future<void> updateCostMap(OccupancyMap costMap) async {
    // 检查是否需要重新处理
    if (_lastProcessedMap == costMap &&
        _lastOpacity == opacity &&
        _cachedImage != null) {
      return;
    }

    if (_isProcessing) return;
    _isProcessing = true;

    try {
      if (costMap.data.isEmpty ||
          costMap.mapConfig.width == 0 ||
          costMap.mapConfig.height == 0) {
        _cachedImage?.dispose();
        _cachedImage = null;
        _lastProcessedMap = costMap;
        _lastOpacity = opacity;
        return;
      }

      // 如果是局部代价地图，更新组件位置
      if (!isGlobal &&
          rosChannel != null &&
          rosChannel!.map.value.height() > 0) {
        // 计算在全局地图中的位置
        // 局部代价地图的origin是其在世界坐标系下的位置
        vm.Vector2 originWorld =
            vm.Vector2(costMap.mapConfig.originX, costMap.mapConfig.originY);

        // 转换为栅格坐标
        vm.Vector2 originGrid = rosChannel!.map.value.xy2idx(originWorld);

        // Flame中position对应组件的左上角
        // xy2idx返回的是栅格坐标，y轴向下增加
        // 但是costMap.mapConfig.origin通常是左下角（ROS习惯）
        // 且局部代价地图在ROS中也是栅格化的
        // 需要注意：xy2idx 转换后的坐标系 y轴是反转的？取决于 occupancy_map.dart 的实现

        // OccupancyMap.xy2idx implementation:
        // double y = height() - (mapPoint.y - mapConfig.originY) / mapConfig.resolution;
        // height() 是全局地图的高度

        position =
            Vector2(originGrid.x, originGrid.y - costMap.mapConfig.height);
      } else {
        position = Vector2.zero();
      }

      final int width = costMap.mapConfig.width;
      final int height = costMap.mapConfig.height;

      List<int> costMapColors = costMap.getCostMapData();

      final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(
        Uint8List.fromList(costMapColors),
      );

      final ui.ImageDescriptor descriptor = ui.ImageDescriptor.raw(
        buffer,
        width: width,
        height: height,
        pixelFormat: ui.PixelFormat.rgba8888,
      );

      final ui.Codec codec = await descriptor.instantiateCodec();
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      final ui.Image image = frameInfo.image;

      buffer.dispose();
      descriptor.dispose();
      codec.dispose();

      _cachedImage?.dispose();
      _cachedImage = image;
      _lastProcessedMap = costMap;
      _lastOpacity = opacity;
    } catch (e) {
      print('Error processing costmap: $e');
    } finally {
      _isProcessing = false;
    }
  }

  @override
  void render(Canvas canvas) {
    // 添加额外的安全检查
    if (_cachedImage == null || !isMounted) {
      return;
    }

    try {
      final Paint paint = Paint()
        ..colorFilter = ColorFilter.mode(
          Colors.white.withOpacity(opacity),
          BlendMode.modulate,
        );

      canvas.drawImage(_cachedImage!, Offset.zero, paint);
    } catch (e) {
      print('Error rendering costmap: $e');
      // 如果绘制失败，清理缓存的图像
      _cachedImage?.dispose();
      _cachedImage = null;
    }
  }
}
