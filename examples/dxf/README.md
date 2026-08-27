# Load DXF

![preview](img/preview.png)

Pan/zoom preview of a mounting-plate fixture (`data/demo.dxf`): rounded plate, holes, arc/ellipse/spline, INSERT blocks, and a faceted 24-gon for Promote Curves.

## Build-n-Run

```bash
cmake -S . -B build
cmake --build build --target dxf
./build/bin/dxf
```

Scroll to zoom, LMB to pan. **File > Open...** loads a `.dxf`. Sidebar: layers, Mirror Y, Complement Arcs, **Promote Curves** (opt-in faceted recovery), Fit View.

