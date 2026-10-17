# rigDxf

![preview](examples/dxf/img/preview.png)

ASCII DXF read/write as **typed 2D POD entities** (paths, arcs, ellipses, splines, text, images, construction lines), with a `ParsedPathDoc` / `CPaths` flatten lane for the plot pipeline. `save` writes the same kinds `loadFile` keeps.

Vendors [dxflib](third_party/dxflib/) 3.26 (**GPL-2.0-or-later**). This pack is a GPL leaf.

Depends on **rigPlotComponent** (+ **rigComponent** for `CArc` / `CEllipse` / `CSpline`). Headless gate: `tools/dxf_smoke.cpp`.

**This is not the Autodesk DXF spec.** The parse ceiling is dxflib's entity list. Anything dxflib does not emit is dropped with no count. Types we hear but do not keep are counted in `Document::errors` (and logged by `PlotDoc::importDxf`).

Coverage detail: [docs/dxf-coverage.md](docs/dxf-coverage.md).

## Curve promotion

**Load is non-destructive** - no heuristic fitting runs automatically.

| Path | When | Default |
|------|------|---------|
| **Bulge to Arc** | During load | Always - LWPOLYLINE/POLYLINE bulge segments become typed `Arc` entities (straight runs stay `Path`) |
| **Faceted circle / arc** | `promoteCurves(doc, settings)` only | **Off** (`promoteFacetedCircles` / `promoteFacetedArcs` = false) |

`promoteBulgeArcCircles` / `promoteBulgeArcs` stay on the settings struct; after load they have nothing left to do because bulges are already arcs.

```cpp
rigkit::dxf::CurvePromotionSettings settings;
settings.promoteFacetedCircles = true; // opt in - hexagons stay polylines unless you ask
settings.promoteFacetedArcs = true;
auto result = rigkit::dxf::promoteCurves(doc, settings);
// result.circlesPromoted, result.arcsPromoted, result.entitiesExamined
```

The example’s **Faceted** layer is a 24-gon so **Promote Curves** in the sidebar has something to recover. Shortcut: `P`.

## API sketch

```cpp
rigkit::dxf::Document doc;
rigkit::dxf::loadFile("drawing.dxf", doc);
rigkit::dxf::flipY(doc);
rigkit::dxf::complementArcs(doc);
// optional: promoteCurves(doc, settings) - faceted recovery only; never implied by load

rigkit::plot::ParsedPathDoc parsed;
rigkit::dxf::flattenToParsed(doc, parsed); // plot lane

rigkit::dxf::save(doc, "out.dxf"); // same kinds loadFile produced
```

`loadFile(path, ParsedPathDoc&)` still works - load + flatten in one call.

[API/docs](https://rigkid.github.io/rigDxf/)
