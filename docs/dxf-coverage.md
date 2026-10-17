# DXF coverage

rigDxf is a **2D geometry subset** for plot / artboard hosts. It does not implement the Autodesk DXF reference (R12 through current). The parser is vendored **dxflib 3.26** (ASCII only).

If a drawing looks incomplete, check `Document::errors` after `loadFile` (or the `[PlotDoc] DXF` warn line). Counts look like `INSERT unresolved=2`. Types dxflib never parses do not appear there.

`loadFile` and `save` share one `Document`. Every kind we keep on read is written back as the same DXF type (or the closest typed entity). `flattenToParsed` / `documentFromParsed` is the plot IR bridge; `PlotDoc::exportDxf` goes through that IR so typed Arc / Ellipse / Spline become paths, then DXF polylines / hatches.

## Imported (model-space 2D)

| DXF type | Becomes | Notes |
|----------|---------|--------|
| LINE | Path | Two-point path. Exploded drawings join later if `dxfJoinConnected` is on. |
| LWPOLYLINE / POLYLINE | Path + Arc | Bulge segments become typed Arc. Z / mesh / polyface flags ignored (XY only). Geometric width (43 constant, else 40/41) becomes `strokeWidthMm`. |
| CIRCLE | Ellipse | `rx == ry`. |
| ARC | Arc | Counter-clockwise sweep, DXF rule. |
| ELLIPSE | Ellipse or Arc | Full turn vs partial parameter range. |
| SPLINE | Spline | Control points, knots, optional weights and fit points. |
| SOLID / TRACE / 3DFACE | Path | Four corners, closed, `filled`. |
| INSERT + named BLOCK | Expanded entities | Flattened through the insert TRS and row/col array. Nested INSERT up to 16. |
| HATCH | Path / Arc / Ellipse | Solid fill becomes one filled path. Pattern hatch keeps boundary strokes only. |
| LAYER | Layer name + colour + visible | ACI and color24. Off / frozen maps to `CPlotLayer.visible`. |
| POINT | Point entity | Dropped on `flattenToParsed` (no 2-point path). Invisible in svgEditor / plot. |
| TEXT / MTEXT / ATTRIB | Text | Live `TextItem` when `dxfImportText` is on. MTEXT formatting codes are stripped. Rotation is stored; the artboard still draws unrotated until the text present honors it. |
| ARCALIGNEDTEXT | Text | Mid-arc placement; not a true arc-text entity. |
| DIMENSION | Path + Text | `*D` block graphics (and fallback measurement text) when `dxfImportDimensions` is on. `dxfDimensionsOwnLayer` moves them onto a layer named Dimensions. |
| LEADER | Path | Vertex polyline (arrow head not rebuilt). |
| IMAGE + IMAGEDEF | Image | `ImageItem` href / pos / size. Display size is `|u|*width` by `|v|*height`. |
| XLINE / RAY | XLine / Ray | Flatten clips to the drawing AABB (padded). Save writes XLINE / RAY. |
| Lineweight | `strokeWidthMm` | Group 370 / 100 = mm when the entity carries a positive width. Polyline geometric width wins when both are set. |

## Heard by dxflib, still dropped (counted)

These fire a callback. We increment `doc.errors` and keep nothing.

None of the drawable 2D callbacks are counted-dropped now. Remaining skips are degenerate geometry (`ELLIPSE_DEGENERATE`, `HATCH_EDGE`) and unresolved INSERT.

## Never parsed (silent)

dxflib has no callback. The group is consumed as unknown and **does not increment** `doc.errors`.

Common on modern CAD exports:

- ATTDEF
- MLINE
- MLEADER / MULTILEADER
- TOLERANCE
- HELIX
- MESH (and polyface / polygon mesh as first-class types)
- 3DSOLID, BODY, REGION, SURFACE and the derived surface types
- VIEWPORT
- WIPEOUT
- SHAPE
- ACAD_TABLE / TABLE
- OLEFRAME / OLE2FRAME
- UNDERLAY / PDFUNDERLAY / DGNUNDERLAY / DWFUNDERLAY
- LIGHT, CAMERA
- ARC_DIMENSION, LARGE_RADIAL_DIMENSION
- ACAD_PROXY_ENTITY
- Binary DXF (not ASCII)

A file that is "mostly tables and MTEXT" can load as a few lines and look empty.

## Blocks, space, and coordinates

| Topic | What happens |
|-------|----------------|
| Named BLOCK / INSERT | Expanded flat. Block structure is not kept. |
| Anonymous `*` blocks | `*D` (dimension graphics) collected when dimensions import is on. `*X` / `*U` / paper-space containers stay skipped. |
| Unresolved INSERT | Counted (`INSERT unresolved=N`) when the block name was never stored. |
| Paper space | `*Paper_Space` blocks skipped. `inPaperSpace` is not used as a filter on ENTITIES. |
| Z / extrusion / OCS | XY only. Group 210/220/230 (extrusion) and elevation are ignored. Entities not in WCS land in the wrong place or collapse. |
| UCS / `$INSUNITS` | Header variables are not applied. Numbers are treated as document millimetres. |
| Linetype | Name is ignored. Dash patterns are not stored. |
| XDATA / XRECORD / DICTIONARY | Ignored. |

## Write

`save` writes **flat R2000 ASCII**: one ENTITIES section, no BLOCK/INSERT tree, no paper space.

| Document kind | DXF |
|---------------|-----|
| Path (2-point) | LINE |
| Path (open / closed) | LWPOLYLINE |
| Path (`filled`) | HATCH SOLID |
| Arc / Ellipse / Spline / Point / Text | ARC / ELLIPSE / SPLINE / POINT / TEXT |
| Image | IMAGE + IMAGEDEF |
| XLine / Ray | XLINE / RAY |
| Layer.visible | LAYER off / frozen flags |

`documentFromParsed` rebuilds Paths / Texts / Images from plot IR so File > Export DXF matches what the artboard shows. Typed curves flatten first.

## Not a goal

Full AutoCAD / BricsCAD / LibreCAD fidelity, 3D ACIS/ASM bodies, annotative text, and dimension style tables. Those need a different parser and a different document model. This pack stays 2D paths, fills, text, and images a plotter or artboard can present.
