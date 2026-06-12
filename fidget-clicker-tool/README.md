# Fidget Clicker Parametric Tool

Turns any 3D model (.stl / .obj) into a printable mechanical fidget clicker
using the Austin's Lab MakerWorld templates (MX switches).
Everything runs locally - your models never leave this machine.

## Run

Double-click `run.bat` (or `python app.py`). Your browser opens at
http://127.0.0.1:5723.

First-time setup if needed: `pip install -r requirements.txt`

## Use

1. Pick your model file (Z must be up, flat bottom).
   - If the file contains TWO separate pieces, the upper one automatically
     becomes the cap and the lower one the switch holder - no cutting.
2. Tune parameters (defaults are sensible):
   - **Cap height** - `auto` detects the natural seam (lid line, knot neck);
     or type a number in mm from the top.
   - **Look** - `Flush` (default): the switch is buried ~13.8 mm so the
     unpressed clicker looks exactly like the original object, and pressing
     sinks the cap into a hidden pocket. `Floating`: tutorial style, the cap
     hovers above the body on the switch stem.
   - **Rest float** - how high the cap rides on an unpressed MX switch.
     `auto` = 13.8 mm (from the Cherry MX datasheet + template geometry).
     If a test print rests visibly high/low, adjust by +-0.5 and re-run.
   - **Size mode** - `Minimize` (default): shrink/grow to the smallest
     clicker that still fits the switch; or keep size / never rescale.
   - **Solid below switch** - material kept under the switch pocket (you
     drill wire holes yourself).
3. Click **Run & Preview**. The 3D view shows the holder and the cap at its
   real resting position, ghost overlays of the switch housing (green) and
   keycap stem (orange), the cut plane (blue), and a mm dimensions readout.
   Warnings appear in the log panel.
4. Click **Export STL** - writes `{prefix}_top.stl` and `{prefix}_bottom.stl`
   into `output/`.

## Printing & assembly

- `_bottom.stl` prints as-is (flat base down). The switch socket is part of
  the print - no supports needed in the pocket.
- `_top.stl` is exported cut-face-up (MX socket facing up) so the socket
  prints cleanly; depending on the model shape you may want supports under
  the overhangs - check in Bambu Studio.
- Push an MX switch into the socket from above, then press the cap onto the
  switch stem.

## Templates

Reads the template STLs from
`..\Mechanical+Switch+Keycap+Fidget+Clicker+Template+\`
(keycap connector, switch housing, housing negative block). Path is set in
`pipeline.py` (`TEMPLATE_DIR`).
