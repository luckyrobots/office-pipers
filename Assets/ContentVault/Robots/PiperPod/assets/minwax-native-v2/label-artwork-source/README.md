# Clean Minwax sleeve artwork

This is a new editable reconstruction of the actual **Minwax Stainable Wood Filler, 6 FL OZ / 177 mL** front label in the two supplied photographs. It contains no scanned mesh, scanned UV atlas, baked reflection, baked shadow, or pasted product photograph. The existing contact-qualified task asset is untouched.

The tube body and separate cap remain separate native POD geometry. The printed sleeve uses this artwork; the cap, nozzle, threads, and shoulder receive their own plastic materials. Label text is not embossed geometry.

## Files

- `minwax-label-master.svg`: resolution-independent vector outlines for every letter and graphic. It requires no installed font to render.
- `BuildLabel.cs.txt` and `Build-Label.ps1`: editable artwork source and offline Windows rasterizer. Text copy and layout can be changed here and regenerated without Python or downloaded fonts.
- `resources/minwax-label-basecolor.png`: 4096 Ã— 4096 full-wrap sRGB albedo.
- `resources/minwax-label-metallic-roughness.png`: 1024 Ã— 1024 packed glTF map. G is roughness, B is metallic (zero); R is unused. No light or reflection is in either map.
- `front-artwork-preview.png`: flat front artwork preview, not a photograph or final rendered model.
- `materials/minwax_printed_sleeve.yaml`: native POD SetMaterial group exposing `label_basecolor_texture` and `label_metallic_roughness_texture`.
- `materials/minwax_black_cap.yaml`: native POD charcoal plastic material for the separate ribbed cap.

## UV and material contract

Model local Y is up, front is +Z. U=.5 is the front centre and U=0/1 is the rear seam. V=(height above cap bottom in mm âˆ’ 33)/160. PNG top is the crimp (height193mm/V1); PNG bottom is sleeve bottom (height33mm/V0). Verify the native image-V convention when binding the maps; do not reverse the wording. The curved main sleeve ends at height181mm/V.925; the separate crimp occupies181â€“193mm and may use the matching gold/black colors directly.

The printed plastic is dielectric. Base color factor is white. With a metallic/roughness map bound, both POD metallic and roughness factors are1, so the map gives absolute values: metallic0, roughness about.34. IOR1.47, subtle coat.08/roughness.30. The yellow is printed/polymer gold, not metallic foil. Cap uses linear RGB(.010,.012,.011), metallic0, roughness.44 and small coat.05. Ribs and threads belong to the native geometry, not baked texture shading.

## Source fidelity

Front wording and capacity are transcribed from `1-Photo-1.jpg` and checked against `2-Photo-2.jpg`, attachment group `F0350AAD-7AC1-4BB6-A99D-7B4160CF7599`. Serif lettering is reconstructed with vector outlines; it is not an exact proprietary font master. Colors are estimates from the supplied photographs, not colorimeter readings.

The old scan atlas was inspected as reference only. Its reverse-label microcopy and barcode were not reliably readable. The reverse therefore stays the same gold plastic and does not invent safety instructions, ingredients, legal claims, batch numbers, or barcode digits. A straight-on back photograph can complete that artwork later.

## Rebuild

Run `powershell -NoProfile -ExecutionPolicy Bypass -File Build-Label.ps1` from any directory on Windows. This uses System.Drawing only as an authoring tool. The installed plugin and model need only the generated standard PNG resources and native POD material graph; users do not install Python or the rasterizer.

The generator is stored as `.cs.txt` below project Assets because the Engine auto-registers every `.cs` as a runtime script. This file is offline artwork source only; the installed task uses generated PNGs and has no System.Drawing runtime dependency.
